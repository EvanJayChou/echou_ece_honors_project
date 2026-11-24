#!/usr/bin/env bash
# nrp_retrain.sh — submit an offline 3DGS retrain of one run package to NRP (Nautilus).
#
# All cluster settings (namespace, image, PVC, GPU, resources) live in ONE file:
#   ros2_ws_offboard/deploy/nrp/nrp.env   (copy nrp.env.example; git-ignored)
# Only per-run values are arguments here. Design: docs/3dgs_design.md §6.2.
#
# Usage:
#   scripts/nrp_retrain.sh <run_id> --heldout <heldout_run_id> [--seeds N] [--dry-run]
#
#   --heldout ID   held-out session's run package (required)
#   --seeds N      training seeds = pods (default: NRP_NUM_SEEDS from nrp.env)
#   --dry-run      render and check the manifest, touch nothing on the cluster
#
# Environment: NRP_ENV  settings file (default: ros2_ws_offboard/deploy/nrp/nrp.env)
#
# Before anything reaches the cluster, the rendered manifest must pass the project's NRP
# rules: a Job (no bare pods), restartPolicy Never, every pod capped at 1 h, no `sleep`
# in any command, 1 GPU per pod, limits within 20% of requests. These are conditions of
# our A100 access; the script refuses to submit otherwise.

_TAG=nrp
source "$(dirname "$0")/common.sh"
trap - EXIT INT TERM   # common.sh's node-cleanup trap; this script starts no ROS nodes
set -euo pipefail

NRP_DIR="$OFFBOARD_WS/deploy/nrp"
NRP_ENV="${NRP_ENV:-$NRP_DIR/nrp.env}"
TEMPLATE="$NRP_DIR/retrain-job.yaml"

usage() { sed -n '2,21p' "$0" | sed 's/^# \{0,1\}//'; }

# ── Arguments ─────────────────────────────────────────────────────────────────
RUN_ID="" HELDOUT_ID="" SEEDS="" DRY_RUN=false
while [ $# -gt 0 ]; do
    case "$1" in
        --heldout)   HELDOUT_ID="${2:-}"; shift 2 ;;
        --heldout=*) HELDOUT_ID="${1#*=}"; shift ;;
        --seeds)     SEEDS="${2:-}"; shift 2 ;;
        --seeds=*)   SEEDS="${1#*=}"; shift ;;
        --dry-run)   DRY_RUN=true; shift ;;
        -h|--help)   usage; exit 0 ;;
        -*)          err "Unknown flag: $1"; usage; exit 1 ;;
        *)           if [ -n "$RUN_ID" ]; then err "Only one run_id allowed"; exit 1; fi
                     RUN_ID="$1"; shift ;;
    esac
done
[ -n "$RUN_ID" ]     || { err "Missing <run_id>"; usage; exit 1; }
[ -n "$HELDOUT_ID" ] || { err "Missing --heldout <heldout_run_id>"; exit 1; }

# ── Settings file ─────────────────────────────────────────────────────────────
if [ ! -f "$NRP_ENV" ]; then
    err "$NRP_ENV not found. Create it from the template:"
    err "  cp $NRP_DIR/nrp.env.example $NRP_DIR/nrp.env   # then fill in the required values"
    exit 1
fi
set -a
# shellcheck disable=SC1090
source "$NRP_ENV"
set +a
log "Settings: $NRP_ENV"

missing=()
for v in NRP_NAMESPACE NRP_IMAGE NRP_PVC NRP_GPU_RESOURCE NRP_GPU_PRODUCT \
         NRP_CPU NRP_MEMORY NRP_NUM_SEEDS NRP_JOB_DEADLINE_S NRP_CONFIG; do
    [ -n "${!v:-}" ] || missing+=("$v")
done
if [ ${#missing[@]} -gt 0 ]; then
    err "Set these in $NRP_ENV: ${missing[*]}"
    exit 1
fi

NUM_SEEDS="${SEEDS:-$NRP_NUM_SEEDS}"
id_re='^[A-Za-z0-9][A-Za-z0-9_.-]*$'
[[ "$RUN_ID" =~ $id_re ]]     || { err "Bad run_id '$RUN_ID' (letters, digits, _ . - only)"; exit 1; }
[[ "$HELDOUT_ID" =~ $id_re ]] || { err "Bad heldout id '$HELDOUT_ID'"; exit 1; }
[[ "$NUM_SEEDS" =~ ^[1-9][0-9]*$ ]]          || { err "Seeds must be a positive integer, got '$NUM_SEEDS'"; exit 1; }
[[ "$NRP_JOB_DEADLINE_S" =~ ^[1-9][0-9]*$ ]] || { err "NRP_JOB_DEADLINE_S must be a positive integer"; exit 1; }
[ "$RUN_ID" != "$HELDOUT_ID" ] || { err "run_id and heldout id are the same"; exit 1; }
[[ "$NRP_IMAGE" == *@sha256:* ]] || warn "NRP_IMAGE is not pinned by digest; benchmark retrains should use …@sha256:…"

CONFIG_PATH="$NRP_CONFIG"
[[ "$CONFIG_PATH" = /* ]] || CONFIG_PATH="$REPO_ROOT/$CONFIG_PATH"
[ -f "$CONFIG_PATH" ] || { err "Offline config not found: $CONFIG_PATH"; exit 1; }

for tool in envsubst kubectl sha256sum python3; do
    command -v "$tool" >/dev/null || { err "'$tool' not found"; exit 1; }
done

# ── Derived names ─────────────────────────────────────────────────────────────
CONFIG_SHA="$(sha256sum "$CONFIG_PATH" | cut -d' ' -f1)"
CONFIG_MAP="gs-offline-config-${CONFIG_SHA:0:12}"   # content-addressed: never edited in place
JOB_NAME="gs-retrain-$(echo "$RUN_ID" | tr 'A-Z_.' 'a-z--')"
JOB_NAME="${JOB_NAME:0:63}"
JOB_NAME="${JOB_NAME%-}"

export RUN_ID HELDOUT_ID NUM_SEEDS JOB_NAME CONFIG_SHA CONFIG_MAP

# ── Render ────────────────────────────────────────────────────────────────────
OUT_DIR="$LOG_DIR/nrp"
mkdir -p "$OUT_DIR"
RENDERED="$OUT_DIR/$JOB_NAME.yaml"
VARS='${JOB_NAME} ${RUN_ID} ${HELDOUT_ID} ${CONFIG_SHA} ${CONFIG_MAP} ${NUM_SEEDS}
      ${NRP_JOB_DEADLINE_S} ${NRP_GPU_PRODUCT} ${NRP_IMAGE} ${NRP_CPU} ${NRP_MEMORY}
      ${NRP_GPU_RESOURCE} ${NRP_PVC}'
envsubst "$VARS" < "$TEMPLATE" > "$RENDERED"
if grep -v '^[[:space:]]*#' "$RENDERED" | grep -q '\${'; then
    err "Unfilled \${VAR} left in $RENDERED"
    exit 1
fi

# ── Rule check (conditions of our NRP A100 access) ────────────────────────────
python3 - "$RENDERED" <<'PY' || { err "Manifest breaks the NRP rules; not submitting."; exit 1; }
import sys, yaml

def q(v):  # Kubernetes quantity -> float (enough for cpu/memory comparisons)
    s = str(v)
    units = {'m': 1e-3, 'Ki': 2**10, 'Mi': 2**20, 'Gi': 2**30, 'Ti': 2**40,
             'k': 1e3, 'M': 1e6, 'G': 1e9, 'T': 1e12}
    for u in sorted(units, key=len, reverse=True):
        if s.endswith(u):
            return float(s[:-len(u)]) * units[u]
    return float(s)

docs = [d for d in yaml.safe_load_all(open(sys.argv[1])) if d]
problems = []
for d in docs:
    if d.get('kind') != 'Job':
        problems.append(f"kind {d.get('kind')!r}: only Jobs are allowed (temporary pods)")
        continue
    pod = d['spec']['template']['spec']
    if pod.get('restartPolicy') != 'Never':
        problems.append('restartPolicy must be Never')
    dl = pod.get('activeDeadlineSeconds')
    if not isinstance(dl, int) or dl > 3600:
        problems.append(f'pod activeDeadlineSeconds must be <= 3600 (1 h), got {dl!r}')
    for c in pod.get('containers', []) + pod.get('initContainers', []):
        words = ' '.join(map(str, (c.get('command') or []) + (c.get('args') or []))).lower()
        if 'sleep' in words or 'infinity' in words:
            problems.append(f"container {c['name']}: 'sleep' is not allowed")
        if c.get('stdin') or c.get('tty'):
            problems.append(f"container {c['name']}: stdin/tty means interactive; not allowed")
        res = c.get('resources', {})
        req, lim = res.get('requests', {}), res.get('limits', {})
        gpus = [k for k in lim if 'nvidia.com/' in k]
        for k in gpus:
            if int(lim[k]) != 1 or int(req.get(k, 0)) != 1:
                problems.append(f"container {c['name']}: request exactly 1 GPU ({k})")
        for k in ('cpu', 'memory'):
            if k in req and k in lim and q(lim[k]) > 1.2 * q(req[k]):
                problems.append(f"container {c['name']}: {k} limit more than 20% above request")
for p in problems:
    print('  - ' + p, file=sys.stderr)
sys.exit(1 if problems else 0)
PY
ok "Rules: Job only, pods capped at 1 h, no sleep, 1 GPU/pod, limits within 20%"

KUBECTL=(kubectl --namespace "$NRP_NAMESPACE")
[ -n "${NRP_KUBE_CONTEXT:-}" ] && KUBECTL+=(--context "$NRP_KUBE_CONTEXT")

log "Job        $JOB_NAME  (run $RUN_ID, held-out $HELDOUT_ID, $NUM_SEEDS seed(s))"
log "Namespace  $NRP_NAMESPACE${NRP_KUBE_CONTEXT:+  (context $NRP_KUBE_CONTEXT)}"
log "GPU        $NRP_GPU_RESOURCE on $NRP_GPU_PRODUCT"
log "Config     ${CONFIG_PATH#"$REPO_ROOT"/}  → ConfigMap $CONFIG_MAP"
log "Manifest   $RENDERED"

# ── Dry run: nothing touches the cluster ──────────────────────────────────────
if [ "$DRY_RUN" = true ]; then
    kubectl create configmap "$CONFIG_MAP" --from-file=offline.yaml="$CONFIG_PATH" \
        --dry-run=client -o name >/dev/null
    kubectl apply --dry-run=client --validate=false -f "$RENDERED" -o name >/dev/null
    ok "Dry run passed. Nothing was submitted. Review $RENDERED, then rerun without --dry-run."
    exit 0
fi

# ── Submit ────────────────────────────────────────────────────────────────────
if "${KUBECTL[@]}" get job "$JOB_NAME" >/dev/null 2>&1; then
    err "Job $JOB_NAME already exists in $NRP_NAMESPACE."
    err "Finished seeds are skipped on re-run (DONE marker), so it is safe to delete and resubmit:"
    err "  ${KUBECTL[*]} delete job $JOB_NAME"
    exit 1
fi

"${KUBECTL[@]}" create configmap "$CONFIG_MAP" --from-file=offline.yaml="$CONFIG_PATH" \
    --dry-run=client -o yaml | "${KUBECTL[@]}" apply -f - >/dev/null
ok "ConfigMap $CONFIG_MAP ready"
"${KUBECTL[@]}" apply -f "$RENDERED"
ok "Submitted. Watch it with:"
echo "  ${KUBECTL[*]} get pods -l job-name=$JOB_NAME -w"
echo "  ${KUBECTL[*]} logs -l job-name=$JOB_NAME --prefix -f"
