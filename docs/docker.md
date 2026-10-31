# <div align="center">Running in Docker (Jetson Orin Nano)</div>

The whole rover stack can run as a container instead of a native install: ROS 2 Jazzy, Nav2, slam_toolbox, GLIM (GPU build), Livox-SDK2, DepthAI v3, and both workspaces built. The image is `linux/arm64` on the Orin Nano and targets a **JetPack 7.2** host, with CUDA available inside the container. The same Dockerfile also builds on an x86 GPU machine for the off-board half.

| File | Purpose |
|------|---------|
| [`docker/Dockerfile`](../docker/Dockerfile) | Image definition (base `nvidia/cuda:13.2.1-runtime-ubuntu24.04` + ROS 2 Jazzy from apt) |
| [`docker/compose.yaml`](../docker/compose.yaml) | Project `rover`: `rover` service (baked image, one-off runs) and `dev` service (persistent `rover-dev` container, live-mounted checkout) |
| [`docker/dev.sh`](../docker/dev.sh) | Start, enter, stop and reset the `rover-dev` container (§4) |
| [`docker/.env.example`](../docker/.env.example) | Template for the optional, git-ignored `docker/.env` (`USER_UID`, `USER_GID`, `RENDER_GID`, `ROS_DOMAIN_ID`) |
| [`.devcontainer/devcontainer.json`](../.devcontainer/devcontainer.json) | VS Code "Reopen in Container" on the `dev` service |
| [`docker/run.sh`](../docker/run.sh) | Same runtime options as Compose, with plain `docker run` |
| [`docker/entrypoint.sh`](../docker/entrypoint.sh) | Sources ROS 2 and both workspaces (your build, else the image's copy), then runs the command |
| [`.dockerignore`](../.dockerignore) | Keeps `.git`, build outputs, docs, maps and bags out of the build context |

---

## 1. Host setup (once, on the Jetson)

The container shares the host's network, devices and udev, so a few things still live on the host:

1. **JetPack 7.2.x** (Ubuntu 24.04): see [reproduction.md §2](reproduction.md#2-os-setup--jetpack-72-ubuntu-2404-on-jetson-orin-nano). ROS and CUDA don't need to be installed on the host; the container brings its own CUDA 13.2 user-space libraries, and the toolkit injects the host's GPU driver. A JetPack 6 host can't run the default image (its driver predates CUDA 13), so use the CPU-only build in §2 there.
2. **Docker Engine + Compose plugin + NVIDIA Container Toolkit** (from NVIDIA's [Orin Nano Docker setup](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/setup_docker.html)):
   ```bash
   sudo apt update
   sudo apt install -y nvidia-container curl          # NVIDIA Container Toolkit from the JetPack repo
   docker --version || curl -fsSL https://get.docker.com | sh
   sudo nvidia-ctk runtime configure --runtime=docker   # registers the "nvidia" runtime in /etc/docker/daemon.json
   sudo systemctl daemon-reload && sudo systemctl restart docker
   docker compose version
   sudo usermod -aG docker $USER      # log out and back in
   ```
   Compose and `docker/run.sh` select the runtime explicitly (`runtime: nvidia` / `--runtime nvidia`), so making it Docker's `default-runtime` is optional. It's only needed for GPU access during `docker build`, which this image doesn't use.
3. **udev rules** (udev doesn't run inside containers):
   ```bash
   sudo cp udev/99-rover.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
4. **Livox Ethernet link**: the static `192.168.1.50` connection from [reproduction.md §6](reproduction.md#6-livox-mid-360--network-setup-and-sdk) (the `nmcli` part only; the SDK is in the image).
5. **Power mode**: `sudo nvpmodel -q`, then pick the highest mode. The in-container scripts can't query this.

## 2. Build the image

From the repo root:

```bash
docker compose -f docker/compose.yaml build
# or: docker build -f docker/Dockerfile -t rover:jazzy .
```

Building natively on the Orin Nano takes a while (Livox-SDK2 plus the C++ packages). The `livox_ros_driver2` submodule doesn't need to be initialised, because the Dockerfile fetches the pinned commit when the folder is empty.

Pinned versions (override with `--build-arg`):

| Build arg | Default |
|-----------|---------|
| `BASE_IMAGE` | `nvidia/cuda:13.2.1-runtime-ubuntu24.04` |
| `ROS_APT_SOURCE_VERSION` | `1.3.0` (ros-infrastructure/ros-apt-source; installs the ROS apt repo + key) |
| `LIVOX_SDK2_REF` | `v1.3.1` (released with driver `1.2.6`; bump the two together) |
| `LIVOX_DRIVER_REF` | `13eb05e…` (the submodule commit, driver tag `1.2.6`) |
| `DEPTHAI_VERSION` | `3.11.0` |
| `GLIM_VERSION` / `GTSAM_POINTS_VERSION` | `1.2.2` / `1.2.2` (koide3 PPA; `-cuda13.1` packages on the CUDA base, plain ones on the CPU base; see [glim.md](glim.md)) |
| `NODE_MAJOR` | `24` (Node.js LTS from NodeSource; Claude Code needs 22 or later) |
| `CLAUDE_CODE_VERSION` | `2.1.310` (auto-updater disabled; bump this and rebuild to upgrade) |
| `USER_UID` / `USER_GID` | `1000` / `1000` (the non-root `echou` user; Compose takes them from `docker/.env`) |

The GitHub CLI (`gh`) comes from GitHub's apt repository, which only serves the newest release, so it isn't pinned. `docker inspect -f '{{index .Config.Labels "rover.versions"}}' rover:jazzy` prints the pinned versions an image was built with.

**Building on a faster x86 machine instead:** `docker buildx build --platform linux/arm64 -f docker/Dockerfile -t rover:jazzy --load .` (needs QEMU binfmt: `docker run --privileged --rm tonistiigi/binfmt --install arm64`). Then move it with `docker save rover:jazzy | ssh rover@<jetson> docker load`.

**Why a plain CUDA image, not `l4t-jetpack`:** JetPack 7.2 moves the Orin family onto the arm64-SBSA CUDA stack, the same one Arm servers use. Standard multi-arch NGC images now run on the Orin Nano. NVIDIA's own Orin Nano Docker guide tests with `nvcr.io/nvidia/pytorch`. Keep the base's CUDA version at or below the host's: JetPack 7.2.1 ships CUDA 13.2.2, and `cat /usr/local/cuda/version.json` on the host shows it. The `-runtime` flavour has the CUDA libraries without the compiler. Switch to `-devel` if you build CUDA code in the image, for example for on-robot 3DGS.

**GLIM and CUDA stay inside the image.** GLIM's prebuilt `-cuda13.1` packages run on this image's CUDA 13.2 runtime: CUDA 13.x shares one `libcudart.so.13` ABI, and the packages' `sm_80` kernels run on the Orin's `sm_87`. Installing them adds no CUDA toolkit, and nothing on the JetPack host is installed, upgraded or reconfigured. If GLIM ever has to be built from source, do it in a `-devel` base inside the image, not on the host. Details: [glim.md §1](glim.md#1-install-in-the-image).

**CPU-only image** (smaller; also runs on a JetPack 6 host; GLIM must then be launched with `use_gpu:=false`):

```bash
docker build -f docker/Dockerfile --build-arg BASE_IMAGE=ros:jazzy-ros-base -t rover:jazzy-cpu .
IMAGE=rover:jazzy-cpu DOCKER_RUNTIME=runc docker/run.sh
```

With Compose, set `DOCKER_RUNTIME=runc` on hosts without the NVIDIA toolkit, such as an x86 laptop with no NVIDIA GPU.

## 3. Run

Every launch script works unchanged inside the container:

```bash
docker compose -f docker/compose.yaml run --rm rover scripts/sensors.sh
docker compose -f docker/compose.yaml run --rm rover scripts/teleop.sh
docker compose -f docker/compose.yaml run --rm rover scripts/teleop_slam.sh
docker compose -f docker/compose.yaml run --rm rover scripts/explore.sh --all-onboard
docker compose -f docker/compose.yaml run --rm rover             # interactive shell
```

Without Compose: `docker/run.sh scripts/teleop_slam.sh` (or `docker/run.sh` for a shell).

Check that the GPU is visible inside the container (the CUDA driver API is injected by the toolkit, so this needs no extra packages):

```bash
docker/run.sh python3 -c "import ctypes; c = ctypes.CDLL('libcuda.so.1'); n = ctypes.c_int(); r = c.cuInit(0); c.cuDeviceGetCount(ctypes.byref(n)); print('cuInit', r, '| devices', n.value)"
# expected: cuInit 0 | devices 1
```

Pass settings through the environment, e.g. `ROS_DOMAIN_ID=5 VESC_PORT=/dev/ttyACM0 docker compose -f docker/compose.yaml run --rm rover scripts/teleop.sh`.

To open a second shell in a running container, for `ros2 topic hz` or a manual `teleop` while `--headless`:

```bash
docker exec -it $(docker ps -q --filter label=com.docker.compose.service=rover | head -1) /entrypoint.sh bash
```

For the dev container (§4) it's simply `docker/dev.sh shell`.

### What the runtime options do

| Option | Why |
|--------|-----|
| `network_mode: host` | DDS discovery with the GPU machine / laptop RViz, and Livox UDP on the host's `192.168.1.x` link |
| `runtime: nvidia` | NVIDIA Container Toolkit mounts the host's GPU driver (`libcuda`, Tegra device nodes) into the container. `DOCKER_RUNTIME=runc` turns it off |
| `ipc: host` | Fast DDS shared-memory transport with nodes on the host or in other containers |
| `privileged` + `/dev:/dev` | The OAK-D re-enumerates as a new USB device after its firmware boots, and the VESC can be re-plugged. Mounting all of `/dev` handles both, and keeps the host's `/dev/vesc` symlink visible |
| `init: true` | Ctrl+C reaches the scripts' cleanup trap, so every node stops |
| `/tmp/rover` volume | Component logs readable on the host |
| `maps/` volume | Maps saved to `ros2_ws/src/robot_slam/ros_data/maps/custom/` in the container land in `<repo>/maps/custom/` on the host |
| `bags/` volume (`rover` service) | `scripts/record.sh` output lands in `<repo>/bags/` on the host. The `dev` service already mounts the whole checkout |

### RViz

The image is `ros-base` (no GUI). Run RViz on a laptop with the same `ROS_DOMAIN_ID`, which also saves Jetson CPU. Host networking makes the container's topics visible on the LAN as if it were native.

## 4. Development inside the container

The `rover` service runs the code baked into the image, so a source edit means an image rebuild. For day-to-day work, use the `dev` service instead. It's one long-running container named `rover-dev`:

- **Your checkout is mounted at `/opt/echou-honors-dev`.** Edits on the host or in the container are the same files, so there's no rebuild and no copying. The image's own copy of the code stays at `/opt/rover`; it's read-only to you, and only used as a fallback until you build (step 7).
- **You're the `echou` user, not root.** Its user and group IDs match your host account, so `build/`, `install/`, `log/` and any new files in the checkout are owned by you on the host.
- **`/home/echou` is a Docker volume (`rover-home`).** It keeps your GitHub login, Claude Code login, git config, shell history and VS Code server between restarts. None of it is in the image or the repo.
- **The toolchain is in the image:** ROS 2 Jazzy, Node.js, the GitHub CLI (`gh`) and Claude Code (`claude`), all at the versions in §2.
- **The runtime options match the `rover` service:** host network, GPU, all of `/dev`, `/tmp/rover` and `maps/`. Launch scripts work the same way. The default `ROS_DOMAIN_ID` is `42` (see §5).

`docker/dev.sh` wraps the Docker commands:

| Command | What it does |
|---------|--------------|
| `docker/dev.sh up` | Start `rover-dev` in the background. Builds the image first if there is none |
| `docker/dev.sh shell` | Open bash inside it, with ROS 2 and the workspaces sourced. `docker/dev.sh shell <cmd>` runs one command |
| `docker/dev.sh down` | Stop and remove the container. The `rover-home` volume (your logins) stays |
| `docker/dev.sh status` | Is it running, and does the home volume exist? |
| `docker/dev.sh reset-home` | Delete the `rover-home` volume after a y/N prompt. This wipes all logins and settings |

### First-time setup

Do these steps once. Commands starting with `docker/` run **on the Jetson host**, from the repo root. The rest run **inside the container**.

1. **On the host, fetch the Livox driver submodule.** `scripts/build.sh onboard` needs it in your checkout:
   ```bash
   git submodule update --init ros2_ws/src/livox_ros_driver2
   ```
2. **On the host, check your user and group IDs** with `id -u`, `id -g` and `getent group render | cut -d: -f3`. The defaults are `1000`, `1000` and `993`, which match this Jetson. If any number differs, run `cp docker/.env.example docker/.env` and set `USER_UID`, `USER_GID` or `RENDER_GID` in `docker/.env` to your numbers. That file is git-ignored. The `render` group is what gives the non-root `echou` user the GPU.
3. **Start the container and open a shell:**
   ```bash
   docker/dev.sh up       # the first run builds the image; on the Orin Nano that takes a while
   docker/dev.sh shell
   ```
   The prompt changes to `echou@<hostname>:/opt/echou-honors-dev$`. Until you build in step 7, a note says the image's copy of each workspace is in use. That's expected.
4. **Log in to GitHub with a fine-grained token.** Create the token in a browser on any machine:
   1. Go to GitHub → *Settings* → *Developer settings* → *Personal access tokens* → *Fine-grained tokens* → *Generate new token*.
   2. Set an expiration, for example 30 days or the end of the term. Don't choose "No expiration".
   3. Under *Repository access*, choose *Only select repositories* and pick this repository.
   4. Under *Permissions*, set *Contents* to *Read and write*. Set *Pull requests* to *Read and write* only if you'll use `gh pr`.
   5. Generate the token and copy it.

   Then, inside the container:
   ```bash
   gh auth login
   #   Where do you use GitHub?            GitHub.com
   #   Preferred protocol for Git?         HTTPS
   #   Authenticate Git with credentials?  Yes
   #   How would you like to authenticate? Paste an authentication token
   gh auth status            # should show "Logged in to github.com"
   ```
   `git push` and `git pull` now use this token, because the repo's `origin` is an HTTPS URL. The token is stored in `/home/echou/.config/gh/` in the volume.
5. **Tell git who you are.** This is stored in `/home/echou/.gitconfig`, separate from the host's git config:
   ```bash
   git config --global user.name  "Your Name"
   git config --global user.email "you@example.com"
   ```
6. **Log in to Claude Code:**
   ```bash
   claude --version          # the pinned version, e.g. 2.1.310 (Claude Code)
   claude
   ```
   Pick your account type when asked. The container can't open a browser, so Claude Code prints a sign-in URL. Open that URL in a browser on any machine, sign in, and copy the code the page shows. Paste the code back into the terminal at the prompt. The login is saved in `/home/echou/.claude` (set by `CLAUDE_CONFIG_DIR`). Type `/exit` to leave Claude Code.
7. **Build both workspaces into your checkout:**
   ```bash
   free -h                          # see §5: other people may be using memory too
   scripts/build.sh onboard
   scripts/build.sh offboard
   exit                             # then docker/dev.sh shell again, to pick up your build
   ```
   These are symlink installs, so Python, launch and YAML edits apply at the next launch without rebuilding. A new shell sources your `install/` folders instead of the image's copy.

### Daily loop

```bash
docker/dev.sh up          # on the host; does nothing if it's already running
docker/dev.sh shell
git pull
scripts/build.sh onboard  # only after C++ changes, new files, or new packages
scripts/teleop_slam.sh    # or any launch script
exit                      # the container keeps running; docker/dev.sh down stops it
```

You can open as many `docker/dev.sh shell` terminals as you like, for example one for the launch script and one for `ros2 topic hz`.

### VS Code

ROS 2 is sourced in every interactive shell in the container, including VS Code terminals, through `/etc/bash.bashrc`. Both flows below use the same `rover-dev` container as `docker/dev.sh`. If VS Code runs on a laptop, connect it to the Jetson with *Remote-SSH* first.

- **Reopen in Container (recommended).** Open the repo folder, then run *Dev Containers: Reopen in Container* from the Command Palette. VS Code reads [`.devcontainer/devcontainer.json`](../.devcontainer/devcontainer.json), starts the `dev` service if needed, opens `/opt/echou-honors-dev` as `echou`, and installs the Python, C/C++ and Claude Code extensions. The container keeps running when you close the window (`shutdownAction: none`).
- **Attach to the running container.** After `docker/dev.sh up`, run *Dev Containers: Attach to Running Container…*, pick `/rover-dev`, then *File → Open Folder → `/opt/echou-honors-dev`*. This flow ignores `devcontainer.json`, so install the extensions yourself the first time.

The VS Code server is installed under `/home/echou/.vscode-server`, so it's kept in the volume and later connections start faster.

### When to rebuild the image

**No image rebuild is needed** for:

- Edits under `ros2_ws/`, `ros2_ws_offboard/`, `scripts/` or anywhere else in the checkout. Run `scripts/build.sh` for C++ changes or new files, inside the container.
- git, `gh`, `claude` and anything else in your home directory.
- Changes to `docker/compose.yaml` or `docker/.env`. Run `docker/dev.sh up` again and Compose recreates the container with the new settings.

**Rebuild the image** for:

- Changes to `docker/Dockerfile`: new apt or pip packages, or a new pinned version such as `CLAUDE_CODE_VERSION`.
- Changes to `docker/entrypoint.sh`, which is copied into the image.
- A new system dependency in a `package.xml`. `sudo apt install` inside the container works as a test, but it's lost whenever the container is recreated, so add the package to the Dockerfile.
- Changes to `USER_UID` or `USER_GID`. Afterwards, run `docker/dev.sh reset-home` too, because the old home volume belongs to the old IDs.
- Running your changes with the `rover` service, which only has the code baked into the image.

To rebuild:

```bash
docker compose -f docker/compose.yaml build dev
docker/dev.sh up          # recreates rover-dev on the new image; /home/echou is kept
```

The `COPY` of the source invalidates the cache after any source change, so a rebuild recompiles both workspaces. Avoid rebuilding just to pick up code edits.

## 5. Shared Jetson

Other students run their own containers on this Jetson. These rules keep you from colliding with each other:

- **Names are unique.** The Compose project is `rover`, not the default `docker` (the folder name, which anyone else's `docker/compose.yaml` would also get). The container is `rover-dev`, the volume `rover-home`, the image `rover:jazzy`. Before your first `up`, check that nobody else uses these names: `docker ps -a`, `docker volume ls`, `docker images rover`.
- **Use your own ROS domain.** Host networking puts every container's DDS traffic on the same network, and anything left at `ROS_DOMAIN_ID=0` sees everyone else's topics. `rover-dev` uses `42` by default. To choose another, set `ROS_DOMAIN_ID` in `docker/.env`; that also changes the `rover` service's default. A laptop running RViz or the GPU machine must use the same number.
- **Only this container drives the rover hardware.** Every privileged container can see all of `/dev`, the VESC, the OAK-D and the Livox. Run launch scripts from `rover-dev` only. Don't run a `rover` service container at the same time, and check `docker ps` that no one else's container is using the hardware.
- **Protect your logins.** Anyone in the `docker` group can mount the `rover-home` volume and read the tokens in it. Use a fine-grained GitHub token limited to this repository, with an expiry date. When you're done with the Jetson for a while, log out with `gh auth logout` and `/logout` inside `claude`, or wipe everything with `docker/dev.sh reset-home`. Revoke the token on GitHub when the term ends.
- **Never `docker commit` the dev container.** The result is an image nobody can rebuild, and it would capture anything written outside the mounts. Change the Dockerfile and rebuild instead.
- **Check memory before heavy runs.** The Orin Nano's 8 GB is shared by the CPU, the GPU and everyone's containers. Before a workspace build or a SLAM run, check `free -h` and `docker stats --no-stream`. If memory is tight, build with fewer parallel jobs: `MAKEFLAGS=-j2 scripts/build.sh onboard`.

## 6. Troubleshooting

- **`/scan` missing, but the Livox pings from the host:** the container must use host networking (`docker inspect … | grep NetworkMode` → `host`).
- **OAK-D `X_LINK_DEVICE_NOT_FOUND`:** check that the container is privileged and has `/dev` mounted, and that the host udev rule is installed. Make sure no process on the host has the camera open.
- **Nothing visible from the GPU machine:** both sides need the same `ROS_DOMAIN_ID` and the same RMW (default Fast DDS), and both must be Jazzy.
- **`unknown or invalid runtime name: nvidia`:** the NVIDIA Container Toolkit isn't registered with Docker. Run `sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker` (§1), or set `DOCKER_RUNTIME=runc` to run without the GPU.
- **`error running createContainer hook #2` … `panic: runtime error: slice bounds out of range [:73] with capacity 71`:** a bug in NVIDIA Container Toolkit 1.19.1's `enable-cuda-compat` hook, which crashes reading the CUDA forward-compat libraries that `nvidia/cuda` images ship in `/usr/local/cuda/compat_orin`. This image deletes that folder, which a JetPack 7.2.1 host doesn't need, so it isn't affected. If you change the base image, keep the `rm -rf /usr/local/cuda/compat_orin` step. To fix it host-wide for every image, an admin can stop the hook from being generated:
  ```bash
  echo 'NVIDIA_CTK_CDI_GENERATE_DISABLED_HOOKS=enable-cuda-compat' | sudo tee -a /etc/nvidia-container-toolkit/nvidia-cdi-refresh.env
  sudo systemctl restart nvidia-cdi-refresh.service
  grep -c enable-cuda-compat /var/run/cdi/nvidia.yaml    # expect 0
  ```
- **`cuInit 801 | devices 0` in `rover-dev`, but `cuInit 0` as root:** the `echou` user isn't in the host's `render` group, which owns `/dev/dri/renderD*`. The NVIDIA runtime only grants that group to the container's first process, so `docker exec` shells and VS Code terminals miss it. Compose adds it with `group_add: ${RENDER_GID:-993}`. If your host's GID differs (`getent group render | cut -d: -f3`), set `RENDER_GID` in `docker/.env` and run `docker/dev.sh up`. `id -Gn` inside should list `render`.
- **`libcuda.so.1: cannot open shared object file` / `cuInit` returns non-zero:** the container was started without the NVIDIA runtime, or the host's driver is older than the image's CUDA (JetPack 6, or a JetPack 7.2 host with CUDA < 13.2.1). Upgrade the host or pick an older `nvidia/cuda:13.x` tag.
- **`Permission denied` in `/tmp/rover` or `maps/` from `rover-dev`:** Docker created the folder as root because it didn't exist yet, for example when a `rover` service run started first. On the host, run `sudo chown -R $USER /tmp/rover maps`. `docker/dev.sh up` and VS Code create both folders as you beforehand, which avoids this.
- **Files in the checkout owned by another user, or `/home/echou` not writable:** the image's `echou` UID/GID don't match yours. Set `USER_UID`/`USER_GID` in `docker/.env` (§4, first-time setup step 2), rebuild, and run `docker/dev.sh reset-home`.
- **pip `externally-managed-environment` error during the build:** the Dockerfile sets `PIP_BREAK_SYSTEM_PACKAGES=1` for Ubuntu 24.04's PEP 668 Python. Keep it if you change the base image.
- **Image build fails at `rosdep install`:** that step continues on missing keys (`-r`). Look further down for the real `colcon` error. A failing Livox build usually means the SDK step didn't install (check `ls /usr/local/lib | grep livox` in an intermediate image).
