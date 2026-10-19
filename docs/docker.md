# <div align="center">Running in Docker (Jetson Orin Nano)</div>

The whole rover stack can run as a container instead of a native install: ROS 2 Humble, Nav2, slam_toolbox, Livox-SDK2, DepthAI v3, and both workspaces built. The image is `linux/arm64` on the Orin Nano. The same Dockerfile also builds on an x86 GPU machine for the off-board half.

| File | Purpose |
|------|---------|
| [`docker/Dockerfile`](../docker/Dockerfile) | Image definition (base `ros:humble-ros-base`, Ubuntu 22.04) |
| [`docker/compose.yaml`](../docker/compose.yaml) | `rover` service (baked image) and `dev` service (live-mounted checkout) |
| [`docker/run.sh`](../docker/run.sh) | Same runtime options as Compose, with plain `docker run` |
| [`docker/entrypoint.sh`](../docker/entrypoint.sh) | Sources ROS 2 and both workspaces, then runs the command |
| [`.dockerignore`](../.dockerignore) | Keeps `.git`, build outputs, docs and maps out of the build context |

---

## 1. Host setup (once, on the Jetson)

The container shares the host's network, devices and udev, so a few things still live on the host:

1. **JetPack 6.x**: see [reproduction.md §2](reproduction.md#2-os-setup--jetpack-6-ubuntu-2204-on-jetson-orin-nano). ROS doesn't need to be installed on the host.
2. **Docker Engine + Compose plugin**:
   ```bash
   docker --version || curl -fsSL https://get.docker.com | sh
   docker compose version
   sudo usermod -aG docker $USER      # log out and back in
   ```
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
# or: docker build -f docker/Dockerfile -t rover:humble .
```

Building natively on the Orin Nano takes a while (Livox-SDK2 plus the C++ packages). The `livox_ros_driver2` submodule doesn't need to be initialised, because the Dockerfile fetches the pinned commit when the folder is empty.

Pinned versions (override with `--build-arg`):

| Build arg | Default |
|-----------|---------|
| `BASE_IMAGE` | `ros:humble-ros-base` |
| `LIVOX_SDK2_REF` | `v1.2.5` |
| `LIVOX_DRIVER_REF` | `4a1def9…` (the submodule commit) |
| `DEPTHAI_VERSION` | `3.11.0` |

**Building on a faster x86 machine instead:** `docker buildx build --platform linux/arm64 -f docker/Dockerfile -t rover:humble --load .` (needs QEMU binfmt: `docker run --privileged --rm tonistiigi/binfmt --install arm64`). Then move it with `docker save rover:humble | ssh rover@<jetson> docker load`.

**CUDA:** nothing in the stack uses the GPU today, so the slim ROS base is the default. For on-robot GPU work later, build with `--build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0` (match your L4T release from `cat /etc/nv_tegra_release`) and add `runtime: nvidia` to the Compose service. The Dockerfile installs ROS 2 from apt when the base doesn't have it.

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

Pass settings through the environment, e.g. `ROS_DOMAIN_ID=5 VESC_PORT=/dev/ttyACM0 docker compose -f docker/compose.yaml run --rm rover scripts/teleop.sh`.

To open a second shell in a running container, for `ros2 topic hz` or a manual `teleop` while `--headless`:

```bash
docker exec -it $(docker ps -q --filter ancestor=rover:humble | head -1) /entrypoint.sh bash
```

### What the runtime options do

| Option | Why |
|--------|-----|
| `network_mode: host` | DDS discovery with the GPU machine / laptop RViz, and Livox UDP on the host's `192.168.1.x` link |
| `ipc: host` | Fast DDS shared-memory transport with nodes on the host or in other containers |
| `privileged` + `/dev:/dev` | The OAK-D re-enumerates as a new USB device after its firmware boots, and the VESC can be re-plugged. Mounting all of `/dev` handles both, and keeps the host's `/dev/vesc` symlink visible |
| `init: true` | Ctrl+C reaches the scripts' cleanup trap, so every node stops |
| `/tmp/rover` volume | Component logs readable on the host |
| `maps/` volume | Maps saved to `ros2_ws/src/robot_slam/ros_data/maps/custom/` in the container land in `<repo>/maps/custom/` on the host |

### RViz

The image is `ros-base` (no GUI). Run RViz on a laptop with the same `ROS_DOMAIN_ID`, which also saves Jetson CPU. Host networking makes the container's topics visible on the LAN as if it were native.

## 4. Development workflow

The `rover` service runs code baked into the image, so source edits need `docker compose … build`. While iterating, use the `dev` service, which mounts the checkout over `/opt/rover`:

```bash
docker compose -f docker/compose.yaml run --rm dev
# inside:
scripts/build.sh onboard && scripts/build.sh offboard   # once; symlink install
source ros2_ws/install/setup.bash && source ros2_ws_offboard/install/local_setup.bash
scripts/teleop_slam.sh
```

After that first build, Python, launch and YAML edits on the host apply at the next launch. The `build/`, `install/` and `log/` folders in the checkout are created by root; if that gets in the way, run `sudo chown -R $USER ros2_ws ros2_ws_offboard`.

## 5. Troubleshooting

- **`/scan` missing, but the Livox pings from the host:** the container must use host networking (`docker inspect … | grep NetworkMode` → `host`).
- **OAK-D `X_LINK_DEVICE_NOT_FOUND`:** check that the container is privileged and has `/dev` mounted, and that the host udev rule is installed. Make sure no process on the host has the camera open.
- **Nothing visible from the GPU machine:** both sides need the same `ROS_DOMAIN_ID` and the same RMW (default Fast DDS), and both must be Humble.
- **Image build fails at `rosdep install`:** that step continues on missing keys (`-r`). Look further down for the real `colcon` error. A failing Livox build usually means the SDK step didn't install (check `ls /usr/local/lib | grep livox` in an intermediate image).
