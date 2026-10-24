# <div align="center">Running in Docker (Jetson Orin Nano)</div>

The whole rover stack can run as a container instead of a native install: ROS 2 Jazzy, Nav2, slam_toolbox, Livox-SDK2, DepthAI v3, and both workspaces built. The image is `linux/arm64` on the Orin Nano and targets a **JetPack 7.2** host, with CUDA available inside the container. The same Dockerfile also builds on an x86 GPU machine for the off-board half.

| File | Purpose |
|------|---------|
| [`docker/Dockerfile`](../docker/Dockerfile) | Image definition (base `nvidia/cuda:13.2.1-runtime-ubuntu24.04` + ROS 2 Jazzy from apt) |
| [`docker/compose.yaml`](../docker/compose.yaml) | `rover` service (baked image) and `dev` service (live-mounted checkout) |
| [`docker/run.sh`](../docker/run.sh) | Same runtime options as Compose, with plain `docker run` |
| [`docker/entrypoint.sh`](../docker/entrypoint.sh) | Sources ROS 2 and both workspaces, then runs the command |
| [`.dockerignore`](../.dockerignore) | Keeps `.git`, build outputs, docs and maps out of the build context |

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
| `LIVOX_SDK2_REF` | `v1.2.5` |
| `LIVOX_DRIVER_REF` | `4a1def9…` (the submodule commit) |
| `DEPTHAI_VERSION` | `3.11.0` |

**Building on a faster x86 machine instead:** `docker buildx build --platform linux/arm64 -f docker/Dockerfile -t rover:jazzy --load .` (needs QEMU binfmt: `docker run --privileged --rm tonistiigi/binfmt --install arm64`). Then move it with `docker save rover:jazzy | ssh rover@<jetson> docker load`.

**Why a plain CUDA image, not `l4t-jetpack`:** JetPack 7.2 moves the Orin family onto the arm64-SBSA CUDA stack, the same one Arm servers use. Standard multi-arch NGC images now run on the Orin Nano. NVIDIA's own Orin Nano Docker guide tests with `nvcr.io/nvidia/pytorch`. Keep the base's CUDA version at or below the host's: JetPack 7.2.1 ships CUDA 13.2.2, and `cat /usr/local/cuda/version.json` on the host shows it. The `-runtime` flavour has the CUDA libraries without the compiler. Switch to `-devel` if you build CUDA code in the image, for example for on-robot 3DGS.

**CPU-only image** (smaller; nothing in the current stack uses the GPU; also runs on a JetPack 6 host):

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
docker exec -it $(docker ps -q --filter ancestor=rover:jazzy | head -1) /entrypoint.sh bash
```

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
- **Nothing visible from the GPU machine:** both sides need the same `ROS_DOMAIN_ID` and the same RMW (default Fast DDS), and both must be Jazzy.
- **`unknown or invalid runtime name: nvidia`:** the NVIDIA Container Toolkit isn't registered with Docker. Run `sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker` (§1), or set `DOCKER_RUNTIME=runc` to run without the GPU.
- **`libcuda.so.1: cannot open shared object file` / `cuInit` returns non-zero:** the container was started without the NVIDIA runtime, or the host's driver is older than the image's CUDA (JetPack 6, or a JetPack 7.2 host with CUDA < 13.2.1). Upgrade the host or pick an older `nvidia/cuda:13.x` tag.
- **pip `externally-managed-environment` error during the build:** the Dockerfile sets `PIP_BREAK_SYSTEM_PACKAGES=1` for Ubuntu 24.04's PEP 668 Python. Keep it if you change the base image.
- **Image build fails at `rosdep install`:** that step continues on missing keys (`-r`). Look further down for the real `colcon` error. A failing Livox build usually means the SDK step didn't install (check `ls /usr/local/lib | grep livox` in an intermediate image).
