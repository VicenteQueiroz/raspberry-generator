#!/usr/bin/env bash
# Run the raspberry-generator container with the host display, GPU and ./generated mounted.
#
#   ./docker/run.sh demo
#   ./docker/run.sh generate --rows 3 --plants_per_row 6 -- --detail low
#   ./docker/run.sh sim raspberry_field
#
# Environment:
#   IMAGE      image name             (default: raspberry-generator)
#   DATA_DIR   host output directory  (default: ./generated next to this repo)
#   GPU        auto | nvidia | dri | none   (default: auto)
#   GZ_PARTITION    Gazebo transport partition (default: raspberry_field), keeps this
#                   sim separate from other Gazebo instances on the host network
#   ROS_DOMAIN_ID   passed through if set
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${IMAGE:-raspberry-generator}"
DATA_DIR="${DATA_DIR:-$REPO/generated}"
GPU="${GPU:-auto}"
mkdir -p "$DATA_DIR"

if [ "$GPU" = auto ]; then
    if command -v nvidia-smi >/dev/null 2>&1 && docker info 2>/dev/null | grep -qi 'runtimes:.*nvidia'; then
        GPU=nvidia
    elif [ -d /dev/dri ]; then
        GPU=dri
    else
        GPU=none
    fi
fi

gpu_args=()
case "$GPU" in
    nvidia) gpu_args=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all
                      -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia) ;;
    dri)    gpu_args=(--device /dev/dri) ;;
    none)   gpu_args=(-e LIBGL_ALWAYS_SOFTWARE=1) ;;
esac

display_args=()
if [ -n "${DISPLAY:-}" ]; then
    # let local containers talk to the X server (undo with: xhost -local:root)
    command -v xhost >/dev/null 2>&1 && xhost +local:root >/dev/null 2>&1 || true
    display_args=(-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

tty_args=()
[ -t 0 ] && tty_args=(-it)

echo "[run.sh] image=$IMAGE gpu=$GPU data=$DATA_DIR" >&2
exec docker run --rm "${tty_args[@]}" --net=host --ipc=host \
    "${gpu_args[@]}" "${display_args[@]}" \
    -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
    -e GZ_PARTITION="${GZ_PARTITION:-raspberry_field}" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -v "$DATA_DIR:/data" \
    "$IMAGE" "$@"
