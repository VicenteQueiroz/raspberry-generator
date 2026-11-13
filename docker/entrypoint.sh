#!/usr/bin/env bash
# Container entrypoint: small command set on top of the raspberry_field package.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

DATA=/data
EXAMPLES=/ws/src/raspberry_field/examples

# give generated files back to the host user (the container runs as root)
fix_owner() {
    if [ -n "${HOST_UID:-}" ]; then
        chown -R "${HOST_UID}:${HOST_GID:-$HOST_UID}" "$DATA" 2>/dev/null || true
    fi
}

world_name_from_args() {
    # value of --world_name in "$@", default raspberry_field
    local name=raspberry_field prev=
    for a in "$@"; do
        [ "$a" = "--" ] && break
        [ "$prev" = "--world_name" ] && name="$a"
        prev="$a"
    done
    echo "$name"
}

sim() {
    local world="${1:-raspberry_field}"
    shift || true
    if [ ! -f "$DATA/$world/$world.sdf" ]; then
        echo "no world at $DATA/$world/$world.sdf - generate it first (see: help)" >&2
        exit 1
    fi
    exec ros2 launch raspberry_field field.launch.py \
        world_dir:="$DATA/$world" world:="$world.sdf" "$@"
}

cmd="${1:-help}"
shift || true
case "$cmd" in
    generate)
        # generate [field options] [-- plant options]   (output always goes to /data)
        ros2 run raspberry_field generate_field.py --out "$DATA" "$@"
        fix_owner
        ;;
    sim)
        # sim [world_name] [launch args...]
        sim "$@"
        ;;
    demo)
        # small field (2 rows x 4 plants, low detail), generated once, then opened in Gazebo
        if [ ! -f "$DATA/demo_field/demo_field.sdf" ]; then
            ros2 run raspberry_field generate_field.py --out "$DATA" --world_name demo_field \
                --rows 2 --plants_per_row 4 --seed 3 -- --detail low --ripeness 0.65
            fix_owner
        fi
        sim demo_field "$@"
        ;;
    snapshot)
        # snapshot <world_name> [max_seconds]  -> real Gazebo RGB + segmentation frames, no GUI
        world="${1:?usage: snapshot <world_name>}"
        /ws/src/raspberry_field/docker/snapshot.sh "$DATA/$world" "${2:-120}"
        fix_owner
        ;;
    render)
        # render <world_name> [render_preview.py options]  -> /data/<world_name>/preview_*.jpg
        world="${1:?usage: render <world_name>}"
        shift
        blender -b --factory-startup --python /ws/src/raspberry_field/scripts/render_preview.py -- \
            --world_dir "$DATA/$world" --out "$DATA/$world/preview" --views overview,row "$@"
        fix_owner
        ;;
    help|-h|--help)
        cat <<EOF
raspberry-generator container

  generate [options] [-- plant options]  generate a field world into /data/<world_name>
                                         (see: generate --help, and the README for plant options)
  sim [world_name] [launch args]         open /data/<world_name> in Gazebo Harmonic (default raspberry_field)
  demo                                   generate a small demo field (once) and open it in Gazebo
  snapshot <world_name>                  headless Gazebo camera + segmentation images (GPU, no display)
  render <world_name>                    Blender preview renders (no GPU / display needed)
  bash | <any command>                   run a shell or any command with ROS 2 sourced

Example scripts from the README are in $EXAMPLES (BLENDER=blender is already set up).
EOF
        ;;
    *)
        exec "$cmd" "$@"
        ;;
esac
