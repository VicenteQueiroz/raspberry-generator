#!/usr/bin/env bash
# Example 3 - a full Gazebo world: 3 trellised hedgerows x 6 unique plants.
# Shows: field layout, trellis posts + wires, mulch strips, low-detail preset for large worlds.
set -euo pipefail
cd "$(dirname "$0")/.."
BLENDER=${BLENDER:-blender}

python3 scripts/generate_field.py --blender "$BLENDER" --out generated --world_name trellis_field \
    --rows 3 --plants_per_row 6 --seed 3 -- --detail low --ripeness 0.65

"$BLENDER" -b --factory-startup --python scripts/render_preview.py -- \
    --world_dir generated/trellis_field --out docs/images/field --views overview,row

# run it in Gazebo:
#   ros2 launch raspberry_field field.launch.py world_dir:=$PWD/generated/trellis_field world:=trellis_field.sdf
