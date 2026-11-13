#!/usr/bin/env bash
# Example 1 - a single plant at harvest time, maximum detail.
# Shows: drupelet-level fruit geometry, every ripeness stage on one plant, segmentation labels.
set -euo pipefail
cd "$(dirname "$0")/.."
BLENDER=${BLENDER:-blender}

"$BLENDER" -b --factory-startup --python blender/raspberry_gen.py -- \
    --model_dir generated/harvest_plant --seed 7 --detail high --ripeness 0.7

"$BLENDER" -b --factory-startup --python scripts/render_preview.py -- \
    --model_dir generated/harvest_plant --out docs/images/harvest \
    --views plant,cluster,fruit,labels --focus ripe
