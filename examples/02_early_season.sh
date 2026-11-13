#!/usr/bin/env bash
# Example 2 - the same kind of plant early in the season.
# Shows: the ripeness/season control - flowers, buds and green/white fruit instead of ripe berries.
set -euo pipefail
cd "$(dirname "$0")/.."
BLENDER=${BLENDER:-blender}

"$BLENDER" -b --factory-startup --python blender/raspberry_gen.py -- \
    --model_dir generated/early_season --seed 21 --detail high --ripeness 0.0 --ripeness_spread 0.2

"$BLENDER" -b --factory-startup --python scripts/render_preview.py -- \
    --model_dir generated/early_season --out docs/images/early \
    --views plant,cluster,labels --focus flower
