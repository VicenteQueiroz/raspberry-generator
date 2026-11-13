#!/usr/bin/env bash
# Render real Gazebo camera images of a generated world without opening the GUI.
#
#   snapshot.sh <world_dir> [seconds]
#
# Adds an RGB camera and a semantic segmentation camera between the first two rows
# (robot's-eye view), runs `gz sim` headless (EGL) and writes the frames to
# <world_dir>/snapshots/{rgb,seg/colored_maps,seg/labels_maps}. labels_maps hold
# the raw gz Label ids (10 canes, 11 leaves, 12 calyx, 20 flowers, 21-25 fruit stages).
set -euo pipefail
WORLD_DIR="$(realpath "${1:?usage: snapshot.sh <world_dir> [seconds]}")"
WAIT="${2:-120}"
NAME="$(basename "$WORLD_DIR")"
OUT="$WORLD_DIR/snapshots"
rm -rf "$OUT"
mkdir -p "$OUT"

python3 - "$WORLD_DIR/$NAME.sdf" "$OUT/snapshot_world.sdf" "$OUT" <<'PY'
import sys, xml.etree.ElementTree as ET
src, dst, out = sys.argv[1:]
text = open(src).read()
world = ET.fromstring(text).find('world')
poses = [[float(v) for v in inc.find('pose').text.split()] for inc in world.findall('include')]
xs = [p[0] for p in poses]
rows = sorted({round(p[1], 1) for p in poses})
y = (rows[0] + rows[1]) / 2 if len(rows) > 1 else rows[0] - 1.25
cams = f'''
    <model name="snapshot_cameras">
      <static>true</static>
      <pose>{min(xs) - 1.2:.3f} {y:.3f} 1.2 0 0.18 0</pose>
      <link name="link">
        <sensor name="rgb" type="camera">
          <always_on>1</always_on><update_rate>2</update_rate>
          <camera><horizontal_fov>1.3</horizontal_fov>
            <image><width>1280</width><height>960</height></image>
            <clip><near>0.05</near><far>100</far></clip>
            <save enabled="true"><path>{out}/rgb</path></save></camera>
        </sensor>
        <sensor name="seg" type="segmentation">
          <always_on>1</always_on><update_rate>2</update_rate><topic>snapshot_seg</topic>
          <camera><segmentation_type>semantic</segmentation_type><horizontal_fov>1.3</horizontal_fov>
            <image><width>1280</width><height>960</height></image>
            <clip><near>0.05</near><far>100</far></clip>
            <save enabled="true"><path>{out}/seg</path></save></camera>
        </sensor>
      </link>
    </model>
  </world>'''
open(dst, 'w').write(text.replace('  </world>', cams))
PY

export GZ_SIM_RESOURCE_PATH="$WORLD_DIR${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -s -r --headless-rendering -v 2 "$OUT/snapshot_world.sdf" > "$OUT/gz.log" 2>&1 &
GZ_PID=$!
trap 'kill $GZ_PID $SUB1 $SUB2 2>/dev/null || true' EXIT
sleep 3
# gz only renders a camera while something subscribes to it (saving alone does not count)
gz topic -e -t "/world/$NAME/model/snapshot_cameras/link/link/sensor/rgb/image" > /dev/null &
SUB1=$!
gz topic -e -t /snapshot_seg/colored_map > /dev/null &
SUB2=$!

echo "waiting for Gazebo to load the meshes and render (max ${WAIT}s)..."
for ((i = 0; i < WAIT; i += 2)); do
    sleep 2
    n=$( (ls "$OUT/rgb" 2>/dev/null || true) | wc -l)
    [ "$n" -ge 4 ] && break
done
if [ "$( (ls "$OUT/rgb" 2>/dev/null || true) | wc -l)" -eq 0 ]; then
    echo "no frames rendered - see $OUT/gz.log (a GPU is needed for headless rendering)" >&2
    exit 1
fi
# keep the last (fully loaded) frame of each kind under a stable name
cp "$(ls -d "$OUT"/rgb/* | sort -V | tail -1)" "$OUT/rgb.png"
cp "$(ls -d "$OUT"/seg/colored_maps/* | sort -V | tail -1)" "$OUT/segmentation_colored.png"
cp "$(ls -d "$OUT"/seg/labels_maps/* | sort -V | tail -1)" "$OUT/segmentation_labels.png"
echo "snapshots written to $OUT (rgb.png, segmentation_colored.png, segmentation_labels.png)"
