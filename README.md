# raspberry-generator
### procedural high-fidelity raspberry fields for Gazebo Harmonic (ROS 2 Jazzy)

A random raspberry field generator for agricultural robotics simulation. The
ROS 2 package is called `raspberry_field`. Each plant is generated from scratch
by a Blender script. There is no hand-made `.blend`
file: geometry, UVs, textures and materials are all code, and the same seed
always gives the same plant.

What a plant contains:
- **Crown of canes:** 4–7 brown, woody **floricanes** that carry fruit, plus
  2–5 green, leafy **primocanes**. Canes lean along the row like a hedgerow and
  carry prickles.
- **Fruiting laterals** on the upper ⅔ of the floricanes. Each lateral has
  trifoliate leaves and a fruit cyme at its tip, and the terminal fruit ripens
  first.
- **Compound leaves** with 3 or 5 leaflets. The leaflets have doubly serrate
  margins, a midrib fold and curl, a veined normal-mapped upper side and a pale
  underside.
- **Aggregate fruit.** Each fruit is made of 75–115 individual drupelets packed
  in a golden-angle spiral on a conical receptacle. It has a 5-sepal calyx that
  reflexes as the fruit ripens.
- **Ripeness stages:** flower / bud → green → white → pink → ripe → overripe.
  Colour comes from a fruit atlas, and the drupelets near the calyx lag behind
  the rest of the fruit.

## Repository layout

```
blender/raspberry_gen.py     Blender script: generates one plant model (meshes, textures, SDF, markers)
scripts/generate_field.py    builds a whole field world (runs Blender in parallel, writes world SDF + markers)
scripts/ground_truth.py      ROS 2 node: publishes markers.json as an RViz MarkerArray
launch/field.launch.py       starts Gazebo Harmonic with a generated world
```

## Outputs

```
<world_dir>/
  raspberry_field.sdf        world: sun, soil, mulch strips, trellis posts + wires, plants
  markers.json               world-frame ground truth (PLANT / FRUIT / FLOWER)
  raspberry_N/
    model.sdf, model.config
    meshes/<part>.glb        canes, leaves, calyx, flowers, fruit_<stage>
    materials/textures/*.png
    markers.json             plant-frame ground truth
```

Each part is its own `<visual>` with a `gz::sim::systems::Label` plugin, so a
Gazebo **segmentation camera** gives per-class masks straight away:

| label | part | label | part |
|---|---|---|---|
| 10 | canes, laterals, petioles, pedicels | 21 | fruit_green |
| 11 | leaves | 22 | fruit_white |
| 12 | calyx (sepals) | 23 | fruit_pink |
| 20 | flowers | 24 | fruit_ripe |
| | | 25 | fruit_overripe |

Each fruit marker records: `translation` (fruit centre), `attach_point`
(pedicel junction, useful for picking), `axis`, `length`, `diameter`, `stage`,
`ripeness`, `drupelets` and `plant_id`.

## Requirements
- Blender ≥ 3.6 (tested with 4.2 LTS). It's only needed to *generate* worlds.
- ROS 2 Jazzy + Gazebo Harmonic (`ros-jazzy-ros-gz`) to run them.

## Build
```bash
cd ~/ros2_ws/src && git clone https://github.com/VicenteQueiroz/raspberry-generator.git
cd ~/ros2_ws && colcon build --packages-select raspberry_field && source install/setup.bash
```

## Generate a field
```bash
ros2 run raspberry_field generate_field.py --out ~/fields --rows 3 --plants_per_row 6 --seed 1 \
    -- --detail medium --ripeness 0.7
# or without ROS: python3 scripts/generate_field.py ... (stdlib only)
```
Field options: `--rows`, `--plants_per_row`, `--row_spacing` (2.5 m),
`--plant_spacing` (0.6 m), `--variants N` (generate N models and reuse them;
this saves GPU memory), `--jobs`, `--blender PATH` (or `$BLENDER`), and
`--no_trellis`.

Plant options go after `--`:

| option | default | meaning |
|---|---|---|
| `--detail low/medium/high` | medium | drupelet / leaf / tube resolution and texture size |
| `--ripeness` | 0.7 | season progress: <0 flowers, 0.3 green, 0.7 mixed, 1.0 overripe |
| `--ripeness_spread` | 0.3 | per-fruit variation |
| `--height` | 1.7 | floricane height [m] |
| `--floricanes`, `--primocanes` | random | cane counts |
| `--fruit_density`, `--leaf_density` | 1.0 | multipliers |
| `--prickles` | 1.0 | 0 = thornless cultivar |
| `--fruit_collisions` | off | add a static sphere collision per fruit |

Use a single plant directly:
`blender -b --factory-startup --python blender/raspberry_gen.py -- --model_dir out/raspberry_0 --seed 42`

**Budget.** A plant has roughly 200–400 fruits. Its triangle count is about
0.3 M at `low`, 0.5–0.9 M at `medium` and about 2.4 M at `high`. For large
fields, use `--detail low` and/or `--variants`.

## Run
```bash
ros2 launch raspberry_field field.launch.py world_dir:=$HOME/fields/raspberry_field
```
This starts `gz sim` with the world dir on `GZ_SIM_RESOURCE_PATH`, a `/clock`
bridge, and `ground_truth.py`. The ground-truth node publishes a latched
`visualization_msgs/MarkerArray` on `/raspberry_field/ground_truth` in frame
`world`, with fruit spheres coloured by stage. Spawn your robot into the world
as usual.

## Notes
- The meshes are written to glTF with **Z-up data** (`export_yup=False`),
  which is what Gazebo's Assimp loader expects. If plants show up lying on
  their side in your Gazebo version, regenerate with `-- --gltf_yup`.
- Leaves are real two-sided geometry (0.3 mm thick) rather than alpha cards,
  because Gazebo ignores glTF `doubleSided`.

## Credits

This project is based on
[**fields-ignition**](https://github.com/azazdeaz/fields-ignition) by András
Polgár ([@azazdeaz](https://github.com/azazdeaz)), a random tomato field
generator for ROS 1 Noetic and Ignition Gazebo. The overall idea and pipeline
come from that project: generate plants with the Blender Python API, assemble
them into a world, and export per-plant and world-level `markers.json` ground
truth. This repository replaces the tomato models with a fully procedural
raspberry generator and moves the stack to ROS 2 Jazzy and Gazebo Harmonic.

## License

MIT. See [LICENSE](LICENSE), which keeps the original fields-ignition copyright notice.
