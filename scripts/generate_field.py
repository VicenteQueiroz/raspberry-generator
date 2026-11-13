#!/usr/bin/env python3
"""
Generate a raspberry hedgerow world for Gazebo Harmonic.

Calls Blender (headless) once per plant model with blender/raspberry_gen.py, then
writes the world SDF and a world-frame markers.json (ground truth for every plant,
fruit and flower).

Example:
    ros2 run raspberry_field generate_field.py --out ~/fields --rows 3 --plants_per_row 8 \
        --seed 1 -- --detail medium --ripeness 0.7

Everything after `--` is forwarded to raspberry_gen.py (see `blender -b --python
raspberry_gen.py -- --help` for the plant options).

Only the Python standard library is needed (plus Blender >= 3.6 on PATH or --blender).
"""
import argparse
import json
import math
import os
import random
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


def find_gen_script():
    here = Path(__file__).resolve().parent
    candidates = [here.parent / 'blender' / 'raspberry_gen.py']
    try:  # installed with colcon: share/raspberry_field/blender
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory('raspberry_field')) / 'blender' / 'raspberry_gen.py')
    except Exception:
        pass
    for c in candidates:
        if c.exists():
            return c
    sys.exit('could not find blender/raspberry_gen.py')


def parse_args():
    argv = sys.argv[1:]
    plant_args = []
    if '--' in argv:
        i = argv.index('--')
        argv, plant_args = argv[:i], argv[i + 1:]
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out', default='generated', help='output directory (world goes to OUT/WORLD_NAME)')
    p.add_argument('--world_name', default='raspberry_field')
    p.add_argument('--seed', type=int, default=178)
    p.add_argument('--rows', type=int, default=3)
    p.add_argument('--plants_per_row', type=int, default=6)
    p.add_argument('--row_spacing', type=float, default=2.5, help='distance between rows [m]')
    p.add_argument('--plant_spacing', type=float, default=0.6, help='distance between plants in a row [m]')
    p.add_argument('--variants', type=int, default=0,
                   help='generate only N unique plant models and reuse them with random yaw '
                        '(0 = every plant unique). Saves a lot of GPU memory on big fields.')
    p.add_argument('--jobs', type=int, default=max(1, (os.cpu_count() or 2) // 2),
                   help='parallel Blender processes')
    p.add_argument('--blender', default=os.environ.get('BLENDER', 'blender'), help='Blender executable')
    p.add_argument('--no_trellis', action='store_true', help='do not add posts and wires')
    p.add_argument('--keep', action='store_true', help='do not delete an existing world directory')
    a = p.parse_args(argv)
    a.plant_args = plant_args
    return a


def rotate_z(v, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]]


def gen_model(blender, script, model_dir, seed, plant_args):
    cmd = [blender, '--background', '--factory-startup', '--python', str(script), '--',
           '--model_dir', str(model_dir), '--seed', str(seed)] + plant_args
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    summary = [l for l in res.stdout.splitlines() if l.startswith('[raspberry_gen]')]
    if res.returncode != 0 or not (model_dir / 'model.sdf').exists():
        sys.stderr.write(res.stdout[-4000:])
        raise RuntimeError(f'Blender failed for {model_dir.name}')
    print(summary[0] if summary else f'generated {model_dir.name}', flush=True)


def trellis_sdf(rows, row_len, x0, row_spacing):
    """Wooden posts every <=8 m and two pairs of wires per row (static)."""
    parts = []
    k = 0
    for r in range(rows):
        y = r * row_spacing
        n_posts = max(2, int(math.ceil(row_len / 8.0)) + 1)
        for i in range(n_posts):
            x = x0 + row_len * i / (n_posts - 1)
            parts.append(f"""      <collision name="post_{k}_c"><pose>{x:.3f} {y:.3f} 0.9 0 0 0</pose>
        <geometry><cylinder><radius>0.04</radius><length>1.8</length></cylinder></geometry></collision>
      <visual name="post_{k}"><pose>{x:.3f} {y:.3f} 0.9 0 0 0</pose>
        <geometry><cylinder><radius>0.04</radius><length>1.8</length></cylinder></geometry>
        <material><diffuse>0.42 0.32 0.22 1</diffuse><ambient>0.3 0.22 0.15 1</ambient></material></visual>""")
            k += 1
        for z in (0.9, 1.5):
            for dy in (-0.15, 0.15):
                parts.append(f"""      <visual name="wire_{k}"><pose>{x0 + row_len / 2:.3f} {y + dy:.3f} {z} 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.0015</radius><length>{row_len:.3f}</length></cylinder></geometry>
        <material><diffuse>0.6 0.6 0.62 1</diffuse><specular>0.8 0.8 0.8 1</specular></material></visual>""")
                k += 1
    return """    <model name="trellis">
      <static>true</static>
      <link name="link">
{}
      </link>
    </model>""".format('\n'.join(parts))


def world_sdf(name, includes, size, center, rows, row_len, x0, row_spacing, trellis):
    mulch = '\n'.join(f"""      <visual name="mulch_{r}"><pose>{x0 + row_len / 2:.3f} {r * row_spacing:.3f} 0.002 0 0 0</pose>
        <geometry><box><size>{row_len + 0.6:.3f} 0.7 0.004</size></box></geometry>
        <material><diffuse>0.18 0.12 0.08 1</diffuse><ambient>0.12 0.08 0.05 1</ambient></material></visual>"""
                      for r in range(rows))
    inc = '\n'.join(f"""    <include>
      <uri>model://{m}</uri>
      <name>{n}</name>
      <pose>{pose}</pose>
    </include>""" for m, n, pose in includes)
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.62 0.74 0.88 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 0.97 0.9 1</diffuse>
      <specular>0.4 0.4 0.4 1</specular>
      <direction>-0.4 0.25 -0.88</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{size[0]:.1f} {size[1]:.1f}</size></plane></geometry>
        </collision>
        <visual name="soil">
          <pose>{center[0]:.3f} {center[1]:.3f} 0 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>{size[0]:.1f} {size[1]:.1f}</size></plane></geometry>
          <material><diffuse>0.36 0.27 0.19 1</diffuse><ambient>0.25 0.19 0.13 1</ambient></material>
        </visual>
{mulch}
      </link>
    </model>

{trellis}

{inc}
  </world>
</sdf>
"""


def main():
    a = parse_args()
    script = find_gen_script()
    if shutil.which(a.blender) is None and not Path(a.blender).exists():
        sys.exit(f'Blender not found: {a.blender} (use --blender or the BLENDER env var)')

    rnd = random.Random(a.seed)
    world_dir = Path(a.out).resolve() / a.world_name
    if world_dir.exists() and not a.keep:
        shutil.rmtree(world_dir)
    world_dir.mkdir(parents=True, exist_ok=True)

    n_plants = a.rows * a.plants_per_row
    n_models = a.variants if a.variants > 0 else n_plants
    models = [f'raspberry_{i}' for i in range(n_models)]
    seeds = [rnd.randint(0, 10 ** 6) for _ in models]

    print(f'generating {n_models} plant model(s) with {a.jobs} job(s) -> {world_dir}', flush=True)
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        futures = [ex.submit(gen_model, a.blender, script, world_dir / m, s, a.plant_args)
                   for m, s in zip(models, seeds)]
        for f in futures:
            f.result()

    plant_markers = {m: json.loads((world_dir / m / 'markers.json').read_text()) for m in models}

    markers, includes = [], []
    next_id = 1
    row_len = (a.plants_per_row - 1) * a.plant_spacing
    for r in range(a.rows):
        for i in range(a.plants_per_row):
            k = r * a.plants_per_row + i
            model = models[k] if a.variants == 0 else rnd.choice(models)
            x = i * a.plant_spacing + rnd.uniform(-0.08, 0.08)
            y = r * a.row_spacing + rnd.uniform(-0.04, 0.04)
            yaw = rnd.choice([0.0, math.pi]) + rnd.gauss(0, 0.15)
            name = f'plant_{k}'
            includes.append((model, name, f'{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}'))
            plant_id = next_id
            next_id += 1
            markers.append({'marker_type': 'PLANT', 'id': plant_id, 'name': name, 'model': model,
                            'translation': [x, y, 0.0], 'yaw': yaw})
            for m in plant_markers[model]:
                w = dict(m)
                w['id'] = next_id
                w['plant_id'] = plant_id
                next_id += 1
                for key in ('translation', 'attach_point'):
                    if key in m:
                        v = rotate_z(m[key], yaw)
                        w[key] = [v[0] + x, v[1] + y, v[2]]
                if 'axis' in m:
                    w['axis'] = rotate_z(m['axis'], yaw)
                markers.append(w)

    margin = 4.0
    size = (row_len + 2 * margin, (a.rows - 1) * a.row_spacing + 2 * margin)
    center = (row_len / 2, (a.rows - 1) * a.row_spacing / 2)
    trellis = '' if a.no_trellis else trellis_sdf(a.rows, row_len + 0.6, -0.3, a.row_spacing)
    (world_dir / f'{a.world_name}.sdf').write_text(
        world_sdf(a.world_name, includes, size, center, a.rows, row_len, 0.0, a.row_spacing, trellis))
    with open(world_dir / 'markers.json', 'w') as f:
        json.dump(markers, f, indent=2)

    counts = {}
    for m in markers:
        key = m.get('stage', m['marker_type'].lower())
        counts[key] = counts.get(key, 0) + 1
    print(f'world written: {world_dir / (a.world_name + ".sdf")}')
    print(f'markers: {counts}')


if __name__ == '__main__':
    main()
