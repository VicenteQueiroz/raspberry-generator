"""
Render preview images of a generated plant model or a whole field world (Blender Cycles).

It loads the exported .glb files, so the images show exactly what Gazebo gets.

    # a single plant model
    blender -b --factory-startup --python scripts/render_preview.py -- \
        --model_dir generated/plant --out docs/images/plant --views plant,cluster,fruit,labels

    # a field world written by generate_field.py
    blender -b --factory-startup --python scripts/render_preview.py -- \
        --world_dir generated/raspberry_field --out docs/images/field --views overview,row

Views:
    plant     the whole plant                         (model)
    cluster   close-up of a visible fruit cluster     (model, uses --focus)
    fruit     macro shot of one fruit                 (model, uses --focus)
    labels    the cluster view coloured by the gz segmentation label of each part
    overview  elevated view of the whole field        (world)
    row       robot's-eye view down the first inter-row (world)
Images are written as <out>_<view>.jpg
"""
import sys
import json
import math
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import bpy
from mathutils import Matrix, Vector, Euler

p = argparse.ArgumentParser()
p.add_argument('--model_dir')
p.add_argument('--world_dir')
p.add_argument('--out', required=True, help='output path prefix')
p.add_argument('--views', default='plant,cluster,fruit,labels')
p.add_argument('--focus', default='ripe', help='marker stage the close-ups look at (ripe, pink, flower, ...)')
p.add_argument('--samples', type=int, default=64)
p.add_argument('--res', default='1280x960')
args = p.parse_args(sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else [])

# visualisation colours for the gz Label ids written by raspberry_gen.py
LABEL_COLORS = {
    'canes': (0.30, 0.16, 0.06), 'leaves': (0.05, 0.45, 0.08), 'calyx': (0.55, 0.75, 0.05),
    'flowers': (0.95, 0.95, 0.95), 'fruit_green': (0.0, 0.9, 0.9), 'fruit_white': (0.95, 0.85, 0.2),
    'fruit_pink': (1.0, 0.3, 0.75), 'fruit_ripe': (0.9, 0.0, 0.0), 'fruit_overripe': (0.35, 0.0, 0.55),
}

sc = bpy.context.scene
for ob in list(bpy.data.objects):
    bpy.data.objects.remove(ob, do_unlink=True)


# --------------------------------------------------------------------------- loading

def import_model(model_dir):
    """Import every part .glb of a model under an empty; returns the empty."""
    root = bpy.data.objects.new(model_dir.name + '_root', None)
    sc.collection.objects.link(root)
    fix = Matrix.Rotation(math.radians(-90), 4, 'X')   # importer assumes Y-up; data is Z-up
    for glb in sorted((model_dir / 'meshes').glob('*.glb')):
        before = set(bpy.data.objects)
        bpy.ops.import_scene.gltf(filepath=str(glb))
        for o in set(bpy.data.objects) - before:
            o['part'] = glb.stem
            if o.parent is None:
                o.matrix_basis = fix @ o.matrix_basis
                o.parent = root
    return root


def instance(root, name):
    new = bpy.data.objects.new(name, None)
    sc.collection.objects.link(new)
    for c in root.children:
        cc = c.copy()            # linked duplicate: shares the mesh data
        sc.collection.objects.link(cc)
        cc.parent = new
    return new


def parse_pose(el):
    v = [float(x) for x in (el.text.split() if el is not None and el.text else [0] * 6)]
    return Vector(v[:3]), Euler(v[3:6], 'XYZ')


def flat_material(name, rgba):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes['Principled BSDF']
    b.inputs['Base Color'].default_value = rgba
    b.inputs['Roughness'].default_value = 0.85
    return m


def soil_material(rgba):
    """Flat SDF colour plus a little procedural noise, for nicer renders."""
    m = flat_material('soil', rgba)
    nt = m.node_tree
    noise = nt.nodes.new('ShaderNodeTexNoise')
    noise.inputs['Scale'].default_value = 8.0
    nt.links.new(nt.nodes.new('ShaderNodeTexCoord').outputs['Object'], noise.inputs['Vector'])
    noise.inputs['Detail'].default_value = 8.0
    ramp = nt.nodes.new('ShaderNodeValToRGB')
    ramp.color_ramp.elements[0].color = [c * 0.6 for c in rgba[:3]] + [1]
    ramp.color_ramp.elements[1].color = [min(c * 1.4, 1) for c in rgba[:3]] + [1]
    nt.links.new(noise.outputs['Fac'], ramp.inputs['Fac'])
    nt.links.new(ramp.outputs['Color'], nt.nodes['Principled BSDF'].inputs['Base Color'])
    return m


def add_primitive(vis, model_pose):
    pos, rot = parse_pose(vis.find('pose'))
    g = vis.find('geometry')
    mat_el = vis.find('material/diffuse')
    rgba = tuple(float(x) for x in mat_el.text.split()) if mat_el is not None else (0.5, 0.5, 0.5, 1)
    rgba = tuple(c ** 2.2 for c in rgba[:3]) + (1.0,)    # treat SDF colours as sRGB for a natural look
    loc = model_pose[0] + pos
    if g.find('plane') is not None:
        sx, sy = (float(x) for x in g.find('plane/size').text.split())
        bpy.ops.mesh.primitive_plane_add(size=1, location=loc)
        ob = bpy.context.active_object
        ob.scale = (sx, sy, 1)
        ob.data.materials.append(soil_material(rgba))
        return
    if g.find('box') is not None:
        sx, sy, sz = (float(x) for x in g.find('box/size').text.split())
        bpy.ops.mesh.primitive_cube_add(size=1, location=loc, rotation=rot)
        bpy.context.active_object.scale = (sx, sy, sz)
    elif g.find('cylinder') is not None:
        bpy.ops.mesh.primitive_cylinder_add(radius=float(g.find('cylinder/radius').text),
                                            depth=float(g.find('cylinder/length').text),
                                            vertices=16, location=loc, rotation=rot)
    else:
        return
    bpy.context.active_object.data.materials.append(flat_material('prim', rgba))


def load_world(world_dir):
    sdf = next(world_dir.glob('*.sdf'))
    world = ET.parse(sdf).getroot().find('world')
    roots, plants = {}, []
    for inc in world.findall('include'):
        model = inc.find('uri').text.strip().replace('model://', '')
        pos, rot = parse_pose(inc.find('pose'))
        if model not in roots:
            roots[model] = import_model(world_dir / model)
            ob = roots[model]
        else:
            ob = instance(roots[model], inc.find('name').text)
        ob.location, ob.rotation_euler = pos, rot
        plants.append(pos)
    for model in world.findall('model'):
        mpose = parse_pose(model.find('pose'))
        for vis in model.iter('visual'):
            add_primitive(vis, mpose)
    return plants


# --------------------------------------------------------------------------- scene

def setup_scene():
    w, h = (int(x) for x in args.res.split('x'))
    sc.render.resolution_x, sc.render.resolution_y = w, h
    sc.render.engine = 'CYCLES'
    sc.cycles.device = 'CPU'
    sc.cycles.samples = args.samples
    sc.cycles.use_denoising = True
    sc.render.image_settings.file_format = 'JPEG'
    sc.render.image_settings.quality = 90
    world = bpy.data.worlds.new('sky')
    sc.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes['Background']
    bg.inputs['Color'].default_value = (0.55, 0.68, 0.86, 1)
    bg.inputs['Strength'].default_value = 0.9
    sun = bpy.data.objects.new('sun', bpy.data.lights.new('sun', 'SUN'))
    sun.data.energy = 4.0
    sun.data.angle = math.radians(2)
    sun.rotation_euler = (math.radians(40), math.radians(15), math.radians(30))
    sc.collection.objects.link(sun)
    cam = bpy.data.objects.new('cam', bpy.data.cameras.new('cam'))
    cam.data.clip_start = 0.002
    sc.collection.objects.link(cam)
    sc.camera = cam
    return cam


def look(cam, pos, target, lens):
    cam.location = Vector(pos)
    cam.data.lens = lens
    cam.rotation_euler = (Vector(target) - Vector(pos)).to_track_quat('-Z', 'Y').to_euler()


def render(name):
    sc.render.filepath = f'{args.out}_{name}.jpg'
    bpy.ops.render.render(write_still=True)
    print('[render_preview] wrote', sc.render.filepath)


def visible(origin, target, radius):
    dg = bpy.context.evaluated_depsgraph_get()
    d = target - origin
    hit, loc, *_ = sc.ray_cast(dg, origin, d.normalized(), distance=d.length + radius)
    return hit and (loc - target).length < radius * 1.2


def pick_focus(markers, dist):
    """An outer, well-populated, unoccluded fruit of the requested stage."""
    pts = [Vector(m['translation']) for m in markers]
    cand = [m for m in markers if m.get('stage') == args.focus] or \
        [m for m in markers if m['marker_type'] == 'FRUIT']
    scored = []
    for m in cand:
        t = Vector(m['translation'])
        crowd = sum(1 for q in pts if (q - t).length < 0.1)
        scored.append((crowd + 12 * t.xy.length, m))
    scored.sort(key=lambda s: -s[0])
    for _, m in scored[:40]:
        t = Vector(m['translation'])
        d = Vector((t.x, t.y, 0.3 * t.xy.length)).normalized()
        if visible(t + d * dist, t, m.get('diameter', 0.01) / 2):
            return t, d
    t = Vector(scored[0][1]['translation'])
    return t, Vector((t.x, t.y, 0.3 * t.xy.length)).normalized()


def label_materials():
    """Replace every material with an emission colour per gz label (segmentation look)."""
    for o in sc.objects:
        if o.type != 'MESH':
            continue
        part = o.get('part')
        o.data.materials.clear()
        m = bpy.data.materials.new('label_' + str(part))
        m.use_nodes = True
        nt = m.node_tree
        nt.nodes.clear()
        em = nt.nodes.new('ShaderNodeEmission')
        em.inputs['Color'].default_value = (*LABEL_COLORS.get(part, (0, 0, 0)), 1)
        out = nt.nodes.new('ShaderNodeOutputMaterial')
        nt.links.new(em.outputs[0], out.inputs[0])
        o.data.materials.append(m)
    sc.world.node_tree.nodes['Background'].inputs['Strength'].default_value = 0.0
    sc.view_settings.view_transform = 'Standard'
    sc.cycles.samples = 8
    sc.cycles.use_denoising = False


def main():
    cam = setup_scene()
    views = args.views.split(',')
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    if args.model_dir:
        model_dir = Path(args.model_dir)
        import_model(model_dir)
        bpy.ops.mesh.primitive_plane_add(size=8)
        bpy.context.active_object.data.materials.append(soil_material((0.07, 0.04, 0.025, 1)))
        markers = json.loads((model_dir / 'markers.json').read_text())
        meshes = [o for o in sc.objects if o.type == 'MESH' and o.get('part')]
        bpy.context.view_layer.update()
        corners = [o.matrix_world @ Vector(c) for o in meshes for c in o.bound_box]
        lo = Vector([min(c[i] for c in corners) for i in range(3)])
        hi = Vector([max(c[i] for c in corners) for i in range(3)])
        center, size = (lo + hi) / 2, hi - lo
        if 'plant' in views:
            dist = max(size.z, size.x, size.y) * 0.95 / (2 * math.tan(math.radians(39.6) / 2))
            d = Vector((0.75, -0.62, 0.22)).normalized()
            look(cam, center + d * dist, center, 35)
            render('plant')
        for view, dist, lens in (('cluster', 0.38, 50), ('fruit', 0.10, 60)):
            if view in views:
                t, d = pick_focus(markers, dist)
                look(cam, t + d * dist, t, lens)
                render(view)
        if 'labels' in views:
            t, d = pick_focus(markers, 0.38)
            look(cam, t + d * 0.38, t, 50)
            label_materials()
            render('labels')
    else:
        plants = load_world(Path(args.world_dir))
        xs, ys = [p.x for p in plants], [p.y for p in plants]
        cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        span = max(max(xs) - min(xs), max(ys) - min(ys)) + 3
        if 'overview' in views:
            look(cam, (cx - 0.55 * span, cy - 0.85 * span, 0.42 * span), (cx, cy, 0.5), 30)
            render('overview')
        if 'row' in views:
            rows = sorted(set(round(y, 1) for y in ys))
            y = (rows[0] + rows[1]) / 2 if len(rows) > 1 else rows[0] - 1.0
            look(cam, (min(xs) - 1.0, y, 1.1), (max(xs) + 1.0, y + 0.1, 0.8), 26)
            render('row')


main()
