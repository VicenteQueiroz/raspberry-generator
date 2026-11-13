"""
Procedural, high-fidelity raspberry plant generator for Gazebo Harmonic.

Everything (geometry, UVs, textures, materials) is generated from code, so no
.blend file is needed. Run it with Blender in background mode:

    blender --background --factory-startup --python raspberry_gen.py -- \
        --model_dir out/raspberry_0 --seed 42

Output (inside --model_dir):
    model.sdf, model.config          Gazebo model (meshes referenced as model://<name>/...)
    meshes/<part>.glb                one glTF binary per semantic part
    materials/textures/*.png         generated textures (also embedded in the .glb)
    markers.json                     ground truth: fruits / flowers relative to the plant origin

Semantic parts (each one is a separate <visual> with a gz Label plugin, so the
segmentation camera can tell them apart):
    canes (10), leaves (11), calyx (12), flowers (20),
    fruit_green (21), fruit_white (22), fruit_pink (23), fruit_ripe (24), fruit_overripe (25)

Botany it is modelled on (Rubus idaeus, summer bearing, hedgerow):
    * a crown with 2nd-year floricanes (brown, woody) that carry fruiting laterals
      on their upper 2/3, and 1st-year primocanes (green, leafy, no fruit)
    * pinnately compound leaves: 3 leaflets on laterals, 5 on vigorous primocanes,
      doubly serrate margins, dark wrinkled upper side and whitish underside
    * fruits in small cymes at the lateral tips, hanging on pedicels
    * each fruit is an aggregate of ~75-115 drupelets packed in a phyllotactic
      spiral on a conical receptacle, with a 5-sepal calyx
"""
import sys
import json
import math
import argparse
from pathlib import Path

import numpy as np
import bpy

# --------------------------------------------------------------------------- #
# Arguments
# --------------------------------------------------------------------------- #

parser = argparse.ArgumentParser(description='Raspberry plant generator')
parser.add_argument('-o', '--model_dir', default='generated/raspberry_0')
parser.add_argument('-s', '--seed', type=int, default=None)
parser.add_argument('--detail', choices=['low', 'medium', 'high'], default='medium',
                    help='mesh / texture resolution preset')
parser.add_argument('--height', type=float, default=1.7,
                    help='approximate floricane height [m]')
parser.add_argument('--floricanes', type=int, default=None,
                    help='number of fruiting canes (default: random 4-7)')
parser.add_argument('--primocanes', type=int, default=None,
                    help='number of new green canes (default: random 2-5)')
parser.add_argument('--ripeness', type=float, default=0.7,
                    help='season progress: <0 mostly flowers, 0.3 green, 0.7 mixed/ripe, 1 overripe')
parser.add_argument('--ripeness_spread', type=float, default=0.3,
                    help='std-dev of per-fruit ripeness around --ripeness')
parser.add_argument('--fruit_density', type=float, default=1.0,
                    help='multiplier for fruits per lateral')
parser.add_argument('--leaf_density', type=float, default=1.0,
                    help='multiplier for leaves per shoot')
parser.add_argument('--prickles', type=float, default=1.0,
                    help='multiplier for prickle density (0 = thornless cultivar)')
parser.add_argument('--fruit_collisions', action='store_true',
                    help='add a sphere collision per fruit (static) to the SDF')
parser.add_argument('--gltf_yup', action='store_true',
                    help='export glTF with +Y up (Gazebo expects Z-up data, so leave off)')

argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
args = parser.parse_known_args(argv)[0]
if args.seed is None:
    args.seed = int(np.random.randint(100000))

rng = np.random.RandomState(args.seed)

DETAIL = {
    'low':    dict(drup_seg=4, drup_rings=3, core_seg=8, leaf_nu=12, leaf_nv=3, tube=6, tex=256),
    'medium': dict(drup_seg=6, drup_rings=3, core_seg=12, leaf_nu=18, leaf_nv=5, tube=8, tex=512),
    'high':   dict(drup_seg=10, drup_rings=6, core_seg=16, leaf_nu=30, leaf_nv=7, tube=12, tex=1024),
}[args.detail]

# ripeness -> stage. (name, exclusive upper bound, gz label)
STAGES = [
    ('flower', 0.0, 20),
    ('green', 0.30, 21),
    ('white', 0.48, 22),
    ('pink', 0.66, 23),
    ('ripe', 0.92, 24),
    ('overripe', 99.0, 25),
]
LABELS = {'canes': 10, 'leaves': 11, 'calyx': 12, 'flowers': 20}
for _name, _, _label in STAGES[1:]:
    LABELS['fruit_' + _name] = _label

R_MAX = 1.05            # highest ripeness value (maps to the right edge of the fruit atlas)
GOLDEN_ANGLE = math.pi * (3.0 - math.sqrt(5.0))
UP = np.array([0.0, 0.0, 1.0])


def stage_of(r):
    for name, upper, _ in STAGES:
        if r < upper:
            return name
    return STAGES[-1][0]


# --------------------------------------------------------------------------- #
# Small math helpers
# --------------------------------------------------------------------------- #

def normalize(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, 1e-12)


def perp(d):
    """Two unit vectors orthogonal to d (and to each other)."""
    a = np.array([1.0, 0, 0]) if abs(d[0]) < 0.9 else np.array([0, 1.0, 0])
    x = normalize(np.cross(d, a))
    return x, np.cross(d, x)


def rot(axis, ang):
    x, y, z = normalize(axis)
    c, s = math.cos(ang), math.sin(ang)
    C = 1 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0, 1)
    return t * t * (3 - 2 * t)


def frames(pts):
    """Parallel-transport frames (T, N, B) along a polyline."""
    T = normalize(np.gradient(pts, axis=0))
    N = np.zeros_like(pts)
    N[0] = perp(T[0])[0]
    for i in range(1, len(pts)):
        v = N[i - 1] - np.dot(N[i - 1], T[i]) * T[i]
        nv = np.linalg.norm(v)
        N[i] = v / nv if nv > 1e-9 else perp(T[i])[0]
    return T, N, np.cross(T, N)


def integrate(start, dirs, ds):
    """Polyline starting at `start` following unit directions `dirs` with step ds."""
    steps = np.cumsum(dirs[:-1] * ds, axis=0)
    return np.vstack([start, start + steps])


def orient_outward(verts, faces, centers):
    """Flip faces whose normal points towards `centers` (array per face or a point)."""
    v0, v1, v2 = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    n = np.cross(v1 - v0, v2 - v0)
    centroid = verts[faces].mean(axis=1)
    flip = np.einsum('ij,ij->i', n, centroid - centers) < 0
    faces = faces.copy()
    faces[flip] = faces[flip, ::-1]
    return faces


# --------------------------------------------------------------------------- #
# Mesh accumulation (numpy -> one Blender object per semantic part)
# --------------------------------------------------------------------------- #

class MeshBuilder:
    def __init__(self, name):
        self.name = name
        self.verts, self.uvs, self.faces, self.mats = [], [], [], []
        self.nv = 0

    def add(self, verts, uvs, faces, mat=0):
        """faces: (F, k) index array, or a list of such arrays sharing `verts`."""
        verts = np.asarray(verts, dtype=float).reshape(-1, 3)
        uvs = np.asarray(uvs, dtype=float).reshape(-1, 2)
        assert len(verts) == len(uvs)
        groups = faces if isinstance(faces, (list, tuple)) else [faces]
        groups = [np.asarray(f, dtype=np.int64) for f in groups if len(f)]
        if not groups:
            return
        self.verts.append(verts)
        self.uvs.append(uvs)
        for f in groups:
            self.faces.append(f + self.nv)
            self.mats.append(np.full(len(f), mat, dtype=np.int32))
        self.nv += len(verts)

    @property
    def empty(self):
        return self.nv == 0

    def face_count(self):
        return sum(len(f) for f in self.faces)

    def to_object(self, materials):
        V = np.concatenate(self.verts)
        UV = np.concatenate(self.uvs)
        faces = []
        for f in self.faces:
            faces.extend(map(tuple, f.tolist()))
        mesh = bpy.data.meshes.new(self.name)
        mesh.from_pydata(V.tolist(), [], faces)
        loop_vi = np.empty(len(mesh.loops), dtype=np.int64)
        mesh.loops.foreach_get('vertex_index', loop_vi)
        uv_layer = mesh.uv_layers.new(name='UVMap')
        uv_layer.data.foreach_set('uv', UV[loop_vi].astype(np.float32).ravel())
        mesh.polygons.foreach_set('material_index', np.concatenate(self.mats))
        mesh.polygons.foreach_set('use_smooth', np.ones(len(mesh.polygons), dtype=bool))
        for m in materials:
            mesh.materials.append(m)
        mesh.update()
        ob = bpy.data.objects.new(self.name, mesh)
        bpy.context.scene.collection.objects.link(ob)
        return ob


# --------------------------------------------------------------------------- #
# Primitive builders
# --------------------------------------------------------------------------- #

def add_tube(mb, pts, radii, sides, mat=0, v_scale=4.0, cap=True):
    """Generalised cylinder along a polyline (parallel transport frames)."""
    pts = np.asarray(pts, dtype=float)
    radii = np.broadcast_to(np.asarray(radii, dtype=float), (len(pts),))
    T, N, B = frames(pts)
    th = np.linspace(0, 2 * np.pi, sides + 1)
    ring = (pts[:, None, :] + radii[:, None, None] *
            (np.cos(th)[None, :, None] * N[:, None, :] + np.sin(th)[None, :, None] * B[:, None, :]))
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    U, Vv = np.meshgrid(th / (2 * np.pi), s * v_scale)
    S = sides + 1
    i = np.arange(len(pts) - 1)[:, None]
    j = np.arange(sides)[None, :]
    a = (i * S + j).ravel()
    mb.add(ring.reshape(-1, 3), np.stack([U, Vv], -1).reshape(-1, 2),
           np.stack([a, a + 1, a + S + 1, a + S], 1), mat)
    if cap:
        last = ring[-1, :-1]
        tip = pts[-1] + T[-1] * radii[-1] * 0.6
        verts = np.vstack([last, tip])
        jj = np.arange(sides)
        f = np.stack([jj, (jj + 1) % sides, np.full(sides, sides)], 1)
        f = orient_outward(verts, f, pts[-1] - T[-1] * radii[-1])
        mb.add(verts, np.tile([[0.5, s[-1] * v_scale]], (sides + 1, 1)), f, mat)
    return T, N, B


def add_blade(mb, origin, X, Z, length, half_w, nv, zfun, mats, thickness=0.0003):
    """Two-sided leaf-like blade. X: along the midrib, Z: blade normal.
    half_w: half width samples along X. zfun(U, V, xl, yl) -> offset along Z."""
    Y = normalize(np.cross(Z, X))
    Z = np.cross(X, Y)
    nu = len(half_w)
    U, Vg = np.meshgrid(np.linspace(0, 1, nu), np.linspace(-1, 1, nv), indexing='ij')
    xl = U * length
    yl = Vg * half_w[:, None]
    zl = zfun(U, Vg, xl, yl)
    P = origin + xl[..., None] * X + yl[..., None] * Y + zl[..., None] * Z
    Nrm = normalize(np.cross(np.gradient(P, axis=0), np.gradient(P, axis=1)))
    hw = max(half_w.max(), 1e-9)
    uv = np.stack([0.5 + 0.5 * Vg * half_w[:, None] / hw, U], -1).reshape(-1, 2)
    i = np.arange(nu - 1)[:, None]
    j = np.arange(nv - 1)[None, :]
    a = (i * nv + j).ravel()
    q = np.stack([a, a + nv, a + nv + 1, a + 1], 1)
    mb.add((P + Nrm * thickness * 0.5).reshape(-1, 3), uv, q, mats[0])
    mb.add((P - Nrm * thickness * 0.5).reshape(-1, 3), uv, q[:, ::-1], mats[1])


def add_spikes(mb, base, d, ref, length, radius, mat=0):
    """Many small 4-sided cones (prickles, stamens). base/d/ref: (n, 3); length/radius: (n,)."""
    n = len(base)
    L = np.asarray(length, float)[:, None]
    rb = np.asarray(radius, float)[:, None]
    e1 = normalize(np.cross(d, ref))
    e2 = np.cross(d, e1)
    k = np.arange(4) * np.pi / 2
    ring = base[:, None, :] + rb[:, None] * (np.cos(k)[None, :, None] * e1[:, None] +
                                              np.sin(k)[None, :, None] * e2[:, None])
    tip = base + d * L
    verts = np.concatenate([ring, tip[:, None]], 1).reshape(-1, 3)
    f = np.array([[0, 1, 4], [1, 2, 4], [2, 3, 4], [3, 0, 4]])
    faces = (f[None] + 5 * np.arange(n)[:, None, None]).reshape(-1, 3)
    faces = orient_outward(verts, faces, np.repeat(base - d * rb, 4, axis=0))
    mb.add(verts, np.full((len(verts), 2), 0.5), faces, mat)


def sphere_template(seg, rings):
    phi = np.linspace(0, np.pi, rings + 1)[1:-1]
    th = np.linspace(0, 2 * np.pi, seg, endpoint=False)
    rv = np.stack([np.outer(np.sin(phi), np.cos(th)),
                   np.outer(np.sin(phi), np.sin(th)),
                   np.outer(np.cos(phi), np.ones(seg))], -1).reshape(-1, 3)
    verts = np.vstack([[0, 0, 1], rv, [0, 0, -1]])
    nr = rings - 1
    bottom = 1 + nr * seg
    j = np.arange(seg)
    jn = (j + 1) % seg
    tris = [np.stack([np.zeros(seg, int), 1 + j, 1 + jn], 1),
            np.stack([np.full(seg, bottom), 1 + (nr - 1) * seg + jn, 1 + (nr - 1) * seg + j], 1)]
    quads = []
    for k in range(nr - 1):
        quads.append(np.stack([1 + k * seg + j, 1 + (k + 1) * seg + j,
                               1 + (k + 1) * seg + jn, 1 + k * seg + jn], 1))
    tris = orient_outward(verts, np.vstack(tris), np.zeros(3))
    quads = orient_outward(verts, np.vstack(quads), np.zeros(3)) if quads else np.zeros((0, 4), int)
    return verts, tris, quads


DRUPELET = sphere_template(DETAIL['drup_seg'], DETAIL['drup_rings'])


# --------------------------------------------------------------------------- #
# Procedural textures
# --------------------------------------------------------------------------- #

def value_noise(H, W, cy, cx):
    """Tileable bilinear value noise in [0, 1]."""
    g = rng.rand(cy, cx)
    y = np.arange(H) * cy / H
    x = np.arange(W) * cx / W
    y0 = np.floor(y).astype(int)
    x0 = np.floor(x).astype(int)
    fy = smoothstep(0, 1, y - y0)[:, None]
    fx = smoothstep(0, 1, x - x0)[None, :]
    y1, x1 = (y0 + 1) % cy, (x0 + 1) % cx
    a, b = g[np.ix_(y0, x0)], g[np.ix_(y0, x1)]
    c, d = g[np.ix_(y1, x0)], g[np.ix_(y1, x1)]
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy


def fbm(H, W, cy, cx, octaves=4):
    out = np.zeros((H, W))
    amp, tot = 1.0, 0.0
    for o in range(octaves):
        out += amp * value_noise(H, W, cy * 2 ** o, cx * 2 ** o)
        tot += amp
        amp *= 0.5
    return out / tot


def lerp_col(c0, c1, t):
    c0 = np.asarray(c0, float) / 255.0
    c1 = np.asarray(c1, float) / 255.0
    t = np.asarray(t)[..., None]
    return c0 * (1 - t) + c1 * t


def height_to_normal(h, strength):
    gy, gx = np.gradient(h)
    n = normalize(np.stack([-gx * strength, -gy * strength, np.ones_like(h)], -1))
    return n * 0.5 + 0.5


TEX_DIR = Path(args.model_dir) / 'materials' / 'textures'


def make_image(name, rgb, non_color=False):
    H, W = rgb.shape[:2]
    rgba = np.concatenate([np.clip(rgb, 0, 1), np.ones((H, W, 1))], -1)
    img = bpy.data.images.new(name, W, H, alpha=False)
    img.pixels.foreach_set(rgba.astype(np.float32).ravel())
    path = TEX_DIR / (name + '.png')
    img.filepath_raw = str(path.resolve())
    img.file_format = 'PNG'
    img.save()
    if non_color:
        img.colorspace_settings.name = 'Non-Color'
    return img


def tex_leaf(T):
    ty = ((np.arange(T) + 0.5) / T)[:, None] * np.ones((1, T))
    tx = ((np.arange(T) + 0.5) / T)[None, :] * np.ones((T, 1))
    adx = np.abs(tx - 0.5)
    mid = np.exp(-(adx / 0.012) ** 2)
    n_veins = 11
    ph = (ty - adx * 1.15) * n_veins
    d = np.minimum(ph - np.floor(ph), 1 - (ph - np.floor(ph))) / n_veins
    lat = np.exp(-(d / 0.0045) ** 2) * np.clip(1 - adx * 1.7, 0, 1) * (ph > 0.3)
    net = fbm(T, T, 18, 18, 3)
    tert = np.exp(-((net - 0.5) / 0.02) ** 2) * 0.15
    veins = np.clip(mid + 0.85 * lat + tert, 0, 1)
    pucker = fbm(T, T, 26, 26, 3)
    height = -veins + 0.35 * pucker
    mottle = fbm(T, T, 6, 6, 4)
    top = lerp_col((40, 78, 28), (66, 108, 40), mottle)
    top = top * (1 - veins[..., None] * 0.75) + lerp_col((105, 140, 70), (105, 140, 70), 0) * veins[..., None] * 0.75
    top *= (0.9 + 0.2 * pucker)[..., None]
    bottom = lerp_col((178, 190, 160), (196, 204, 182), mottle)
    bottom = bottom * (1 - veins[..., None] * 0.6) + np.array([140, 165, 115]) / 255 * veins[..., None] * 0.6
    normal = height_to_normal(height, strength=T / 90.0)
    return top, bottom, normal


def tex_bark(T, base, alt, streak_col):
    H, W = T * 2, T // 2
    streaks = value_noise(H, W, 6, 40)
    patches = fbm(H, W, 4, 6, 4)
    fine = fbm(H, W, 64, 64, 2)
    col = lerp_col(base, alt, smoothstep(0.45, 0.7, patches))
    col = col * (1 - 0.35 * smoothstep(0.6, 0.85, streaks))[..., None] + \
        np.asarray(streak_col) / 255 * 0.35 * smoothstep(0.6, 0.85, streaks)[..., None]
    col *= (0.88 + 0.24 * fine)[..., None]
    normal = height_to_normal(streaks * 0.6 + fine * 0.4, strength=T / 60.0)
    return col, normal


# fruit colour stops: ripeness value -> sRGB
FRUIT_STOPS = [
    (0.00, (112, 150, 62)),    # green
    (0.30, (196, 204, 134)),   # whitish green
    (0.45, (232, 206, 172)),   # cream
    (0.58, (228, 128, 122)),   # pink / turning
    (0.72, (196, 32, 52)),     # light red
    (0.88, (150, 14, 38)),     # ripe deep red
    (R_MAX, (78, 6, 28)),      # overripe
]


def tex_fruit_atlas(W=256, H=64):
    r = (np.arange(W) + 0.5) / W * R_MAX
    xs = [s[0] for s in FRUIT_STOPS]
    col = np.stack([np.interp(r, xs, [s[1][c] for s in FRUIT_STOPS]) for c in range(3)], -1) / 255.0
    v = (np.arange(H) + 0.5) / H
    shade = 0.55 + 0.45 * smoothstep(0.0, 0.7, v)
    hi = 0.10 * smoothstep(0.85, 1.0, v)
    return np.clip(col[None, :, :] * shade[:, None, None] + hi[:, None, None], 0, 1)


def srgb_to_lin(c):
    c = np.asarray(c, float) / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def make_material(name, color=None, image=None, roughness=0.5, normal_img=None,
                  normal_strength=1.0, specular=0.5):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    bsdf = next(n for n in nt.nodes if n.type == 'BSDF_PRINCIPLED')
    bsdf.inputs['Roughness'].default_value = roughness
    for key in ('Specular IOR Level', 'Specular'):
        if key in bsdf.inputs:
            bsdf.inputs[key].default_value = specular
            break
    if color is not None:
        bsdf.inputs['Base Color'].default_value = (*srgb_to_lin(color), 1.0)
    if image is not None:
        t = nt.nodes.new('ShaderNodeTexImage')
        t.image = image
        nt.links.new(t.outputs['Color'], bsdf.inputs['Base Color'])
    if normal_img is not None:
        t = nt.nodes.new('ShaderNodeTexImage')
        t.image = normal_img
        nm = nt.nodes.new('ShaderNodeNormalMap')
        nm.inputs['Strength'].default_value = normal_strength
        nt.links.new(t.outputs['Color'], nm.inputs['Color'])
        nt.links.new(nm.outputs['Normal'], bsdf.inputs['Normal'])
    m.use_backface_culling = False
    return m


def build_materials():
    T = DETAIL['tex']
    TEX_DIR.mkdir(parents=True, exist_ok=True)
    leaf_top, leaf_bot, leaf_n = tex_leaf(T)
    bark, bark_n = tex_bark(T // 2, (98, 58, 46), (140, 128, 118), (60, 34, 28))
    green, green_n = tex_bark(T // 2, (86, 124, 50), (128, 76, 58), (110, 150, 70))
    img_leaf_n = make_image('leaf_normal', leaf_n, non_color=True)
    return {
        'canes': [
            make_material('floricane_bark', image=make_image('floricane_bark', bark), roughness=0.8,
                          normal_img=make_image('floricane_normal', bark_n, True)),
            make_material('stem_green', image=make_image('stem_green', green), roughness=0.6,
                          normal_img=make_image('stem_green_normal', green_n, True), normal_strength=0.5),
            make_material('prickle', color=(92, 40, 34), roughness=0.5),
        ],
        'leaves': [
            make_material('leaf_upper', image=make_image('leaf_upper', leaf_top), roughness=0.55,
                          normal_img=img_leaf_n, normal_strength=1.0),
            make_material('leaf_lower', image=make_image('leaf_lower', leaf_bot), roughness=0.85,
                          normal_img=img_leaf_n, normal_strength=0.6, specular=0.2),
        ],
        'fruit': [make_material('drupelets', image=make_image('fruit_atlas', tex_fruit_atlas()),
                                roughness=0.38, specular=0.6)],
        'calyx': [make_material('sepal', color=(92, 118, 52), roughness=0.85),
                  make_material('sepal_dry', color=(108, 96, 52), roughness=0.9)],
        'flowers': [make_material('petal', color=(242, 240, 228), roughness=0.6),
                    make_material('flower_center', color=(150, 170, 80), roughness=0.7),
                    make_material('stamen', color=(232, 214, 150), roughness=0.7)],
    }


# --------------------------------------------------------------------------- #
# Plant organs
# --------------------------------------------------------------------------- #

class Plant:
    def __init__(self):
        self.parts = {k: MeshBuilder(k) for k in LABELS}
        self.markers = []
        self.collisions = []   # ('cylinder', p0, p1, radius) | ('sphere', center, radius)

    # ---------------- canes / shoots ----------------

    def add_prickles(self, pts, radii, T, N, B, density, mat=2):
        length = np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))
        n = rng.poisson(max(0.0, length * density * args.prickles))
        if n == 0:
            return
        # skip the very base and the soft tip
        idx = rng.randint(int(len(pts) * 0.03), max(int(len(pts) * 0.9), 2), n)
        psi = rng.uniform(0, 2 * np.pi, n)
        radial = np.cos(psi)[:, None] * N[idx] + np.sin(psi)[:, None] * B[idx]
        base = pts[idx] + radial * radii[idx, None] * 0.9
        d = normalize(radial - 0.45 * T[idx])
        add_spikes(self.parts['canes'], base, d, T[idx], rng.uniform(0.0018, 0.0035, n),
                   rng.uniform(0.0004, 0.0007, n), mat)

    def cane(self, base, length, azim, lean, arch, r0, r1, floricane):
        ds = 0.02
        n = max(int(length / ds), 4)
        s = np.linspace(0, 1, n)
        phi = lean + arch * s ** 2.2 + 0.04 * np.sin(s * rng.uniform(8, 14) + rng.uniform(0, 6))
        az = azim + 0.25 * np.sin(s * rng.uniform(2, 5) + rng.uniform(0, 6)) * s
        dirs = np.stack([np.sin(phi) * np.cos(az), np.sin(phi) * np.sin(az), np.cos(phi)], -1)
        pts = integrate(base, dirs, length / (n - 1))
        radii = r0 + (r1 - r0) * s ** 0.8
        T, N, B = add_tube(self.parts['canes'], pts, radii, DETAIL['tube'],
                           mat=0 if floricane else 1, v_scale=4.0)
        self.add_prickles(pts, radii, T, N, B, 25.0 if floricane else 45.0)
        step = max(n // 5, 1)
        for i in range(0, n - 1, step):
            j = min(i + step, n - 1)
            self.collisions.append(('cylinder', pts[i], pts[j], float(radii[i])))
        return pts, radii, T, N, B

    def floricane(self, base, azim):
        length = args.height * rng.uniform(0.9, 1.15)
        pts, radii, T, N, B = self.cane(base, length, azim, rng.uniform(0.04, 0.22),
                                        rng.uniform(0.2, 1.1), rng.uniform(0.0048, 0.0062),
                                        0.0022, floricane=True)
        n = len(pts)
        s = 0.0
        psi = rng.uniform(0, 2 * np.pi)
        while True:
            s += rng.uniform(0.045, 0.08)
            rel = s / length
            if rel > 0.97:
                break
            i = int(rel * (n - 1))
            psi += np.deg2rad(144) + rng.normal(0, 0.2)
            out = np.cos(psi) * N[i] + np.sin(psi) * B[i]
            node = pts[i] + out * radii[i] * 0.8
            if rel > 0.33 and rng.rand() < 0.65:
                self.lateral(node, T[i], out, rel)
            elif rel > 0.15 and rng.rand() < 0.12 * args.leaf_density:
                self.compound_leaf(node, normalize(out + 0.6 * UP), rng.uniform(0.05, 0.07), 3,
                                   old=True)

    def primocane(self, base, azim):
        length = args.height * rng.uniform(0.45, 0.95)
        pts, radii, T, N, B = self.cane(base, length, azim, rng.uniform(0.02, 0.15),
                                        rng.uniform(0.05, 0.5), rng.uniform(0.0045, 0.006),
                                        0.0018, floricane=False)
        n = len(pts)
        s = rng.uniform(0.04, 0.08)
        psi = rng.uniform(0, 2 * np.pi)
        while s < length:
            rel = s / length
            i = min(int(rel * (n - 1)), n - 1)
            psi += np.deg2rad(144) + rng.normal(0, 0.15)
            out = np.cos(psi) * N[i] + np.sin(psi) * B[i]
            if rng.rand() < (0.95 if rel > 0.25 else 0.55) * args.leaf_density:
                size = rng.uniform(0.075, 0.11) * (1 - 0.65 * rel ** 4)
                leaflets = 5 if (rel < 0.75 and rng.rand() < 0.75) else 3
                pet = normalize(out * rng.uniform(0.8, 1.2) + UP * rng.uniform(0.4, 0.9))
                self.compound_leaf(pts[i] + out * radii[i], pet, size, leaflets)
            s += rng.uniform(0.045, 0.07) * (1 - 0.45 * rel)

    def lateral(self, node, T, out, rel):
        length = rng.uniform(0.12, 0.36) * (0.55 + 0.75 * math.sin(math.pi * min(rel + 0.1, 1.0)))
        beta = rng.uniform(0.55, 1.05)
        d0 = normalize(T * math.cos(beta) + out * math.sin(beta))
        e0 = math.asin(np.clip(d0[2], -1, 1))
        a0 = math.atan2(d0[1], d0[0])
        droop = rng.uniform(0.6, 1.5)
        n = max(int(length / 0.012), 5)
        s = np.linspace(0, 1, n)
        e = e0 - droop * s ** 1.3
        a = a0 + rng.normal(0, 0.15) * s
        dirs = np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], -1)
        pts = integrate(node, dirs, length / (n - 1))
        radii = 0.0024 + (0.0012 - 0.0024) * s
        Tl, Nl, Bl = add_tube(self.parts['canes'], pts, radii, max(DETAIL['tube'] - 2, 5), mat=1)
        self.add_prickles(pts, radii, Tl, Nl, Bl, 12.0)

        # leaves along the lateral
        n_leaves = max(1, int(round(rng.randint(3, 6) * args.leaf_density)))
        psi = rng.uniform(0, 2 * np.pi)
        for k in range(n_leaves):
            rl = (k + 0.6) / (n_leaves + 0.4) * 0.9
            i = int(rl * (n - 1))
            psi += np.deg2rad(144) + rng.normal(0, 0.2)
            lo = np.cos(psi) * Nl[i] + np.sin(psi) * Bl[i]
            pet = normalize(lo + Tl[i] * 0.5 + UP * rng.uniform(0.3, 0.8))
            leaflets = 5 if (k == 0 and rng.rand() < 0.2) else 3
            self.compound_leaf(pts[i] + lo * radii[i], pet, rng.uniform(0.05, 0.085), leaflets)

        # fruit cyme at the tip: terminal fruit ripens first
        n_fruit = 1 + rng.poisson(2.5 * args.fruit_density)
        cluster_r = args.ripeness + rng.normal(0, args.ripeness_spread * 0.6)
        for k in range(n_fruit):
            if k == 0:
                i, r = n - 1, cluster_r + 0.15
                d = Tl[-1]
            else:
                i = int(rng.uniform(0.72, 0.98) * (n - 1))
                r = cluster_r + rng.normal(-0.05, args.ripeness_spread * 0.5)
                ang = rng.uniform(0, 2 * np.pi)
                d = normalize(np.cos(ang) * Nl[i] + np.sin(ang) * Bl[i] + Tl[i] * 0.6)
            self.fruit_on_pedicel(pts[i], d, float(np.clip(r, -0.3, R_MAX)))

    # ---------------- leaves ----------------

    def leaflet(self, origin, X, Z, length, rel_width):
        nu, nv = DETAIL['leaf_nu'], DETAIL['leaf_nv']
        u = np.linspace(0, 1, nu)
        w = np.sin(np.pi * u ** 0.72) ** 0.85 * (1 - 0.25 * u ** 2)
        teeth = np.ones(nu)
        teeth[1:-1:2] = 0.92            # notches
        teeth[2:-1:4] *= 1.03           # doubly serrate
        w = w * teeth * length * rel_width * 0.5
        w[0] = length * 0.012
        w[-1] = 0.0
        fold = rng.uniform(0.08, 0.35)
        curl = rng.uniform(0.6, 2.4)
        wave_f, wave_p = rng.uniform(80, 160), rng.uniform(0, 6)
        twist = rng.normal(0, 0.15)

        def zf(U, V, xl, yl):
            return (fold * np.abs(yl) - curl * xl ** 2 + twist * xl * yl * 10 +
                    0.0009 * np.sin(xl * wave_f + wave_p) * np.abs(V) ** 2)

        add_blade(self.parts['leaves'], origin, X, Z, length, w, nv, zf, (0, 1))

    def compound_leaf(self, base, petiole_dir, size, n_leaflets, old=False):
        """size: length of the terminal leaflet [m]."""
        pet_len = size * rng.uniform(0.45, 0.8)
        n = 8
        t = np.linspace(0, 1, n)[:, None]
        dirs = normalize(petiole_dir[None] * (1 - 0.5 * t) + np.array([0, 0, -1.0])[None] * 0.5 * t ** 1.5)
        pts = integrate(base, dirs, pet_len / (n - 1))
        add_tube(self.parts['canes'], pts, np.linspace(0.0012, 0.0008, n), 5, mat=1, cap=False)

        end, tang = pts[-1], dirs[-1]
        F = normalize(np.array([tang[0], tang[1], 0.0]))
        if np.linalg.norm(tang[:2]) < 1e-3:
            F = perp(UP)[0]
        Lft = normalize(np.cross(UP, F))
        pitch = rng.uniform(-0.15, 0.45) + (0.5 if old else 0.0)
        M = rot(F, rng.normal(0, 0.25)) @ rot(Lft, pitch)
        Fp, Lp, Up = M @ F, M @ Lft, M @ UP

        def place(origin, angle, scale, extra_droop=0.0):
            X = rot(Up, angle) @ Fp
            X = rot(np.cross(Up, X), extra_droop) @ X
            Z = normalize(Up - np.dot(Up, X) * X)
            self.leaflet(origin, X, Z, size * scale, rng.uniform(0.58, 0.72))

        cursor = end
        if n_leaflets == 5:
            for sgn in (-1, 1):
                place(cursor, sgn * rng.uniform(1.2, 1.6), rng.uniform(0.6, 0.75), 0.2)
            rach = size * rng.uniform(0.2, 0.3)
            rpts = np.stack([cursor, cursor + Fp * rach * 0.5, cursor + Fp * rach])
            add_tube(self.parts['canes'], rpts, 0.0007, 4, mat=1, cap=False)
            cursor = rpts[-1]
        for sgn in (-1, 1):
            place(cursor, sgn * rng.uniform(0.85, 1.25), rng.uniform(0.75, 0.88), 0.12)
        stalk = size * rng.uniform(0.12, 0.22)
        spts = np.stack([cursor, cursor + Fp * stalk * 0.5, cursor + Fp * stalk])
        add_tube(self.parts['canes'], spts, 0.0007, 4, mat=1, cap=False)
        place(spts[-1], rng.normal(0, 0.1), 1.0)

    # ---------------- fruit ----------------

    def fruit_on_pedicel(self, origin, d, r):
        L = rng.uniform(0.014, 0.035)
        n = 7
        w = np.linspace(0, 1, n)[:, None] * rng.uniform(0.5, 0.95)
        dirs = normalize(d[None] * (1 - w) + np.array([0, 0, -1.0])[None] * w)
        pts = integrate(origin, dirs, L / (n - 1))
        add_tube(self.parts['canes'], pts, np.linspace(0.0009, 0.0007, n),
                 max(DETAIL['tube'] - 3, 4), mat=1, cap=False)
        axis = normalize(dirs[-1] * 0.35 + np.array([0, 0, -1.0]) * 0.65 + rng.normal(0, 0.12, 3))
        if r < 0:
            self.flower(pts[-1], axis, r)
        else:
            self.fruit(pts[-1], axis, r)

    def calyx(self, attach, axis, ex, ey, reflex, length, mat):
        base_ang = rng.uniform(0, 2 * np.pi)
        for k in range(5):
            ang = base_ang + k * 2 * np.pi / 5 + rng.normal(0, 0.12)
            rad = np.cos(ang) * ex + np.sin(ang) * ey
            a = reflex + rng.normal(0, 0.15)
            X = normalize(rad * math.cos(a) - axis * math.sin(a))
            tang = np.cross(axis, rad)
            Z = normalize(np.cross(X, tang))
            u = np.linspace(0, 1, 6)
            w = 0.0016 * (1 - u) ** 0.9 * (0.6 + 0.4 * np.sin(np.pi * np.clip(u * 2, 0, 1)))
            curl = rng.uniform(15, 45)
            add_blade(self.parts['calyx'], attach + rad * 0.0015, X, Z,
                      length * rng.uniform(0.85, 1.15), w, 3,
                      lambda U, V, xl, yl, c=curl: c * xl ** 2, (mat, mat), 0.0002)

    def fruit(self, attach, axis, r):
        stage = stage_of(r)
        ex, ey = perp(axis)
        size = (0.55 + 0.45 * smoothstep(0.0, 0.65, r)) * rng.uniform(0.85, 1.15)
        diameter = 0.020 * size
        Rmax = diameter * 0.5 * 0.84
        Lf = diameter * rng.uniform(0.9, 1.15)
        c, w1 = rng.uniform(0.28, 0.38), rng.uniform(0.40, 0.46)
        w2 = 1.0 - c

        sg = np.linspace(0, 1, 400)

        def prof(s):
            w = np.where(s < c, w1, w2)
            return Rmax * np.sqrt(np.clip(1 - ((s - c) / w) ** 2, 0, 1))

        Rg = prof(sg)
        zg = sg * Lf
        dR = np.gradient(Rg, zg)
        dA = 2 * np.pi * Rg * np.sqrt(1 + dR ** 2)
        cumA = np.concatenate([[0], np.cumsum(0.5 * (dA[1:] + dA[:-1]) * np.diff(zg))])
        A = cumA[-1]

        # drupelets on a phyllotactic spiral, uniform by area
        N = rng.randint(75, 116)
        idx = np.arange(N)
        s_i = np.interp((idx + 0.5) / N * A, cumA, sg)
        th = idx * GOLDEN_ANGLE + rng.normal(0, 0.05, N)
        R_i = np.interp(s_i, sg, Rg)
        slope = np.clip(np.interp(s_i, sg, dR), -8, 8)
        a = math.sqrt(A / (N * math.pi)) * 1.28 * rng.uniform(0.92, 1.08, N)
        nloc = normalize(np.stack([np.cos(th), np.sin(th), -slope], -1))
        surf = np.stack([R_i * np.cos(th), R_i * np.sin(th), s_i * Lf], -1)
        cen = surf + nloc * (0.12 * a)[:, None] + rng.normal(0, 0.04, (N, 3)) * a[:, None]
        t1 = np.stack([-np.sin(th), np.cos(th), np.zeros(N)], -1)
        t2 = np.cross(nloc, t1)
        basis = np.stack([t1, t2, nloc], 1)                      # (N, 3, 3) rows
        sc = np.stack([a, a, a * 0.78 * rng.uniform(0.9, 1.1, N)], -1)
        tv, tris, quads = DRUPELET
        K = len(tv)
        local = cen[:, None, :] + np.einsum('nkj,njl->nkl', tv[None] * sc[:, None, :], basis)
        R = np.stack([ex, ey, axis])
        world = attach + local.reshape(-1, 3) @ R
        # per-drupelet colour: base (near the calyx) lags behind, plus noise
        u_i = (r - 0.10 * (1 - s_i) * smoothstep(0.2, 0.8, r) +
               rng.normal(0, 0.03, N)) / R_MAX
        u_i = np.clip(u_i, 0.005, 0.995)
        uv = np.stack([np.repeat(u_i, K), np.tile((tv[:, 2] + 1) * 0.5, N)], -1)
        mb = self.parts['fruit_' + stage]
        off = (np.arange(N) * K)[:, None, None]
        mb.add(world, uv, [(tris[None] + off).reshape(-1, 3), (quads[None] + off).reshape(-1, 4)])

        # dark core (receptacle) filling the gaps between drupelets
        cs = DETAIL['core_seg']
        sc_ = np.linspace(0.0, 1.0, 10)
        rc = prof(sc_) * 0.97
        thc = np.linspace(0, 2 * np.pi, cs, endpoint=False)
        ring = np.stack([np.outer(rc, np.cos(thc)), np.outer(rc, np.sin(thc)),
                         np.outer(sc_ * Lf, np.ones(cs))], -1).reshape(-1, 3)
        verts = np.vstack([ring, [[0, 0, -0.03 * Lf]], [[0, 0, Lf]]])
        top, bot = len(ring), len(ring) + 1
        j = np.arange(cs)
        jn = (j + 1) % cs
        faces = [np.stack([np.full(cs, top), j, jn], 1),
                 np.stack([np.full(cs, bot), 9 * cs + jn, 9 * cs + j], 1)]
        for k in range(9):
            faces.append(np.stack([k * cs + j, k * cs + jn, (k + 1) * cs + jn], 1))
            faces.append(np.stack([k * cs + j, (k + 1) * cs + jn, (k + 1) * cs + j], 1))
        faces = np.vstack(faces)
        fc = verts[faces].mean(axis=1)
        faces = orient_outward(verts, faces, np.stack([np.zeros(len(fc)), np.zeros(len(fc)), fc[:, 2]], -1))
        mb.add(attach + verts @ R, np.tile([[np.clip(r / R_MAX, 0.005, 0.995), 0.02]], (len(verts), 1)),
               faces)

        # calyx: hugging the green fruit, reflexed on ripe fruit, dry on overripe
        reflex = float(np.interp(r, [0, 0.5, 0.8], [-0.35, 0.3, 0.9]))
        self.calyx(attach, axis, ex, ey, reflex, rng.uniform(0.006, 0.009),
                   mat=1 if r > 0.95 else 0)

        center = attach + axis * Lf * 0.5
        self.markers.append({
            'marker_type': 'FRUIT',
            'translation': center.tolist(),
            'stage': stage,
            'ripeness': round(float(r), 4),
            'attach_point': attach.tolist(),
            'axis': axis.tolist(),
            'length': round(float(Lf + 0.9 * a.mean()), 5),
            'diameter': round(float(2 * (Rmax + 0.9 * a.mean())), 5),
            'drupelets': int(N),
        })
        if args.fruit_collisions:
            self.collisions.append(('sphere', center, float(Rmax)))

    def flower(self, attach, axis, r):
        ex, ey = perp(axis)
        R = np.stack([ex, ey, axis])
        self.calyx(attach, axis, ex, ey, rng.uniform(0.0, 0.4), rng.uniform(0.006, 0.008), 0)
        open_ = smoothstep(-0.3, -0.05, r)    # buds -> open flowers
        base_ang = rng.uniform(0, 2 * np.pi) + np.pi / 5
        for k in range(5):
            ang = base_ang + k * 2 * np.pi / 5 + rng.normal(0, 0.1)
            rad = np.cos(ang) * ex + np.sin(ang) * ey
            a = -1.2 + 1.5 * open_ + rng.normal(0, 0.12)
            X = normalize(rad * math.cos(a) + axis * math.sin(-a))
            Z = normalize(np.cross(X, np.cross(axis, rad)))
            u = np.linspace(0, 1, 7)
            w = 0.0027 * np.sin(np.pi * np.clip(u, 0, 1) ** 1.5) + 0.0004
            w[-1] = 0.0008
            add_blade(self.parts['flowers'], attach + rad * 0.002 + axis * 0.001, X, Z,
                      rng.uniform(0.0045, 0.0065), w, 3,
                      lambda U, V, xl, yl: -12 * xl ** 2 + 0.25 * np.abs(yl), (0, 0), 0.0002)
        # pistil dome
        cs = 10
        sc_ = np.linspace(0, 1, 5)
        rr = 0.0028 * np.sqrt(np.clip(1 - sc_ ** 2, 0, 1))
        thc = np.linspace(0, 2 * np.pi, cs, endpoint=False)
        ring = np.stack([np.outer(rr, np.cos(thc)), np.outer(rr, np.sin(thc)),
                         np.outer(0.001 + sc_ * 0.0032, np.ones(cs))], -1).reshape(-1, 3)
        j = np.arange(cs)
        jn = (j + 1) % cs
        faces = []
        for k in range(4):
            faces.append(np.stack([k * cs + j, k * cs + jn, (k + 1) * cs + jn], 1))
            faces.append(np.stack([k * cs + j, (k + 1) * cs + jn, (k + 1) * cs + j], 1))
        faces = orient_outward(ring, np.vstack(faces), np.zeros(3))
        self.parts['flowers'].add(attach + ring @ R, np.full((len(ring), 2), 0.5), faces, 1)
        # stamens: a ring of thin filaments splaying out around the pistils
        ns = 36
        t = rng.uniform(0, 2 * np.pi) + np.arange(ns) * 2 * np.pi / ns
        rad = np.outer(np.cos(t), ex) + np.outer(np.sin(t), ey)
        base = attach + axis * 0.0012 + rad * 0.0026
        d = normalize(rad * rng.uniform(0.6, 1.0, (ns, 1)) + axis * rng.uniform(0.5, 1.0, (ns, 1)))
        add_spikes(self.parts['flowers'], base, d, np.tile(axis, (ns, 1)),
                   rng.uniform(0.0025, 0.0038, ns), np.full(ns, 0.00022), 2)
        self.markers.append({
            'marker_type': 'FLOWER',
            'translation': (attach + axis * 0.003).tolist(),
            'stage': 'flower' if open_ > 0.5 else 'bud',
            'ripeness': round(float(r), 4),
            'attach_point': attach.tolist(),
            'axis': axis.tolist(),
        })


# --------------------------------------------------------------------------- #
# Model assembly + export
# --------------------------------------------------------------------------- #

def gen_plant():
    plant = Plant()
    n_flor = args.floricanes if args.floricanes is not None else rng.randint(4, 8)
    n_prim = args.primocanes if args.primocanes is not None else rng.randint(2, 6)
    for k in range(n_flor + n_prim):
        # hedgerow: canes leaning mostly along the row (x axis), crown ~15 cm wide
        side = 0.0 if rng.rand() < 0.5 else np.pi
        azim = side + rng.normal(0, 0.6)
        base = np.array([rng.normal(0, 0.05), rng.normal(0, 0.035), 0.0])
        if k < n_flor:
            plant.floricane(base, azim)
        else:
            plant.primocane(base, azim)
    return plant


def write_sdf(model_dir, name, parts, collisions):
    vis = []
    for part in parts:
        vis.append(f"""      <visual name="{part}">
        <geometry>
          <mesh>
            <uri>model://{name}/meshes/{part}.glb</uri>
          </mesh>
        </geometry>
        <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
          <label>{LABELS[part]}</label>
        </plugin>
      </visual>""")
    col = []
    for k, c in enumerate(collisions):
        if c[0] == 'cylinder':
            _, p0, p1, r = c
            d = p1 - p0
            L = float(np.linalg.norm(d))
            if L < 1e-4:
                continue
            m = (p0 + p1) / 2
            pitch = math.atan2(math.hypot(d[0], d[1]), d[2])
            yaw = math.atan2(d[1], d[0])
            geom = f'<cylinder><radius>{r:.4f}</radius><length>{L:.4f}</length></cylinder>'
            pose = f'{m[0]:.4f} {m[1]:.4f} {m[2]:.4f} 0 {pitch:.4f} {yaw:.4f}'
        else:
            _, m, r = c
            geom = f'<sphere><radius>{r:.4f}</radius></sphere>'
            pose = f'{m[0]:.4f} {m[1]:.4f} {m[2]:.4f} 0 0 0'
        col.append(f"""      <collision name="c{k}">
        <pose>{pose}</pose>
        <geometry>{geom}</geometry>
      </collision>""")
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
{chr(10).join(col)}
{chr(10).join(vis)}
    </link>
  </model>
</sdf>
"""
    (model_dir / 'model.sdf').write_text(sdf)
    (model_dir / 'model.config').write_text(f"""<?xml version="1.0" ?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>raspberry_field generator</name></author>
  <description>Procedural raspberry plant (seed {args.seed}).</description>
</model>
""")


def export_glb(ob, path):
    for o in bpy.context.scene.objects:
        o.select_set(False)
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob
    bpy.ops.export_scene.gltf(filepath=str(path), export_format='GLB', use_selection=True,
                              export_yup=args.gltf_yup, export_apply=True)


def main():
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob, do_unlink=True)

    model_dir = Path(args.model_dir).resolve()
    (model_dir / 'meshes').mkdir(parents=True, exist_ok=True)
    mats = build_materials()
    plant = gen_plant()

    exported = []
    stats = {}
    for part, mb in plant.parts.items():
        if mb.empty:
            continue
        key = 'fruit' if part.startswith('fruit_') else part
        ob = mb.to_object(mats[key])
        export_glb(ob, model_dir / 'meshes' / f'{part}.glb')
        exported.append(part)
        stats[part] = mb.face_count()

    write_sdf(model_dir, model_dir.name, exported, plant.collisions)
    with open(model_dir / 'markers.json', 'w') as f:
        json.dump(plant.markers, f, indent=2)

    counts = {}
    for m in plant.markers:
        counts[m['stage']] = counts.get(m['stage'], 0) + 1
    print('[raspberry_gen] model={} seed={} faces={} total={} fruits={}'.format(
        model_dir.name, args.seed, stats, sum(stats.values()), counts))


if __name__ == '__main__':
    main()
