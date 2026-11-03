# Scale MS-Human-700 (MuJoCo Menagerie, Apache-2.0) to a character's limb segments, the way MuscleMimic's human_scale_table does:
# the height first (one global factor from the leg length), then each limb on its own (the thigh, shin, foot, upper arm and forearm
# factors). Every body gets one uniform factor; its offsets, geoms, sites, joints, mesh scale and inertia follow it, so the
# muscles' paths (sites on bodies) scale with the bones. The muscles are not fitted from geometry.
# Targets come from body_frames.py's json (the finch's 23 body frames). Segment lengths are pose-invariant, so an A-pose rig
# and a model that stands with its arms down compare directly.
#   uv run --with mujoco python fit_msh700.py <MS-Human-700.xml> <finch_frames.json> <out.xml> [--plant-forearm 1.3]
import json, sys
import numpy as np
import mujoco as mj
xml, frames, out = sys.argv[1:4]
plant = float(sys.argv[sys.argv.index("--plant-forearm") + 1]) if "--plant-forearm" in sys.argv else None
F = json.load(open(frames))["frames"]
P = lambda n: np.array(F[n]["position"])
# segment: (model body A, model body B, character joint A, character joint B); both ends are body origins at the model's rest pose
SEG = {
    "thigh": ("femur_{s}", "tibia_{s}", "{S}Leg", "{S}Shin"),
    "shin": ("tibia_{s}", "calcn_{s}", "{S}Shin", "{S}Foot"),
    "foot": ("calcn_{s}", "toes_{s}", "{S}Foot", "{S}ToeBase"),
    "upperarm": ("humerus_{s}", "ulna_{s}", "{S}Arm", "{S}ForeArm"),
    "forearm": ("ulna_{s}", "hand_{s}", "{S}ForeArm", "{S}Hand"),
}
SIDES = (("l", "Left"), ("r", "Right"))
# which segment's factor each body takes; every other body takes the global (leg length) factor
GROUP = {"femur": "thigh", "tibia": "shin", "talus": "shin", "calcn": "foot", "toes": "foot", "humerus": "upperarm", "ulna": "forearm", "radius": "forearm", "proximal_row": "forearm"}


def lengths(model_xml_or_spec):
    spec = model_xml_or_spec
    m = spec.compile()
    d = mj.MjData(m)
    mj.mj_forward(m, d)
    pos = lambda n: d.xpos[mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, n)]
    out = {}
    for seg, (a, b, _, _) in SEG.items():
        for s, _ in SIDES:
            out[f"{seg}_{s}"] = float(np.linalg.norm(pos(a.format(s=s)) - pos(b.format(s=s))))
    out["hip_width"] = float(np.linalg.norm(pos("femur_l") - pos("femur_r")))
    out["shoulder_width"] = float(np.linalg.norm(pos("humerus_l") - pos("humerus_r")))
    return out, m, d


def targets():
    t = {}
    for seg, (_, _, ca, cb) in SEG.items():
        for s, S in SIDES:
            t[f"{seg}_{s}"] = float(np.linalg.norm(P(ca.format(S=S)) - P(cb.format(S=S))))
    t["hip_width"] = float(np.linalg.norm(P("LeftLeg") - P("RightLeg")))
    t["shoulder_width"] = float(np.linalg.norm(P("LeftArm") - P("RightArm")))
    return t


def factors_for(model_len, tgt):
    # height first: the leg length ratio is the global factor; each limb segment then takes its own ratio (left and right averaged)
    g = (tgt["thigh_l"] + tgt["shin_l"] + tgt["thigh_r"] + tgt["shin_r"]) / (model_len["thigh_l"] + model_len["shin_l"] + model_len["thigh_r"] + model_len["shin_r"])
    f = {"global": g}
    for seg in SEG:
        f[seg] = 0.5 * sum(tgt[f"{seg}_{s}"] / model_len[f"{seg}_{s}"] for s, _ in SIDES)
    return f


def apply(spec, f):
    body_f = {"world": 1.0}
    meshes_done = set()
    mesh_scale = {}
    for b in spec.bodies:
        if b.name == "world":
            continue
        base = b.name.rsplit("_", 1)[0]
        own = f[GROUP[base]] if base in GROUP else f["global"]
        parent = b.parent.name if b.parent is not None else "world"
        fp = body_f.get(parent, 1.0)
        body_f[b.name] = own
        b.pos = np.array(b.pos) * fp
        b.ipos = np.array(b.ipos) * own
        if b.explicitinertial:
            b.mass = b.mass * own**3
            b.inertia = np.array(b.inertia) * own**5
        for g in b.geoms:
            g.pos = np.array(g.pos) * own
            g.size = np.array(g.size) * own
            if g.type == mj.mjtGeom.mjGEOM_MESH and g.meshname not in meshes_done:
                meshes_done.add(g.meshname)
                mesh_scale[g.meshname] = own
        for s in b.sites:
            s.pos = np.array(s.pos) * own
            s.size = np.array(s.size) * own
        for j in b.joints:
            j.pos = np.array(j.pos) * own
    for mesh in spec.meshes:
        if mesh.name in mesh_scale:
            mesh.scale = np.array(mesh.scale) * mesh_scale[mesh.name]
    return body_f


tgt = targets()
spec0 = mj.MjSpec.from_file(xml)
before, m0, d0 = lengths(spec0)
fac = factors_for(before, tgt)
spec = mj.MjSpec.from_file(xml)
used = dict(fac)
if plant is not None:
    used["forearm"] *= plant   # the planted fault: one limb scale wrong
apply(spec, used)
after, m1, d1 = lengths(spec)
print("scale factors:", {k: round(v, 4) for k, v in fac.items()}, "(forearm planted x%.2f)" % plant if plant else "")
print("segment                 target  model before -> after   (relative error)")
worst = 0.0
for k in sorted(tgt):
    e0 = before[k] / tgt[k] - 1
    e1 = after[k] / tgt[k] - 1
    scaled = k not in ("hip_width", "shoulder_width")
    worst = max(worst, abs(e1) if scaled else 0.0)
    print(f"  {k:16s} {tgt[k]:7.4f}  {before[k]:7.4f} -> {after[k]:7.4f}   {100*e0:+7.1f}% -> {100*e1:+7.1f}%" + ("" if scaled else "   (width: uniform scaling cannot reach it, reported only)"))
print(f"GATE limb segments match the character: worst relative error {worst:.2e} (tolerance 1e-3): {'PASS' if worst < 1e-3 else 'FAIL'}")
tmass0, tmass1 = float(m0.body_mass.sum()), float(m1.body_mass.sum())
print(f"GATE model valid: {m1.nbody} bodies, {m1.nu} muscles, mass {tmass0:.1f} -> {tmass1:.1f} kg, forward step ok: {np.isfinite(d1.xpos).all()}")
if plant is None:
    open(out, "w").write(spec.to_xml())
    print("scaled model written to", out)
else:
    print("(planted run: nothing written)")
sys.exit(0 if worst < 1e-3 else 1)
