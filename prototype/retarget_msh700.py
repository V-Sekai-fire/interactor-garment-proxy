# Retarget Kimodo/SOMA30 motion clips onto MS-Human-700 (MuJoCo Menagerie, Apache-2.0) by solving joint angles per frame.
# The clip is the 30-joint SOMA30 skeleton: per frame and joint 12 float32, a row-major 3x3 local rotation then the local offset
# (the layout Retarget.lean in interactor-motion-retarget reads: rotation at +0..8, offset at +9..11; Y up, +Z forward, +X left).
# Method: SOMA joints give 17 sites (pelvis, spine1, head, shoulder, elbow, wrist, hip, knee, ankle, toes; the table of MuscleMimic's
# smplh_to_myofullbody.json, with head_neck for the head (SOMA Neck2: the model's neck joints sit at head_neck, so only a point beyond it sees the neck) plus a skull point 5 cm up the head_neck body fitted to SOMA Head, and hand_* for lunate_*). Targets are built down the site tree with one
# scale per segment (model length / clip length), so the target is reachable in bone length and the gate measures the pose. The
# per-frame joint angles come from bounded least squares (scipy L-BFGS-B) on the 17 site positions, bounds = the model's joint ranges.
# Every rotation here is a 3x3 matrix (MuJoCo xmat for the model); no quaternions.
# Free joints: pelvis 6, hip 3, knee, ankle, subtalar, lumbar/thoracic/neck 3 each, shoulder (elv_angle, shoulder_elv, shoulder_rot),
# elbow_flexion, pro_sup, wrist 2. The coupled joints (knee translations, patella, clavicle, scapula) stay at 0.
# Gates, each with a control that must fail: (1) site errors under a tolerance, control = shins planted 30% wrong;
# (2) knee and elbow flexion inside the model's ranges, control = a solution with flexion pushed out of range.
# Result on the four clips (see the PR): walk, run and jump_forward pass; crouch_walk fails gate 1 (head mean 0.036 m) because the spine
# joints L5_S1_FE and T12_L1_FE sit on their -0.5 rad limit for the whole clip: the model's spine bends less than the clip's crouch.
#   uv run --with mujoco --with numpy --with scipy python retarget_msh700.py <MS-Human-700.xml> <motion_dir> [--clips walk,run,...] [--out=DIR]
import json
import os
import sys

import mujoco
import numpy as np
from scipy.optimize import minimize

TOL_MEAN = 0.030  # m, per-site mean over the frames. 1.8% of a 1.7 m body; the arms and spine of SOMA are not rigid like the model's
TOL_P99 = 0.060  # m, per-site 99th percentile over the frames. The shortest segment (ankle to toes, 0.13 m) planted 30% wrong is 0.04 m;
# the control plants the shin (0.40 m, 0.12 m error) so it must clear the p99 bound twice over
CLIPS = ["walk", "run", "crouch_walk", "jump_forward"]
PLANT = 1.3  # the planted fault: shin segment scale 30% wrong
KNEE_RANGE_DEG, ELBOW_RANGE_DEG = 138.0, 126.0  # as the task states them; read back from the model and compared

# (site, model point, SOMA point, parent site). ("mid", a, b) is the midpoint of two bodies / joints.
SITES = [
    ("pelvis", ("mid", "femur_r", "femur_l"), ("mid", "RightLeg", "LeftLeg"), None),
    ("spine1", "lumbar1", "Chest", "pelvis"),
    ("head", "head_neck", "Neck2", "spine1"),
    ("skull", ("point", "head_neck", 0.05), "Head", "head"),
]
for side, S in (("l", "Left"), ("r", "Right")):
    SITES += [
        (f"{side}_shoulder", f"humerus_{side}", f"{S}Arm", "spine1"),
        (f"{side}_elbow", f"ulna_{side}", f"{S}ForeArm", f"{side}_shoulder"),
        (f"{side}_wrist", f"hand_{side}", f"{S}Hand", f"{side}_elbow"),
        (f"{side}_hip", f"femur_{side}", f"{S}Leg", "pelvis"),
        (f"{side}_knee", f"tibia_{side}", f"{S}Shin", f"{side}_hip"),
        (f"{side}_ankle", f"talus_{side}", f"{S}Foot", f"{side}_knee"),
        (f"{side}_toes", f"toes_{side}", f"{S}ToeBase", f"{side}_ankle"),
    ]
SITE_NAMES = [s[0] for s in SITES]
FREE = (["pelvis_tz", "pelvis_ty", "pelvis_tx", "pelvis_tilt", "pelvis_list", "pelvis_rotation"]
        + [f"{j}_{s}" for s in "rl" for j in ("hip_flexion", "hip_adduction", "hip_rotation", "knee_angle", "ankle_angle", "subtalar_angle")]
        + ["L5_S1_FE", "L5_S1_LB", "L5_S1_AR", "T12_L1_FE", "T12_L1_LB", "T12_L1_AR", "T1_head_neck_FE", "T1_head_neck_LB", "T1_head_neck_AR"]
        + [f"{j}_{s}" for s in "rl" for j in ("elv_angle", "shoulder_elv", "shoulder_rot", "elbow_flexion", "pro_sup", "deviation", "flexion")])
ANKLE_SOLE = 0.055  # [m] ankle joint to the sole, the model's colliding ankle height (SOMA feet are the same scale)
RESTART_VAL = 0.02  # [m^2] a frame whose solve ends above this restarts from other poses when Rig.restart is set
W_FLOOR = 100.0  # weight of the foot-capsule floor penalty against the squared site error (both m^2)
FLOOR_MARGIN = 0.001  # [m] keep the colliding foot capsules this far above z = 0
TOL_FLOOR, TOL_FLOOR_FRAC = 0.002, 0.05  # gate: deepest capsule point [m] and the fraction of frames more than 1 mm under
LAMBDA_PREV, LAMBDA_ZERO = 1e-5, 1e-5  # tie the angles the 17 positions cannot see (twists, wrist) to the last frame and to zero
# SOMA (x left, y up, z forward) to the model (x forward, y left, z up): a cyclic permutation, det +1
C = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def load_clip(motion_dir, name):
    a = np.fromfile(os.path.join(motion_dir, name + ".f32"), "<f4")
    return a.reshape(-1, 30, 12).astype(np.float64)


def soma_fk(clip, parents):
    """World positions (F,30,3) and rotations (F,30,3,3): R_w = R_parent_w @ R_local, p_w = p_parent_w + R_parent_w @ offset."""
    F = clip.shape[0]
    P, R = np.zeros((F, 30, 3)), np.zeros((F, 30, 3, 3))
    for j in range(30):
        Rl, t = clip[:, j, :9].reshape(F, 3, 3), clip[:, j, 9:]
        if parents[j] < 0:
            R[:, j], P[:, j] = Rl, t
        else:
            R[:, j] = R[:, parents[j]] @ Rl
            P[:, j] = P[:, parents[j]] + np.einsum("fij,fj->fi", R[:, parents[j]], t)
    return P, R


def soma_sites(P, names):
    idx = {n: i for i, n in enumerate(names)}
    out = np.zeros((P.shape[0], len(SITES), 3))
    for k, (_, _, sp, _) in enumerate(SITES):
        out[:, k] = (P[:, idx[sp[1]]] + P[:, idx[sp[2]]]) / 2 if isinstance(sp, tuple) else P[:, idx[sp]]
    return out


def yaw_matrix(R0):
    """Rotation about SOMA up (Y) that turns the first frame's pelvis forward (R0 @ +Z) to +Z."""
    f = R0 @ np.array([0.0, 0.0, 1.0])
    a = np.arctan2(f[0], f[2])
    c, s = np.cos(-a), np.sin(-a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


class Rig:
    def __init__(self, xml):
        self.m = mujoco.MjModel.from_xml_path(os.path.abspath(xml))
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.free = np.array([m.joint(n).qposadr[0] for n in FREE])
        assert all(m.jnt_type[m.joint(n).id] in (2, 3) for n in FREE) and m.nq == m.nv  # slide/hinge only: qpos index = dof index
        self.lo = m.jnt_range[[m.joint(n).id for n in FREE], 0].copy()
        self.hi = m.jnt_range[[m.joint(n).id for n in FREE], 1].copy()
        self.bodies = []
        for _, mp, _, _ in SITES:
            if isinstance(mp, tuple) and mp[0] == "point":
                self.bodies.append([m.body(mp[1]).id])
                continue
            names = mp[1:] if isinstance(mp, tuple) else (mp,)
            self.bodies.append([m.body(n).id for n in names])
        mujoco.mj_forward(m, self.d)
        self.points = {}  # site index -> (body id, offset in the body frame): a point on the body, rigid with it
        for k, (_, mp, _, _) in enumerate(SITES):
            if isinstance(mp, tuple) and mp[0] == "point":
                b = m.body(mp[1]).id
                self.points[k] = (b, self.d.xmat[b].reshape(3, 3).T @ np.array([0.0, 0.0, mp[2]]))
        self.rest = self.sites()
        # colliding capsules on the calcn and toes: the floor term keeps their lowest points above z = 0 (the retarget fits 17 site
        # positions only, so foot pitch is free and the capsules otherwise sink ~2 cm)
        self.caps = [(g, int(m.geom_bodyid[g])) for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE
                     and (m.geom_contype[g] or m.geom_conaffinity[g]) and m.body(m.geom_bodyid[g]).name.split("_")[0] in ("calcn", "toes")]
        self.w_floor = W_FLOOR
        self.restart, self.restarts_used = False, 0  # opt-in: restart a stuck frame from other poses (Rig.solve)
        self.ground = None  # optional height of the ground under a world xy (stairs, ramps); None is the flat floor at z = 0

    def sites(self):
        out = np.array([np.mean([self.d.xpos[b] for b in bs], axis=0) for bs in self.bodies])
        for k, (b, off) in self.points.items():
            out[k] = self.d.xpos[b] + self.d.xmat[b].reshape(3, 3) @ off
        return out

    def sites_jac(self):
        """Site positions (17,3) and their Jacobian w.r.t. the free joints (17*3, nfree), after mj_kinematics."""
        m, d = self.m, self.d
        mujoco.mj_comPos(m, d)  # cdof, which mj_jacBody reads
        J = np.zeros((len(SITES), 3, m.nv))
        jp = np.zeros((3, m.nv))
        for k, bs in enumerate(self.bodies):
            for b in bs:
                if k in self.points:
                    mujoco.mj_jac(m, d, jp, None, d.xpos[b] + d.xmat[b].reshape(3, 3) @ self.points[k][1], b)
                else:
                    mujoco.mj_jacBody(m, d, jp, None, b)
                J[k] += jp / len(bs)
        return self.sites(), J[:, :, self.free].reshape(-1, len(self.free))

    def floor_terms(self):
        """Sum of squared depths of the capsule bottom end points under FLOOR_MARGIN, and its gradient over the free joints.
        Call after mj_kinematics and mj_comPos."""
        m, d = self.m, self.d
        val, grad, jp = 0.0, np.zeros(len(self.free)), np.zeros((3, m.nv))
        for g, b in self.caps:
            Rg, p = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
            half, rad = m.geom_size[g][1], m.geom_size[g][0]
            for sign in (-1.0, 1.0):
                pt = p + sign * Rg[:, 2] * half
                viol = FLOOR_MARGIN - (pt[2] - rad - (self.ground(pt[:2]) if self.ground else 0.0))
                if viol > 0:
                    mujoco.mj_jac(m, d, jp, None, pt, b)
                    val += viol * viol
                    grad -= 2.0 * viol * jp[2, self.free]
        return val, grad

    def floor_depth(self, q):
        """(F,) deepest point of the colliding foot capsules below z = 0 [m], 0 when above."""
        m, d = self.m, self.d
        out = np.zeros(q.shape[0])
        for f in range(q.shape[0]):
            d.qpos[:] = q[f]
            mujoco.mj_kinematics(m, d)
            for g, _ in self.caps:
                Rg, p = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
                for sign in (-1.0, 1.0):
                    pt = p + sign * Rg[:, 2] * m.geom_size[g][1]
                    low = pt[2] - m.geom_size[g][0] - (self.ground(pt[:2]) if self.ground else 0.0)
                    out[f] = max(out[f], -low)
        return out

    def solve(self, targets, bounded=True, tag=""):
        """Per-frame bounded least squares over the free joints, each frame warm-started from the one before."""
        m, d, F = self.m, self.d, targets.shape[0]
        q = np.zeros((F, m.nq))
        x = np.zeros(len(self.free))
        x[:3] = 0.0
        bounds = list(zip(self.lo, self.hi)) if bounded else None
        if bounds:  # the root slides carry no anatomical range (the +-5 m is a placeholder), the clip's travel sets them
            bounds[:3] = [(None, None)] * 3
            # the heading hinge is unlimited too: the model declares pelvis_rotation +-90 deg with limited="false", and the clip's
            # heading (only the first frame's is removed) turns further than that in a circle, a pivot or a long take
            bounds[FREE.index("pelvis_rotation")] = (None, None)
        for f in range(F):
            tgt, prev = targets[f].reshape(-1), x.copy()

            def fun(v):
                d.qpos[:] = 0.0
                d.qpos[self.free] = v
                mujoco.mj_kinematics(m, d)
                s, J = self.sites_jac()
                r = s.reshape(-1) - tgt
                val = r @ r + LAMBDA_PREV * np.sum((v - prev) ** 2) + LAMBDA_ZERO * np.sum(v**2)
                grad = 2 * J.T @ r + 2 * LAMBDA_PREV * (v - prev) + 2 * LAMBDA_ZERO * v
                if self.w_floor:
                    fv, fg = self.floor_terms()
                    val, grad = val + self.w_floor * fv, grad + self.w_floor * fg
                return val, grad

            if f == 0:  # root slides first (pelvis site onto its target), then everything
                d.qpos[:] = 0.0
                mujoco.mj_kinematics(m, d)
                s, J = self.sites_jac()
                x[:3] = np.linalg.lstsq(J[:3, :3], tgt[:3] - s[0], rcond=None)[0]
            res = minimize(fun, x, jac=True, method="L-BFGS-B", bounds=bounds, options={"maxiter": 400, "ftol": 1e-14, "gtol": 1e-9})
            if self.restart and res.fun > RESTART_VAL:  # stuck in a local minimum (a toe caught on a riser): start again from other poses
                d.qpos[:] = 0.0
                mujoco.mj_kinematics(m, d)
                s0, J0 = self.sites_jac()
                slides = np.linalg.lstsq(J0[:3, :3], tgt[:3] - s0[0], rcond=None)[0]
                for start in (np.r_[slides, x[3:]], np.r_[slides, np.zeros(len(x) - 3)]):
                    alt = minimize(fun, start, jac=True, method="L-BFGS-B", bounds=bounds, options={"maxiter": 400, "ftol": 1e-14, "gtol": 1e-9})
                    if alt.fun < res.fun:
                        res = alt
                self.restarts_used += 1
            x = res.x
            q[f, self.free] = x
        return q

    def site_track(self, q):
        out = np.zeros((q.shape[0], len(SITES), 3))
        for f in range(q.shape[0]):
            self.d.qpos[:] = q[f]
            mujoco.mj_kinematics(self.m, self.d)
            out[f] = self.sites()
        return out


def ground_offset(P, names, fps, rest_h=0.013, dead=0.06, window_s=0.75, smooth_s=0.5):
    """Ground height under the clip, per frame, when it is lower than the flat floor the model stands on (model frame, z up).

    A source recorded with a step down has feet far below z = 0 although they stand on something, and no foot is ever below the
    ground it stands on, so only the low side is certain. The lowest foot point (toe base, or ankle less the ankle-to-sole
    height) is minimised over +-window_s (a flight phase has ground within that window on both sides), measured against
    rest_h, the sole height of a standing foot in the source. Offsets above -dead are dropped, so a clip on the flat floor
    returns zeros and is untouched; the rest is averaged over smooth_s so the step down is a ramp, not a jump. Higher ground
    (a box) is left alone: feet on it stay where the source put them."""
    from scipy.ndimage import minimum_filter1d, uniform_filter1d

    ix = {n: i for i, n in enumerate(names)}
    h = np.full(len(P), np.inf)
    for side in ("Left", "Right"):
        toe, foot = P[:, ix[f"{side}ToeBase"], 2], P[:, ix[f"{side}Foot"], 2]
        h = np.minimum(h, np.minimum(toe, foot - ANKLE_SOLE))
    low = minimum_filter1d(h, size=2 * int(round(window_s * fps)) + 1, mode="nearest") - rest_h
    g = np.minimum(low + dead, 0.0)
    return uniform_filter1d(g, size=max(int(round(smooth_s * fps)), 1), mode="nearest")


def segment_scales(rig, S):
    """Per site: model rest segment length / median clip segment length (parent -> site); the root scale is the mean leg ratio."""
    sc = {}
    ix = {n: i for i, n in enumerate(SITE_NAMES)}
    for k, (n, _, _, par) in enumerate(SITES):
        if par is None:
            continue
        ml = np.linalg.norm(rig.rest[k] - rig.rest[ix[par]])
        cl = np.median(np.linalg.norm(S[:, k] - S[:, ix[par]], axis=1))
        sc[n] = ml / cl
    sc["root"] = float(np.mean([sc[f"{s}_{j}"] for s in "lr" for j in ("knee", "ankle")]))
    return sc


def build_targets(S, sc):
    """S: clip sites (F,17,3) in the model frame. Targets down the tree: parent target + segment scale * clip segment vector."""
    ix = {n: i for i, n in enumerate(SITE_NAMES)}
    T = np.zeros_like(S)
    T[:, 0] = sc["root"] * (S[:, 0] - np.array([S[0, 0, 0], S[0, 0, 1], 0.0]))  # start at the horizontal origin; height scaled
    for k, (n, _, _, par) in enumerate(SITES):
        if par is not None:
            T[:, k] = T[:, ix[par]] + sc[n] * (S[:, k] - S[:, ix[par]])
    return T


def site_errors(track, T):
    return np.linalg.norm(track - T, axis=2)  # (F,17)


def range_report(rig, q):
    """Knee and elbow flexion (rad) per side against the ranges stored in the model."""
    m = rig.m
    rep = {}
    for j in ("knee_angle", "elbow_flexion"):
        for s in "rl":
            jid = m.joint(f"{j}_{s}").id
            rep[f"{j}_{s}"] = (q[:, m.jnt_qposadr[jid]], m.jnt_range[jid])
    return rep


def flexion_gate(rep, eps=1e-6):
    bad = [k for k, (v, r) in rep.items() if v.min() < r[0] - eps or v.max() > r[1] + eps]
    return not bad, bad


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) < 2:
        sys.exit(__doc__ or "usage: retarget_msh700.py <MS-Human-700.xml> <motion_dir> [--clips a,b]")
    xml, mdir = args[0], args[1]
    clips = CLIPS
    fps = next((float(a.split("=", 1)[1]) for a in sys.argv[1:] if a.startswith("--fps=")), 30.0)  # the clips' frame rate, recorded in the json
    skip_controls = "--skip-controls" in sys.argv  # bulk runs: gates only, the controls are three extra solves per clip
    out_dir = next((a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--out=")), None)
    for a in sys.argv[1:]:
        if a.startswith("--clips="):
            clips = a.split("=", 1)[1].split(",")
    names = open(os.path.join(mdir, "skeleton.txt")).read().split()
    parents = [int(x) for x in open(os.path.join(mdir, "parents.txt")).read().split()]
    rig = Rig(xml)
    m = rig.m
    kr, er = m.jnt_range[m.joint("knee_angle_r").id], m.jnt_range[m.joint("elbow_flexion_r").id]
    ok_rng = abs(np.degrees(kr[1]) - KNEE_RANGE_DEG) < 0.5 and abs(np.degrees(er[1]) - ELBOW_RANGE_DEG) < 0.5 and kr[0] == 0 == er[0]
    print(f"model ranges: knee 0..{np.degrees(kr[1]):.1f} deg, elbow 0..{np.degrees(er[1]):.1f} deg (stated 138 / 126): {'ok' if ok_rng else 'MISMATCH'}")
    fails = [] if ok_rng else ["model ranges differ from the stated ones"]
    print(f"tolerance: per-site mean <= {TOL_MEAN} m and per-site p99 <= {TOL_P99} m; control = shin scale x{PLANT}")
    for name in clips:
        clip = load_clip(mdir, name)
        P, R = soma_fk(clip, parents)
        Rm = C @ yaw_matrix(R[0, 0])  # SOMA world -> model frame, heading removed
        P = np.einsum("ij,fnj->fni", Rm, P)
        if "--ground" in sys.argv:  # opt-in: remove the ground height of a clip recorded on uneven ground
            off = ground_offset(P, names, fps)
            P[:, :, 2] -= off[:, None]
            print(f"  ground offset: min {off.min():+.3f} m, max {off.max():+.3f} m, frames moved {int((off != 0).sum())} of {len(off)}")
        S = soma_sites(P, names)
        sc = segment_scales(rig, S)
        T = build_targets(S, sc)
        T_plain = T
        if "--plant" in sys.argv:  # foot planting, off by default (plant_feet.py, after interactor-xr-pilot Body.cpp) on the targets
            from plant_feet import foot_floor_heights, plant_feet

            ix = {n: i for i, n in enumerate(SITE_NAMES)}
            feet = {s: {"ankle": ix[f"{s}_ankle"], "toes": ix[f"{s}_toes"], "hip": ix[f"{s}_hip"]} for s in "lr"}
            fh = foot_floor_heights(rig.m)
            leg = float(np.mean([np.linalg.norm(rig.rest[ix[f"{s}_hip"]] - rig.rest[ix[f"{s}_knee"]]) + np.linalg.norm(rig.rest[ix[f"{s}_knee"]] - rig.rest[ix[f"{s}_ankle"]]) for s in "lr"]))
            T, planted, unreachable = plant_feet(T, feet, fh["ankle"], leg, sc["root"], toes_floor_z=fh["toes"])
            print(f"  planted frames L/R {planted.sum(0)}, unreachable {int(unreachable.sum())}")
        q = rig.solve(T)
        if out_dir:  # the solved qpos of the whole model, (frames, nq), for whoever drives the model with it
            os.makedirs(out_dir, exist_ok=True)
            np.save(os.path.join(out_dir, f"{name}.qpos.npy"), q)
            meta = {"clip": name, "frames": int(q.shape[0]), "nq": int(q.shape[1]), "fps": fps, "fps_note": "given by --fps (default 30), not read from the clip", "free_joint_names": list(FREE),
                    "free_qpos_index": [int(i) for i in rig.free], "frame": "model frame: x forward, y left, z up; first-frame heading removed; starts at the horizontal origin",
                    "site_error_mean_max_m": float(site_errors(rig.site_track(q), T).mean(0).max())}
            json.dump(meta, open(os.path.join(out_dir, f"{name}.qpos.json"), "w"), indent=1)
        err = site_errors(rig.site_track(q), T)
        mean, p99 = err.mean(0), np.percentile(err, 99, axis=0)
        gate1 = mean.max() <= TOL_MEAN and p99.max() <= TOL_P99
        w = SITE_NAMES[int(np.argmax(mean))], SITE_NAMES[int(np.argmax(p99))]
        print(f"[{name}] {clip.shape[0]} frames: site error mean max {mean.max():.4f} m ({w[0]}), p99 max {p99.max():.4f} m ({w[1]}), overall mean {err.mean():.4f} m")
        print(f"  GATE 1 site errors under tolerance: {'PASS' if gate1 else 'FAIL'}")
        if not gate1:
            fails.append(f"{name}: site errors")
        # gate 3: the colliding foot capsules stay above the floor; control = the same targets without the floor term
        depth = rig.floor_depth(q)
        g3 = depth.max() <= TOL_FLOOR and (depth > 0.001).mean() < TOL_FLOOR_FRAC
        print(f"  GATE 3 floor: deepest {depth.max() * 1000:.1f} mm, frames over 1 mm {(depth > 0.001).mean():.0%} -> {'PASS' if g3 else 'FAIL'}")
        if not g3:
            fails.append(f"{name}: foot floor penetration")
        if not skip_controls:
            rig.w_floor = 0.0
            d0 = rig.floor_depth(rig.solve(T_plain))
            rig.w_floor = W_FLOOR
            c3 = d0.max() <= TOL_FLOOR and (d0 > 0.001).mean() < TOL_FLOOR_FRAC
            print(f"  CONTROL 3 without the floor term: deepest {d0.max() * 1000:.1f} mm, frames over 1 mm {(d0 > 0.001).mean():.0%} -> gate {'PASS' if c3 else 'FAIL'} (must FAIL)")
            if c3:
                fails.append(f"{name}: control 3 did not fail")
        if not skip_controls:
            # control 1: the shins asked 30% wrong must fail the same gate
            sc_bad = dict(sc, l_ankle=sc["l_ankle"] * PLANT, r_ankle=sc["r_ankle"] * PLANT)  # the ankle segment is the shin (knee -> ankle)
            Tb = build_targets(S, sc_bad)
            eb = site_errors(rig.site_track(rig.solve(Tb)), Tb)
            c1 = eb.mean(0).max() <= TOL_MEAN and np.percentile(eb, 99, axis=0).max() <= TOL_P99
            print(f"  CONTROL 1 shin x{PLANT}: mean max {eb.mean(0).max():.4f} m, p99 max {np.percentile(eb, 99, axis=0).max():.4f} m -> gate {'PASS' if c1 else 'FAIL'} (must FAIL)")
            if c1:
                fails.append(f"{name}: control 1 did not fail")
        # gate 2: knee and elbow flexion inside the model's ranges
        rep = range_report(rig, q)
        g2, bad = flexion_gate(rep)
        peaks = ", ".join(f"{k} {np.degrees(v.min()):.0f}..{np.degrees(v.max()):.0f}" for k, (v, _) in rep.items())
        print(f"  flexion deg: {peaks}")
        print(f"  GATE 2 knee and elbow flexion inside ranges: {'PASS' if g2 else 'FAIL ' + str(bad)}")
        if not g2:
            fails.append(f"{name}: flexion range")
        if not skip_controls:
            # control 2: the same report with the right knee pushed past 0 and the left elbow past its top must fail
            rep_bad = {k: (v.copy(), r) for k, (v, r) in rep.items()}
            rep_bad["knee_angle_r"][0][0] = -0.3
            rep_bad["elbow_flexion_l"][0][-1] = rep["elbow_flexion_l"][1][1] + 0.3
            c2, _ = flexion_gate(rep_bad)
            print(f"  CONTROL 2 flexion pushed out of range: gate {'PASS' if c2 else 'FAIL'} (must FAIL)")
            if c2:
                fails.append(f"{name}: control 2 did not fail")
    print(("ALL GATES PASS (controls skipped)" if skip_controls else "ALL GATES AND CONTROLS AS REQUIRED") if not fails else "PROBLEMS: " + "; ".join(fails))
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
