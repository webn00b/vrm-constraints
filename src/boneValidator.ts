/**
 * Runtime bone-rotation validator.
 *
 * Once per frame, after the drag controller has written the pose, `clampAll`
 * clamps every constrained bone back into its anatomical range. Two clamp modes:
 *   - box   ('default' / 'mixamoLive') — per-axis Euler clamp with a gimbal-safe
 *     geodesic fallback (see `clampQuaternion`).
 *   - cone  ('strict') — coupled swing-twist cone (see `clampCone`).
 */

import * as THREE from 'three';
import type { VRM } from '@pixiv/three-vrm';
import { VRMHumanBoneName } from '@pixiv/three-vrm';
import {
  type BoneConstraintProfileId,
  mergeConstraints,
  type RotationConstraint,
} from './boneConstraints';

export interface ValidationStats {
  /** Bones clamped during the most recent frame. */
  clampedThisFrame: number;
  /** Bone with the largest overshoot this frame, null if none. */
  worstBone: VRMHumanBoneName | null;
}

// Reusable scratch — the validator runs every frame on every bone, so we avoid
// allocating Euler / Quaternion / Vector objects per call.
const _euler = new THREE.Euler();
const _origQuat = new THREE.Quaternion();
const _idQuat = new THREE.Quaternion();
const _slerpQuat = new THREE.Quaternion();
const _dryQuat = new THREE.Quaternion();
const _twistAxisVec = new THREE.Vector3();
const _qTwist = new THREE.Quaternion();
const _qSwing = new THREE.Quaternion();
const _swingVec = new THREE.Vector3();

// Local axis a bone extends along in the normalized rest frame = its twist axis
// (0=X, 1=Y, 2=Z). Arms/hands/fingers run along X, spine/legs along Y, feet/
// eyes/jaw along Z. The other two axes span the swing plane.
function twistAxisIndex(bone: string): 0 | 1 | 2 {
  if (/Arm|Hand|Shoulder|Thumb|Index|Middle|Ring|Little/.test(bone)) return 0;
  if (/Foot|Toes|Eye|Jaw/.test(bone)) return 2;
  return 1;
}
const SWING_AXES: Record<0 | 1 | 2, [number, number]> = {
  0: [1, 2],
  1: [0, 2],
  2: [0, 1],
};

/** Swing-twist extents of an Euler box, in the local rotation-vector frame. */
interface ConeExtent {
  ti: 0 | 1 | 2;          // twist axis index
  s1: number; s2: number; // swing axis indices
  aMin: number; aMax: number; // swing extent along s1 (rad)
  bMin: number; bMax: number; // swing extent along s2 (rad)
  twMin: number; twMax: number; // twist extent (rad)
}

/** Decompose q about twist axis `ti`; return twist angle + swing components. */
function swingTwist(
  q: THREE.Quaternion,
  ti: 0 | 1 | 2,
  s1: number,
  s2: number,
): { twist: number; a: number; b: number; swingAngle: number } {
  const comp = ti === 0 ? q.x : ti === 1 ? q.y : q.z;
  const w = q.w;
  const tl = Math.hypot(comp, w);
  let twist = 0;
  if (tl < 1e-8) {
    _qTwist.identity();
  } else {
    const nd = comp / tl, nw = w / tl;
    _qTwist.set(ti === 0 ? nd : 0, ti === 1 ? nd : 0, ti === 2 ? nd : 0, nw);
    twist = 2 * Math.atan2(nd, nw);
  }
  // swing = q * qTwist⁻¹
  _qSwing.copy(_qTwist).conjugate().premultiply(q);
  if (_qSwing.w < 0) _qSwing.set(-_qSwing.x, -_qSwing.y, -_qSwing.z, -_qSwing.w);
  const swingAngle = 2 * Math.acos(Math.min(1, _qSwing.w));
  let a = 0, b = 0;
  if (swingAngle > 1e-6) {
    const k = swingAngle / Math.sin(swingAngle / 2);
    const sv = [_qSwing.x * k, _qSwing.y * k, _qSwing.z * k];
    a = sv[s1];
    b = sv[s2];
  }
  return { twist, a, b, swingAngle };
}

/**
 * Cone extents straight from the bone's Euler box: the twist axis bounds twist,
 * the other two bound the swing plane. Using the axis ranges directly (rather
 * than sampling the box volume) keeps each extent to its true single-axis limit —
 * volume sampling pulled in impossible corner combinations and let the twist
 * angle wrap past ±180°. The ellipse + twist coupling in `clampCone` add the
 * cross-axis limiting on top.
 */
function deriveConeExtent(c: RotationConstraint, ti: 0 | 1 | 2): ConeExtent {
  const [s1, s2] = SWING_AXES[ti];
  return {
    ti, s1, s2,
    aMin: c.min[s1], aMax: c.max[s1],
    bMin: c.min[s2], bMax: c.max[s2],
    twMin: c.min[ti], twMax: c.max[ti],
  };
}

export class BoneValidator {
  private vrm: VRM;
  private constraints: Partial<Record<VRMHumanBoneName, RotationConstraint>>;
  private overrides?: Partial<Record<VRMHumanBoneName, RotationConstraint>>;
  private nodeCache = new Map<VRMHumanBoneName, THREE.Object3D>();
  private coneCache = new Map<VRMHumanBoneName, ConeExtent>();
  private stats: ValidationStats = { clampedThisFrame: 0, worstBone: null };

  /** 'strict' profile clamps with a coupled swing-twist cone instead of a box. */
  private get coneMode(): boolean { return this.profileId === 'strict'; }

  // Disabled by default — ROM clamping is opt-in via the panel toggle.
  enabled = false;
  profileId: BoneConstraintProfileId = 'default';

  constructor(vrm: VRM, overrides?: Partial<Record<VRMHumanBoneName, RotationConstraint>>) {
    this.vrm = vrm;
    this.overrides = overrides;
    this.constraints = mergeConstraints(overrides, this.profileId);
    this.rebuildCache();
  }

  private rebuildCache(): void {
    this.nodeCache.clear();
    this.coneCache.clear();
    const humanoid = this.vrm.humanoid;
    for (const name of Object.keys(this.constraints) as VRMHumanBoneName[]) {
      const node = humanoid.getNormalizedBoneNode(name);
      if (node) this.nodeCache.set(name, node);
      const c = this.constraints[name];
      if (c) this.coneCache.set(name, deriveConeExtent(c, twistAxisIndex(name)));
    }
  }

  /** True if `q` decomposes (in the bone's Euler order) inside every axis bound. */
  private eulerInBox(c: RotationConstraint, q: THREE.Quaternion): boolean {
    _euler.setFromQuaternion(q, c.order);
    const a = [_euler.x, _euler.y, _euler.z];
    const eps = 1e-3;
    for (let i = 0; i < 3; i++) {
      if (a[i] < c.min[i] - eps || a[i] > c.max[i] + eps) return false;
    }
    return true;
  }

  /**
   * Clamp a quaternion in place to a bone's anatomical ROM (the per-axis Euler
   * box), robust against the gimbal branch flip.
   *
   * Pass 1 — per-axis Euler clamp: quaternion → Euler (bone's order) → clamp each
   * axis → back. Exact and cheap. But quaternion→Euler forces the middle axis
   * into [-90°,90°], so a bone bent past 90° there decomposes on the antipodal
   * branch and this pass snaps it to a wrong (yet numerically in-range-looking)
   * orientation.
   *
   * Pass 2 — flip guard: re-decompose the result; if it is NOT actually inside
   * the box, the clamp flipped branches. Fall back to a geodesic pullback: slerp
   * from rest (identity) toward the original orientation and binary-search the
   * furthest point that still satisfies the box. This never flips (it stays on
   * the shortest arc) and always lands inside ROM, so the bone stops cleanly at
   * its limit from any drag direction instead of snapping to garbage.
   *
   * ('strict' profile routes to `clampCone` instead — see below.)
   *
   * Returns the largest per-axis Euler overshoot (radians), 0 if already valid.
   */
  clampQuaternion(bone: VRMHumanBoneName, q: THREE.Quaternion): number {
    const c = this.constraints[bone];
    if (!c) return 0;

    if (this.coneMode) {
      const cone = this.coneCache.get(bone);
      if (cone) return this.clampCone(cone, q);
    }

    const ox = q.x, oy = q.y, oz = q.z, ow = q.w;

    // ── Pass 1: per-axis Euler clamp ───────────────────────────────────────────
    _euler.setFromQuaternion(q, c.order);
    let overshoot = 0;
    const clampAxis = (v: number, lo: number, hi: number): number => {
      if (v < lo) { const d = lo - v; if (d > overshoot) overshoot = d; return lo; }
      if (v > hi) { const d = v - hi; if (d > overshoot) overshoot = d; return hi; }
      return v;
    };
    const cx = clampAxis(_euler.x, c.min[0], c.max[0]);
    const cy = clampAxis(_euler.y, c.min[1], c.max[1]);
    const cz = clampAxis(_euler.z, c.min[2], c.max[2]);

    if (overshoot === 0) return 0; // already valid — leave q untouched

    _euler.set(cx, cy, cz, c.order);
    q.setFromEuler(_euler);
    // Preserve hemisphere (setFromEuler returns a canonical, possibly antipodal
    // form) so downstream frame-diffs don't read a phantom 180° flip.
    if (q.x * ox + q.y * oy + q.z * oz + q.w * ow < 0) {
      q.set(-q.x, -q.y, -q.z, -q.w);
    }

    // ── Pass 2: flip guard via geodesic pullback ───────────────────────────────
    if (!this.eulerInBox(c, q)) {
      _origQuat.set(ox, oy, oz, ow);
      _idQuat.identity();
      let lo = 0, hi = 1;
      for (let i = 0; i < 20; i++) {
        const mid = (lo + hi) / 2;
        _slerpQuat.copy(_idQuat).slerp(_origQuat, mid);
        if (this.eulerInBox(c, _slerpQuat)) lo = mid; else hi = mid;
      }
      q.copy(_idQuat).slerp(_origQuat, lo);
    }
    return overshoot;
  }

  /**
   * Coupled swing-twist cone clamp (the 'strict' profile).
   *
   * The per-axis Euler box lets a joint sit at max on every axis at once, which
   * produces anatomically impossible corner poses (hip fully flexed AND fully
   * abducted AND fully twisted → a pretzel). This instead:
   *   - splits the rotation into twist (about the bone axis) + swing (where the
   *     bone points), so there is no Euler gimbal fold;
   *   - clamps the swing to an *ellipse* over the two swing axes — the axes are
   *     coupled, so you can max one direction OR blend, never both corners;
   *   - clamps twist to a range that *shrinks* as the swing nears its limit
   *     (real joints lose axial rotation at the end of their arc).
   * Extents come from the same anatomical box (deriveConeExtent), so single-axis
   * motion still reaches its true range.
   */
  private clampCone(cone: ConeExtent, q: THREE.Quaternion): number {
    const { ti, s1, s2 } = cone;
    const st = swingTwist(q, ti, s1, s2);
    let a = st.a, b = st.b;
    const twist = st.twist;

    // Asymmetric elliptical swing clamp (extent chosen per side).
    const ea = Math.max(1e-3, a >= 0 ? cone.aMax : -cone.aMin);
    const eb = Math.max(1e-3, b >= 0 ? cone.bMax : -cone.bMin);
    let overshoot = 0;
    const rr = (a / ea) * (a / ea) + (b / eb) * (b / eb);
    let swFrac = Math.min(1, Math.sqrt(rr));
    if (rr > 1) {
      const s = 1 / Math.sqrt(rr);
      overshoot = st.swingAngle * (1 - s);
      a *= s; b *= s;
      swFrac = 1;
    }

    // Twist range shrinks toward the swing limit (couple twist to swing).
    const shrink = 1 - 0.5 * swFrac; // full twist at rest, half at the cone edge
    const twLo = cone.twMin * shrink;
    const twHi = cone.twMax * shrink;
    let twC = twist;
    if (twC < twLo) { const d = twLo - twC; if (d > overshoot) overshoot = d; twC = twLo; }
    else if (twC > twHi) { const d = twC - twHi; if (d > overshoot) overshoot = d; twC = twHi; }

    if (overshoot === 0) return 0; // already inside the cone — leave q untouched

    // Recombine q = swing' · twist'.
    _twistAxisVec.set(ti === 0 ? 1 : 0, ti === 1 ? 1 : 0, ti === 2 ? 1 : 0);
    _qTwist.setFromAxisAngle(_twistAxisVec, twC);
    const arr = [0, 0, 0];
    arr[s1] = a; arr[s2] = b;
    _swingVec.set(arr[0], arr[1], arr[2]);
    const ang = _swingVec.length();
    if (ang < 1e-8) _qSwing.identity();
    else _qSwing.setFromAxisAngle(_swingVec.multiplyScalar(1 / ang), ang);
    q.copy(_qSwing).multiply(_qTwist);
    return overshoot;
  }

  /** Non-mutating: how far (radians) `q` is outside `bone`'s ROM. 0 = valid. */
  dryRunOvershoot(bone: VRMHumanBoneName, q: THREE.Quaternion): number {
    _dryQuat.copy(q);
    return this.clampQuaternion(bone, _dryQuat);
  }

  /** Apply clampQuaternion to every known bone. Called once per frame. */
  clampAll(): ValidationStats {
    if (!this.enabled) {
      this.stats.clampedThisFrame = 0;
      this.stats.worstBone = null;
      return this.stats;
    }

    let clamped = 0;
    let worstBone: VRMHumanBoneName | null = null;
    let worstDelta = 0;
    for (const [bone, node] of this.nodeCache) {
      const overshoot = this.clampQuaternion(bone, node.quaternion);
      if (overshoot > 0) {
        clamped++;
        if (overshoot > worstDelta) { worstDelta = overshoot; worstBone = bone; }
      }
    }
    this.stats.clampedThisFrame = clamped;
    this.stats.worstBone = worstBone;
    return this.stats;
  }

  setEnabled(on: boolean): void {
    this.enabled = on;
  }

  setProfile(profileId: BoneConstraintProfileId): void {
    if (this.profileId === profileId) return;
    this.profileId = profileId;
    this.constraints = mergeConstraints(this.overrides, this.profileId);
    this.rebuildCache();
    this.stats.clampedThisFrame = 0;
    this.stats.worstBone = null;
  }

  /** The active per-bone constraints (used by the debug logs). */
  getConstraints(): Partial<Record<VRMHumanBoneName, RotationConstraint>> {
    return this.constraints;
  }

  /** Debug: this bone's cone extents in degrees (strict mode only), else null. */
  getConeExtentDeg(bone: VRMHumanBoneName): {
    axes: string;
    swingA: [number, number];
    swingB: [number, number];
    twist: [number, number];
  } | null {
    if (!this.coneMode) return null;
    const e = this.coneCache.get(bone);
    if (!e) return null;
    const r = THREE.MathUtils.radToDeg;
    const round = (x: number): number => Math.round(r(x));
    return {
      axes: `twist=${'XYZ'[e.ti]} swing=${'XYZ'[e.s1]},${'XYZ'[e.s2]}`,
      swingA: [round(e.aMin), round(e.aMax)],
      swingB: [round(e.bMin), round(e.bMax)],
      twist: [round(e.twMin), round(e.twMax)],
    };
  }
}
