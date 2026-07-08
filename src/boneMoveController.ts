import * as THREE from 'three';
import type { VRM, VRMHumanBoneName } from '@pixiv/three-vrm';
import type { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { BoneValidator } from './boneValidator';

// Body + limbs — fingers/toes excluded (joints sit too close for reliable picking).
const MOVE_BONES: string[] = [
  'hips', 'spine', 'chest', 'upperChest', 'neck', 'head',
  'leftShoulder', 'leftUpperArm', 'leftLowerArm', 'leftHand',
  'rightShoulder', 'rightUpperArm', 'rightLowerArm', 'rightHand',
  'leftUpperLeg', 'leftLowerLeg', 'leftFoot',
  'rightUpperLeg', 'rightLowerLeg', 'rightFoot',
];

// Which child joint we aim toward when a bone is grabbed. Rotating the grabbed
// bone points this child at the cursor. Leaves (hand/foot/head) have no child —
// handled by getTipWorld's fallbacks.
const CHILD_OF: Record<string, string> = {
  hips: 'spine', spine: 'chest', chest: 'upperChest', upperChest: 'neck', neck: 'head',
  leftShoulder: 'leftUpperArm', leftUpperArm: 'leftLowerArm', leftLowerArm: 'leftHand',
  rightShoulder: 'rightUpperArm', rightUpperArm: 'rightLowerArm', rightLowerArm: 'rightHand',
  leftUpperLeg: 'leftLowerLeg', leftLowerLeg: 'leftFoot',
  rightUpperLeg: 'rightLowerLeg', rightLowerLeg: 'rightFoot',
};

const MARKER_RADIUS = 0.024;
const COLOR_IDLE = 0x111111;   // black grab handles
const COLOR_HOVER = 0x4a4a4a;
const COLOR_ACTIVE = 0xff8b3d; // orange while dragging

interface Marker {
  bone: string;
  mesh: THREE.Mesh;
  node: THREE.Object3D; // normalized bone we rotate
  mat: THREE.MeshBasicMaterial;
}

/**
 * Drag-to-aim posing for VRM humanoid bones.
 *
 * Grab a joint marker and drag: the grabbed bone rotates so its child joint (its
 * "tip") follows the pointer on a camera-facing plane. The per-bone absolute
 * local quaternion is stored in `dragDeltas` and re-asserted every frame by
 * apply(), exactly like the gizmo controller — so the frame's ROM clamp runs on
 * top and you watch the joint stop dead at its anatomical limit.
 *
 * Public surface (apply / update / resetAll / getSelected / getVrm / setEnabled /
 * dispose) matches BoneDragController so main's render loop is unchanged.
 */
export class BoneMoveController {
  private vrm: VRM;
  private camera: THREE.Camera;
  private domElement: HTMLElement;
  private orbit: OrbitControls;
  private validator: BoneValidator;

  /** Hard lock: clamp the stored rotation at drag time so a bone can never even
   *  record an out-of-ROM angle (distinct from the render-time ROM clamp). */
  private hardLock = false;

  private markers: Marker[] = [];
  private dragDeltas = new Map<string, THREE.Quaternion>();

  private raycaster = new THREE.Raycaster();
  private pointerNDC = new THREE.Vector2();
  private plane = new THREE.Plane();
  private _enabled = false;

  private selectedBone: string | null = null;
  private hoverBone: string | null = null;
  private dragging = false;

  /** When on, log grab / drag / release + ROM checks to the console. */
  debug = false;
  private _lastLogT = 0;

  // Scratch
  private _O = new THREE.Vector3();
  private _T = new THREE.Vector3();
  private _P = new THREE.Vector3();
  private _camDir = new THREE.Vector3();
  private _curDir = new THREE.Vector3();
  private _wantDir = new THREE.Vector3();
  private _parentPos = new THREE.Vector3();
  private _qWorldDelta = new THREE.Quaternion();
  private _qWorld = new THREE.Quaternion();
  private _qParent = new THREE.Quaternion();
  private _qPreLock = new THREE.Quaternion();
  private _euler = new THREE.Euler();

  private _onPointerDown = (e: PointerEvent): void => this.handlePointerDown(e);
  private _onPointerMove = (e: PointerEvent): void => this.handlePointerMove(e);
  private _onPointerUp = (e: PointerEvent): void => this.handlePointerUp(e);

  constructor(
    vrm: VRM,
    camera: THREE.Camera,
    domElement: HTMLElement,
    orbit: OrbitControls,
    validator: BoneValidator,
  ) {
    this.vrm = vrm;
    this.camera = camera;
    this.domElement = domElement;
    this.orbit = orbit;
    this.validator = validator;

    for (const name of MOVE_BONES) {
      const rawNode = vrm.humanoid.getRawBoneNode(name as VRMHumanBoneName);
      const normNode = vrm.humanoid.getNormalizedBoneNode(name as VRMHumanBoneName);
      if (!rawNode || !normNode) continue;
      const geo = new THREE.SphereGeometry(MARKER_RADIUS, 14, 10);
      const mat = new THREE.MeshBasicMaterial({
        color: COLOR_IDLE,
        depthTest: false,
        transparent: true,
        opacity: 0.9,
      });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.renderOrder = 999; // draw over the mesh so handles stay visible
      mesh.userData.boneName = name;
      mesh.visible = false;
      rawNode.add(mesh); // rides the visible rig
      this.markers.push({ bone: name, mesh, node: normNode, mat });
    }

    domElement.addEventListener('pointerdown', this._onPointerDown);
    window.addEventListener('pointermove', this._onPointerMove);
    window.addEventListener('pointerup', this._onPointerUp);
  }

  // ── Public API (mirrors BoneDragController) ──────────────────────────────────

  get enabled(): boolean { return this._enabled; }
  getVrm(): VRM { return this.vrm; }
  getSelected(): string | null { return this.selectedBone; }

  /** List of draggable bones (for building the joint panel). */
  getBones(): string[] { return this.markers.map((m) => m.bone); }

  /** The stored local rotation for a bone, or null if it is at rest. */
  getStoredQuat(bone: string): THREE.Quaternion | null {
    return this.dragDeltas.get(bone) ?? null;
  }

  /** Current stored rotation as Euler degrees in `order`. [0,0,0] at rest. */
  getStoredEulerDeg(bone: string, order: THREE.EulerOrder): [number, number, number] {
    const q = this.dragDeltas.get(bone);
    if (!q) return [0, 0, 0];
    this._euler.setFromQuaternion(q, order);
    const r = THREE.MathUtils.radToDeg;
    return [r(this._euler.x), r(this._euler.y), r(this._euler.z)];
  }

  /** Set a bone's pose from Euler degrees (panel sliders). Hard-lock clamps it. */
  setBoneEuler(bone: string, deg: [number, number, number], order: THREE.EulerOrder): void {
    const d = THREE.MathUtils.degToRad;
    this._euler.set(d(deg[0]), d(deg[1]), d(deg[2]), order);
    this._qParent.setFromEuler(this._euler);
    if (this.hardLock) this.validator.clampQuaternion(bone as VRMHumanBoneName, this._qParent);
    const slot = this.dragDeltas.get(bone);
    if (slot) slot.copy(this._qParent);
    else this.dragDeltas.set(bone, this._qParent.clone());
    this.selectedBone = bone;
  }

  /** When on, dragging can never push a bone past its ROM (clamp at the source).
   *  Also legalises any already-stored poses so nothing illegal survives. */
  setHardLock(v: boolean): void {
    this.hardLock = v;
    if (v) {
      for (const [name, q] of this.dragDeltas) {
        this.validator.clampQuaternion(name as VRMHumanBoneName, q);
      }
    }
  }

  setEnabled(v: boolean): void {
    this._enabled = v;
    for (const m of this.markers) m.mesh.visible = v;
    if (!v) {
      this.dragging = false;
      this.selectedBone = null;
      this.orbit.enabled = true;
    }
  }

  resetAll(): void {
    for (const name of this.dragDeltas.keys()) {
      const node = this.vrm.humanoid.getNormalizedBoneNode(name as VRMHumanBoneName);
      node?.quaternion.identity();
    }
    this.dragDeltas.clear();
    this.selectedBone = null;
    this.dragging = false;
    this.orbit.enabled = true;
    this.refreshMarkerColors();
  }

  /** Re-assert each stored absolute local rotation. Rig was reset to rest first. */
  apply(): void {
    if (this.dragDeltas.size === 0) return;
    for (const [name, q] of this.dragDeltas) {
      const node = this.vrm.humanoid.getNormalizedBoneNode(name as VRMHumanBoneName);
      if (node) node.quaternion.multiply(q); // rest(identity) * q = q
    }
  }

  /** Nothing to reposition — markers are children of raw bones. Kept for parity. */
  update(): void { /* markers ride the rig automatically */ }

  dispose(): void {
    this.domElement.removeEventListener('pointerdown', this._onPointerDown);
    window.removeEventListener('pointermove', this._onPointerMove);
    window.removeEventListener('pointerup', this._onPointerUp);
    for (const m of this.markers) {
      m.mesh.parent?.remove(m.mesh);
      m.mesh.geometry.dispose();
      m.mat.dispose();
    }
    this.markers = [];
    this.dragDeltas.clear();
  }

  // ── Pointer handling ─────────────────────────────────────────────────────────

  /** Point the raycaster at the pointer's screen position. */
  private castRayFromPointer(ev: PointerEvent): void {
    const rect = this.domElement.getBoundingClientRect();
    this.pointerNDC.x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointerNDC.y = -((ev.clientY - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(this.pointerNDC, this.camera as THREE.PerspectiveCamera);
  }

  private pickBone(ev: PointerEvent): string | null {
    this.castRayFromPointer(ev);
    const hits = this.raycaster.intersectObjects(this.markers.map((m) => m.mesh), false);
    return hits.length ? (hits[0].object.userData.boneName as string) : null;
  }

  private handlePointerDown(ev: PointerEvent): void {
    if (!this._enabled || ev.button !== 0) return;
    const bone = this.pickBone(ev);
    if (!bone) {
      if (this.debug) console.log('%c[grab] miss — no marker under cursor', 'color:#888');
      return;
    }
    if (this.debug) {
      console.log(
        `%c[grab] ${bone}`,
        'color:#2ecc71;font-weight:bold',
        { hardLock: this.hardLock, romClamp: this.validator.enabled },
      );
    }
    ev.stopPropagation();
    ev.preventDefault();

    this.selectedBone = bone;
    this.dragging = true;
    this.orbit.enabled = false;

    // Freeze a camera-facing drag plane through the current tip position.
    this.getTipWorld(bone, this._T);
    this.camera.getWorldDirection(this._camDir);
    this.plane.setFromNormalAndCoplanarPoint(this._camDir, this._T);

    this.refreshMarkerColors();
  }

  private handlePointerMove(ev: PointerEvent): void {
    if (this._enabled && !this.dragging) {
      const hover = this.pickBone(ev);
      if (hover !== this.hoverBone) {
        this.hoverBone = hover;
        this.domElement.style.cursor = hover ? 'grab' : '';
        this.refreshMarkerColors();
      }
      return;
    }
    if (!this.dragging || !this.selectedBone) return;
    const bone = this.selectedBone;

    // Pointer → drag plane → world target the tip should reach.
    this.castRayFromPointer(ev);
    if (!this.raycaster.ray.intersectPlane(this.plane, this._P)) return;

    const node = this.vrm.humanoid.getNormalizedBoneNode(bone as VRMHumanBoneName);
    if (!node || !node.parent) return;

    node.getWorldPosition(this._O);
    this.getTipWorld(bone, this._T);
    this._curDir.subVectors(this._T, this._O);
    this._wantDir.subVectors(this._P, this._O);
    if (this._curDir.lengthSq() < 1e-8 || this._wantDir.lengthSq() < 1e-8) return;
    this._curDir.normalize();
    this._wantDir.normalize();

    // World-space rotation that swings the tip from where it is to the cursor.
    this._qWorldDelta.setFromUnitVectors(this._curDir, this._wantDir);
    node.getWorldQuaternion(this._qWorld);
    this._qWorld.premultiply(this._qWorldDelta);      // newWorld = delta * curWorld
    node.parent.getWorldQuaternion(this._qParent);
    this._qParent.invert().multiply(this._qWorld);    // local = parentWorld^-1 * newWorld

    const preClamp = this.debug ? this.eulerDeg(bone, this._qParent) : null;
    const swingDeg = this.debug
      ? Math.round(THREE.MathUtils.radToDeg(this._curDir.angleTo(this._wantDir)))
      : 0;

    // Hard lock: clamp the rotation to ROM here, at the source, so the bone
    // physically stops at its limit and never even records an illegal angle.
    let hardLockCorrectionDeg = 0;
    if (this.hardLock) {
      this._qPreLock.copy(this._qParent);
      this.validator.clampQuaternion(bone as VRMHumanBoneName, this._qParent);
      const dot = Math.min(1, Math.abs(this._qPreLock.dot(this._qParent)));
      hardLockCorrectionDeg = Math.round(THREE.MathUtils.radToDeg(2 * Math.acos(dot)));

      // Always-on safety net: if hard lock is on but the stored rotation still
      // ends outside ROM, hard lock genuinely failed — surface it regardless of
      // the debug toggle.
      const residual = this.validator.dryRunOvershoot(bone as VRMHumanBoneName, this._qParent);
      if (residual > 2e-2) {
        console.warn(
          `[hardlock] ${bone} STILL OUT OF ROM after clamp`,
          { residualDeg: Math.round(THREE.MathUtils.radToDeg(residual)), storedDeg: this.eulerDeg(bone, this._qParent) },
        );
      }
    }

    if (this.debug) {
      const now = performance.now();
      if (now - this._lastLogT > 120) { // throttle: ~8 logs/sec
        this._lastLogT = now;
        const rom = this.romCheck(bone, this._qParent);
        console.log(
          `%c[drag] ${bone}`,
          rom.within ? 'color:#7aa2ff' : 'color:#ff8b3d;font-weight:bold',
          {
            swingToCursorDeg: swingDeg,
            preClampDeg: preClamp,
            storedDeg: rom.deg,
            order: rom.order,
            within: rom.within,
            hardLock: this.hardLock,
            hardLockCorrectionDeg,
            ...(rom.within ? {} : { overshootDeg: rom.overshootDeg }),
          },
        );
      }
    }

    // Store as this bone's absolute local rotation (rest = identity, so this IS
    // the delta apply() re-asserts each frame before the ROM clamp runs).
    const slot = this.dragDeltas.get(bone);
    if (slot) slot.copy(this._qParent);
    else this.dragDeltas.set(bone, this._qParent.clone());
  }

  private handlePointerUp(_ev: PointerEvent): void {
    if (!this.dragging) return;
    const bone = this.selectedBone;
    this.dragging = false;
    this.orbit.enabled = true;
    this.refreshMarkerColors();

    if (this.debug && bone) {
      const q = this.dragDeltas.get(bone);
      if (q) {
        const rom = this.romCheck(bone, q);
        // Only hard lock clamps the STORED rotation. The ROM toggle clamps the
        // live node at render time and deliberately keeps the raw drag intact,
        // so a stored overshoot with only ROM on is expected, not a bug.
        if (this.hardLock && !rom.within) {
          console.warn(
            `[release] ${bone} STORED OUT OF ROM despite hard lock (likely gimbal flip)`,
            { storedDeg: rom.deg, order: rom.order, overshootDeg: rom.overshootDeg },
          );
        } else if (!this.hardLock && this.validator.enabled && !rom.within) {
          console.log(
            `%c[release] ${bone} — raw stored angle out of ROM (render-clamped on screen; hard lock off)`,
            'color:#c9a227',
            { storedDeg: rom.deg, order: rom.order, overshootDeg: rom.overshootDeg },
          );
        } else {
          console.log(
            `%c[release] ${bone}`, 'color:#2ecc71',
            { storedDeg: rom.deg, within: rom.within, order: rom.order },
          );
        }
      }
    }
  }

  // ── Helpers ──────────────────────────────────────────────────────────────────

  /** Local Euler (deg) of a quaternion in the bone's constraint order. */
  private eulerDeg(bone: string, q: THREE.Quaternion): [number, number, number] {
    const c = this.validator.getConstraints()[bone as VRMHumanBoneName];
    const e = new THREE.Euler().setFromQuaternion(q, c?.order ?? 'XYZ');
    const r = THREE.MathUtils.radToDeg;
    return [Math.round(r(e.x)), Math.round(r(e.y)), Math.round(r(e.z))];
  }

  /** Report whether `q` is inside the bone's ROM, via the validator's own
   *  swing-twist clamp (dry run) so the check matches what the clamp enforces. */
  private romCheck(bone: string, q: THREE.Quaternion): {
    within: boolean;
    order: string;
    deg: [number, number, number];
    overshootDeg: number;
  } {
    const c = this.validator.getConstraints()[bone as VRMHumanBoneName];
    const overshoot = this.validator.dryRunOvershoot(bone as VRMHumanBoneName, q);
    return {
      within: overshoot < 1e-4,
      order: c?.order ?? 'none',
      deg: this.eulerDeg(bone, q),
      overshootDeg: Math.round(THREE.MathUtils.radToDeg(overshoot)),
    };
  }

  /** World position of the joint we aim at when `bone` is grabbed. */
  private getTipWorld(bone: string, out: THREE.Vector3): void {
    const childName = CHILD_OF[bone];
    if (childName) {
      const child = this.vrm.humanoid.getNormalizedBoneNode(childName as VRMHumanBoneName);
      if (child) { child.getWorldPosition(out); return; }
    }
    // Leaf (hand/foot/head): aim a synthetic tip continuing the bone outward.
    const node = this.vrm.humanoid.getNormalizedBoneNode(bone as VRMHumanBoneName);
    if (!node) { out.set(0, 0, 0); return; }
    node.getWorldPosition(out);
    if (node.parent) {
      node.parent.getWorldPosition(this._parentPos);
      this._curDir.subVectors(out, this._parentPos);
      if (this._curDir.lengthSq() > 1e-8) {
        out.addScaledVector(this._curDir.normalize(), 0.1);
      }
    }
  }

  private refreshMarkerColors(): void {
    for (const m of this.markers) {
      const c = m.bone === this.selectedBone && this.dragging
        ? COLOR_ACTIVE
        : m.bone === this.hoverBone
          ? COLOR_HOVER
          : COLOR_IDLE;
      m.mat.color.setHex(c);
    }
  }
}
