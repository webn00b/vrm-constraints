import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { VRMHumanBoneName } from '@pixiv/three-vrm';
import { loadVRM } from './vrmLoader';
import { BoneMoveController } from './boneMoveController';
import { BoneValidator } from './boneValidator';
import type { BoneConstraintProfileId, RotationConstraint } from './boneConstraints';

// BASE_URL is '/' in dev and '/vrm-constraints/' in the Pages build.
const MODEL_URL = `${import.meta.env.BASE_URL}models/sample.vrm`;

// The shared constraint set leaves Hips unconstrained (it carries global root
// orientation for animation playback). This app is a poser, so an unconstrained
// Hips means grabbing the pelvis marker rotates the whole avatar freely — hard
// lock appears broken. Give it a sane pelvic-tilt ROM instead.
const d = THREE.MathUtils.degToRad;
const OVERRIDES: Partial<Record<VRMHumanBoneName, RotationConstraint>> = {
  [VRMHumanBoneName.Hips]: {
    order: 'YXZ',
    min: [d(-30), d(-45), d(-25)], // pitch / yaw / roll
    max: [d(30), d(45), d(25)],
  },
};

// ── Renderer / scene / camera ────────────────────────────────────────────────
const canvas = document.getElementById('scene') as HTMLCanvasElement;
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.outputColorSpace = THREE.SRGBColorSpace;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1d24);

const camera = new THREE.PerspectiveCamera(
  35,
  window.innerWidth / window.innerHeight,
  0.1,
  100,
);
camera.position.set(0, 1.25, 2.4);

const orbit = new OrbitControls(camera, renderer.domElement);
orbit.target.set(0, 1.0, 0);
orbit.enableDamping = true;
orbit.update();

// ── Lights + ground ──────────────────────────────────────────────────────────
scene.add(new THREE.HemisphereLight(0xffffff, 0x333844, 1.4));
const dir = new THREE.DirectionalLight(0xffffff, 1.6);
dir.position.set(1.5, 3, 2);
scene.add(dir);

const grid = new THREE.GridHelper(10, 20, 0x3a3f4b, 0x2a2e37);
scene.add(grid);

// ── App state ────────────────────────────────────────────────────────────────
let dragController: BoneMoveController | null = null;
let validator: BoneValidator | null = null;
let skeletonHelper: THREE.SkeletonHelper | null = null;
let loadedVrm: import('@pixiv/three-vrm').VRM | null = null;
const clock = new THREE.Clock();

const statusEl = document.getElementById('status') as HTMLDivElement;
const toggleEl = document.getElementById('constraint-toggle') as HTMLInputElement;
const hardlockEl = document.getElementById('hardlock-toggle') as HTMLInputElement;
const debugEl = document.getElementById('debug-toggle') as HTMLInputElement;
const meshEl = document.getElementById('mesh-toggle') as HTMLInputElement;
const profileEl = document.getElementById('profile') as HTMLSelectElement;
const resetEl = document.getElementById('reset') as HTMLButtonElement;
const loadingEl = document.getElementById('loading') as HTMLDivElement;

// ── Boot ─────────────────────────────────────────────────────────────────────
loadVRM(MODEL_URL)
  .then((vrm) => {
    scene.add(vrm.scene);
    loadedVrm = vrm;

    // Skeleton overlay — rides the posed raw bones.
    skeletonHelper = new THREE.SkeletonHelper(vrm.scene);
    (skeletonHelper.material as THREE.LineBasicMaterial).linewidth = 2;
    scene.add(skeletonHelper);
    applyViewMode(meshEl.checked); // default: mesh off → skeleton visible

    validator = new BoneValidator(vrm, OVERRIDES);
    validator.setEnabled(toggleEl.checked);
    validator.setProfile(profileEl.value as BoneConstraintProfileId);

    dragController = new BoneMoveController(
      vrm,
      camera,
      renderer.domElement,
      orbit,
      validator,
    );
    dragController.setEnabled(true);
    dragController.setHardLock(hardlockEl.checked);
    dragController.debug = debugEl.checked;

    // Debug handle — inspect / script the rig from the console.
    (window as unknown as Record<string, unknown>).__viewer = {
      vrm,
      dragController,
      validator,
    };

    buildJointPanel();
    loadingEl.classList.add('hidden');
  })
  .catch((err) => {
    console.error(err);
    loadingEl.textContent = `Failed to load model: ${err.message ?? err}`;
  });

// ── UI wiring ────────────────────────────────────────────────────────────────
toggleEl.addEventListener('change', () => {
  validator?.setEnabled(toggleEl.checked);
});

hardlockEl.addEventListener('change', () => {
  dragController?.setHardLock(hardlockEl.checked);
});

debugEl.addEventListener('change', () => {
  if (dragController) dragController.debug = debugEl.checked;
  if (debugEl.checked) console.log('%c[debug] logging ON — grab a marker and drag', 'color:#2ecc71;font-weight:bold');
});

meshEl.addEventListener('change', () => applyViewMode(meshEl.checked));

/** showMesh=true → VRM mesh; false → skeleton only. Grab markers stay visible. */
function applyViewMode(showMesh: boolean): void {
  if (skeletonHelper) skeletonHelper.visible = !showMesh;
  if (!loadedVrm) return;
  loadedVrm.scene.traverse((obj) => {
    // Avatar meshes only — leave bones and the drag markers (tagged with
    // userData.boneName) alone so you can still grab joints in skeleton view.
    const mesh = obj as THREE.Mesh;
    if (mesh.isMesh && !mesh.userData.boneName) mesh.visible = showMesh;
  });
}

profileEl.addEventListener('change', () => {
  validator?.setProfile(profileEl.value as BoneConstraintProfileId);
  buildJointPanel(); // ROM bounds differ per profile → rebuild sliders
});

resetEl.addEventListener('click', () => {
  dragController?.resetAll();
});

// ── Joint panel (per-joint sliders + requested→actual diagnostics) ───────────
interface JointRow {
  bone: string;
  order: THREE.EulerOrder;
  nameEl: HTMLElement;
  badgeEl: HTMLElement;
  sliders: HTMLInputElement[];
  valEls: HTMLElement[];
}
const jointRows: JointRow[] = [];
const _jEuler = new THREE.Euler();
const _jQuat = new THREE.Quaternion();
const R2D = THREE.MathUtils.radToDeg;

/** Euler (deg) the bone actually renders at = stored pose after the live clamp. */
function renderedEulerDeg(bone: string, order: THREE.EulerOrder): [number, number, number] {
  const q = dragController?.getStoredQuat(bone);
  if (!q) return [0, 0, 0];
  _jQuat.copy(q);
  if (validator?.enabled) validator.clampQuaternion(bone as VRMHumanBoneName, _jQuat);
  _jEuler.setFromQuaternion(_jQuat, order);
  return [R2D(_jEuler.x), R2D(_jEuler.y), R2D(_jEuler.z)];
}

function buildJointPanel(): void {
  if (!dragController || !validator) return;
  const list = document.getElementById('joints-list') as HTMLDivElement;
  list.innerHTML = '';
  jointRows.length = 0;
  const cons = validator.getConstraints();

  for (const bone of dragController.getBones()) {
    const c = cons[bone as VRMHumanBoneName];
    if (!c) continue;
    const order = c.order as THREE.EulerOrder;

    const row = document.createElement('div');
    row.className = 'joint';
    const head = document.createElement('div');
    head.className = 'joint-head';
    const nameEl = document.createElement('span');
    nameEl.className = 'jname';
    nameEl.textContent = bone;
    const badgeEl = document.createElement('span');
    badgeEl.className = 'jbadge';
    head.append(nameEl, badgeEl);
    row.append(head);

    const sliders: HTMLInputElement[] = [];
    const valEls: HTMLElement[] = [];
    for (let i = 0; i < 3; i++) {
      const axis = document.createElement('div');
      axis.className = 'axis';
      const lab = document.createElement('label');
      lab.textContent = 'XYZ'[i];
      const s = document.createElement('input');
      s.type = 'range';
      s.min = String(Math.round(R2D(c.min[i])));
      s.max = String(Math.round(R2D(c.max[i])));
      s.step = '1';
      s.value = '0';
      const val = document.createElement('span');
      val.className = 'val';
      val.textContent = '0°';
      s.addEventListener('input', () => {
        const deg: [number, number, number] = [
          Number(sliders[0].value), Number(sliders[1].value), Number(sliders[2].value),
        ];
        dragController?.setBoneEuler(bone, deg, order);
      });
      axis.append(lab, s, val);
      row.append(axis);
      sliders.push(s);
      valEls.push(val);
    }
    list.append(row);
    jointRows.push({ bone, order, nameEl, badgeEl, sliders, valEls });
  }
}

function updateJointPanel(): void {
  if (!dragController) return;
  const selected = dragController.getSelected();
  for (const r of jointRows) {
    const stored = dragController.getStoredEulerDeg(r.bone, r.order);
    const rendered = renderedEulerDeg(r.bone, r.order);
    let clamped = false;
    for (let i = 0; i < 3; i++) {
      const sv = Math.round(stored[i]);
      const rv = Math.round(rendered[i]);
      // Don't fight the user while they drag a slider.
      if (document.activeElement !== r.sliders[i]) r.sliders[i].value = String(sv);
      const diff = Math.abs(rv - sv) > 1;
      r.valEls[i].textContent = diff ? `${sv}→${rv}°` : `${rv}°`;
      r.valEls[i].classList.toggle('clamped', diff);
      if (diff) clamped = true;
    }
    r.badgeEl.textContent = clamped ? 'ROM-clamped' : '';
    r.nameEl.classList.toggle('sel', r.bone === selected);
  }
}

function dumpJoints(): void {
  if (!dragController || !validator) return;
  const cons = validator.getConstraints();
  const rows = dragController.getBones().map((bone) => {
    const c = cons[bone as VRMHumanBoneName];
    if (!c) return null;
    const order = c.order as THREE.EulerOrder;
    const req = dragController!.getStoredEulerDeg(bone, order).map(Math.round);
    const ren = renderedEulerDeg(bone, order).map(Math.round);
    const cone = validator!.getConeExtentDeg(bone as VRMHumanBoneName);
    const rom = (i: number): string => `${Math.round(R2D(c.min[i]))}..${Math.round(R2D(c.max[i]))}`;
    return {
      bone, order,
      requested: `[${req.join(', ')}]`,
      rendered: `[${ren.join(', ')}]`,
      clamped: req.some((v, i) => Math.abs(v - ren[i]) > 1) ? 'YES' : '',
      romX: rom(0), romY: rom(1), romZ: rom(2),
      cone: cone ? `${cone.axes} | swingA ${cone.swingA} swingB ${cone.swingB} twist ${cone.twist}` : 'box',
    };
  }).filter(Boolean);
  console.log(
    `%c[joints] profile=${validator.profileId} ROM=${validator.enabled} hardLock=${hardlockEl.checked}`,
    'color:#2ecc71;font-weight:bold',
  );
  console.table(rows);
}

(document.getElementById('joints-dump') as HTMLButtonElement)
  .addEventListener('click', dumpJoints);

// ── Render loop ──────────────────────────────────────────────────────────────
let statusTimer = 0;

function animate(): void {
  requestAnimationFrame(animate);
  const delta = clock.getDelta();
  orbit.update();

  const vrm = dragController?.getVrm() ?? null;
  if (vrm && dragController && validator) {
    // 1. Reset the normalized rig to its rest pose (T-pose). The viewer has no
    //    base animation, so rest = identity on every humanoid bone.
    vrm.humanoid.resetNormalizedPose();
    // 2. Layer the accumulated drag deltas on top (rest * delta = absolute).
    dragController.apply();
    // 3. Clamp to per-bone ROM (no-op when the toggle is off).
    const stats = validator.clampAll();
    // 4. Propagate normalized -> raw bones, run spring bones / expressions.
    vrm.update(delta);
    // 5. Reposition the gizmo onto the (now clamped) selected bone.
    dragController.update();

    statusTimer += delta;
    if (statusTimer > 0.1) {
      statusTimer = 0;
      renderStatus(stats.clampedThisFrame, stats.worstBone);
      updateJointPanel();
    }
  }

  renderer.render(scene, camera);
}
animate();

function renderStatus(clamped: number, worst: string | null): void {
  const sel = dragController?.getSelected();
  const parts: string[] = [];
  parts.push(sel ? `Selected: <b>${sel}</b>` : 'Click a bone marker to select');
  if (validator?.enabled && clamped > 0) {
    parts.push(
      `<span class="clamped">Clamped ${clamped} bone${clamped > 1 ? 's' : ''}` +
        (worst ? ` (worst: ${worst})` : '') +
        '</span>',
    );
  }
  statusEl.innerHTML = parts.join('<br>');
}

// ── Resize ───────────────────────────────────────────────────────────────────
window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
