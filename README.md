# VRM Bone Viewer

Minimal standalone viewer for a VRM avatar with **per-bone rotation constraints
(range-of-motion)**. Grab a bone with the mouse, rotate it, and watch the ROM
clamp stop it at the anatomical limit — or flip the black switch to let it rotate
freely.

## Run

```bash
npm install
npm run dev        # http://localhost:5180
```

## Use

- **Orbit / zoom** — drag empty space, scroll to zoom.
- **Show VRM mesh** — the black switch at the top. Off (default) shows just the
  **skeleton** (`THREE.SkeletonHelper` over the posed bones); on shows the VRM
  mesh. Grab markers stay visible in both modes, so you can pose in skeleton view.
- **Move a body part** — grab a **black joint marker** (hips, spine, chest, neck,
  head, arms, legs) and drag it. The grabbed bone rotates so its child joint aims
  at the cursor (single-bone drag-to-aim), so the limb follows your mouse.
- **Bone constraints (ROM)** — the black switch. ON = each bone is clamped to its
  anatomical range every frame (elbow can't bend backward, arm can't over-rotate,
  etc.). OFF = bones rotate without limits. This is a *render-time* clamp: you can
  still drag past the limit and the bone snaps back; the raw over-rotation is kept
  internally and reappears if you turn ROM off.
- **Hard lock — can't break rules** — clamps at the *source*, the moment you drag.
  The bone physically stops dead at its limit and an illegal angle is never even
  recorded. Turning it on also legalises any pose already made. Use this when you
  want a hard guarantee that no bone can be posed outside the rules.
- **Profile**:
  - `strict (coupled)` — **default**. Anatomical ranges enforced as a *coupled
    swing-twist cone*: single-axis motion reaches its full range, but the axes are
    linked, so a joint can't max several at once (no hip-flex + abduct + twist
    pretzel) and axial twist shrinks near the swing limit. The realistic one.
  - `default (box)` — the same ranges as independent per-axis limits (a joint can
    sit at max on every axis simultaneously → some impossible combined poses).
  - `mixamoLive (wide)` — much wider envelope from a dance/mocap corpus; allows
    extreme poses. For stylised content, not realism.
- **Reset pose** — snap every dragged bone back to T-pose.

The status line shows the selected bone and how many bones the clamp is currently
catching.

## Joints panel (right side)

Every draggable joint (20 bones) gets a row with **X / Y / Z sliders**. This is the
second way to pose — precise, per-axis, complementing mouse drag:

- **Slide to pose** — each slider is bounded to that bone's ROM in the active
  profile, so you can't request past the anatomical limit on a single axis.
- **Two-way sync** — dragging a marker in the 3D view moves the sliders to match
  the pose, and vice-versa. The selected joint's name is highlighted.
- **requested → actual diagnostics** — when the clamp moves an axis (e.g. the
  strict cone couples a combined pose, or hard lock trims it), the value shows
  `requested→actual°` in **orange** and the row gets a **ROM-clamped** badge. This
  is how you *see* where a limit is biting and by how much.
- **Dump → console** — prints a `console.table` of every joint: `requested`,
  `rendered`, `clamped` flag, per-axis ROM (`romX/Y/Z`), and the swing-twist
  **cone extents** (`twist`, `swingA`, `swingB`). The fastest way to audit the
  limits for a bad/asymmetric value — plus a header line with the active profile,
  ROM and hard-lock state.

## Hips constraint

The shared constraint set leaves **Hips** unconstrained (it carries the avatar's
global root orientation during animation playback). In a poser that means grabbing
the pelvis marker rotates the whole avatar freely — hard lock looks broken. `main.ts`
passes a `BoneValidator` override giving Hips a pelvic-tilt ROM
(`X±30° Y±45° Z±25°`) so hard lock applies to every draggable joint.

## Debug logs

Flip on **Debug logs** and open the browser console (F12). You get
`[grab]`/`[drag]`/`[release]` events with the stored Euler, ROM check, and how
much hard lock corrected (`hardLockCorrectionDeg`). A `[hardlock] … STILL OUT OF
ROM after clamp` warning fires **always** (even with debug off) if hard lock ever
leaves a bone outside its ROM — the direct signal that hard lock failed.

### The clamp: Euler box + geodesic flip-guard

`BoneValidator.clampQuaternion` enforces each bone's anatomical ROM (the per-axis
Euler `min/max`) in two passes:

1. **Per-axis Euler clamp** — quaternion → Euler (bone's order) → clamp each axis
   → back. Exact and cheap.
2. **Flip guard** — quaternion→Euler forces the *middle* axis into `[-90°,90°]`,
   so a bone bent past 90° there decomposes on the antipodal branch and pass 1
   snaps it to a wrong orientation. Pass 2 re-checks the result; if it's not
   actually inside the box, it falls back to a **geodesic pullback**: slerp from
   rest toward the original orientation and binary-search the furthest point still
   inside ROM. No fold, always lands in range — the bone stops cleanly at its
   limit from any direction.

Two earlier attempts and why they failed:
- *Pure Euler* (pass 1 only) — the middle-axis fold snapped bones to garbage
  ("doesn't always work").
- *Swing–twist limiter* — avoided the fold, but reused the Euler numbers as swing
  bounds, a different space, so poses drifted up to ~120° outside the intended
  ROM (bones bent into unrealistic poses that hard lock let through).

Current clamp is verified: **43,200 random poses × 54 bones → 0° residual** — every
result lands inside the anatomical box.

### `strict` — coupled swing-twist cone (default profile)

The Euler box has one weakness for a poser: it treats the axes as independent, so
a joint can sit at max on all three at once — hip fully flexed AND abducted AND
twisted is a numerically-valid pretzel. The `strict` profile fixes that in
`clampCone`:

- Split the local rotation into **twist** (about the bone axis) + **swing** (where
  the bone points) — gimbal-free.
- Clamp the swing to an **ellipse** over the two swing axes, so the axes are
  *coupled*: you can max one direction or blend, never both corners.
- Clamp twist to a range that **shrinks toward the swing limit** (real joints lose
  axial rotation at the end of their arc).

Extents come straight from the bone's Euler box (`deriveConeExtent` reads the
twist-axis range for twist and the other two for the swing plane — no volume
sampling, which had pulled in impossible corner combos and let twist wrap past
±180°). Single-axis range is unchanged; only impossible *combinations* are cut.

Hinges (elbow, knee) flex in one plane, so the `strict` constraint set pins their
perpendicular swing tight (`strictLowerArm`; the knee is already tight). Otherwise
the ported elbow box left a ~150° sideways margin and the forearm could swing out.

Verified: hip flex 125° / abduct 55° / knee flex 140° / elbow flex 140° pass
untouched; hip flex100+abduct50+twist40, knee sideways/twist, and elbow sideways
swing are clamped; twist ranges stay anatomical (arm ±95°, not the ±347° the old
sampler produced); 160 drags → 0° residual.

## How it works

Per frame:

1. `vrm.humanoid.resetNormalizedPose()` — normalized rig back to rest (T-pose).
2. `BoneMoveController.apply()` — re-assert each dragged bone's absolute rotation
   (from mouse drag or the panel sliders).
3. `BoneValidator.clampAll()` — clamp every bone to its ROM. `default`/`mixamoLive`
   use the Euler-box + geodesic clamp; `strict` uses the coupled swing-twist cone.
   No-op when the ROM switch is off. (Hard lock has already clamped the stored
   rotation at drag/slide time, independently of this.)
4. `vrm.update()` — propagate normalized → raw bones, run spring bones.

## Source

`boneConstraints.ts`, `boneValidator.ts`, `vrmLoader.ts` and `sample.vrm` are
lifted from the `vrm-player` project. `boneMoveController.ts` (drag-to-aim posing)
is new for this viewer.

Debug handle: `window.__viewer = { vrm, dragController, validator }`.
