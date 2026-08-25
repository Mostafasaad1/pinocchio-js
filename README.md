# Pinocchio-js

> High-performance **WebAssembly** port of the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) rigid body dynamics library — runs in browsers and Node.js with full TypeScript support.

[![License: BSD-2-Clause](https://img.shields.io/badge/License-BSD--2--Clause-blue.svg)](LICENSE)
[![npm](https://img.shields.io/npm/v/pinocchio-js)](https://www.npmjs.com/package/pinocchio-js)

---

## Live Demo

A production application built on Pinocchio-js:
[robot-analyzer.io](https://mostafasaad1.github.io/robot-analyzer-js/)

---

## Features

| Category | What is Included |
|---|---|
| **Dynamics Algorithms** | RNEA (inverse dynamics), ABA (forward dynamics), ABA with external forces, CRBA (mass matrix), kinetic/potential energy, generalized gravity, non-linear effects |
| **Kinematics** | Forward kinematics (position-only), forward kinematics with velocity and acceleration (QVA), joint/frame Jacobians, frame velocities, frame accelerations |
| **Center of Mass** | CoM position, CoM Jacobian |
| **Joint Configuration** | `integrate` on the Pinocchio configuration manifold — correct for SO(3)/SE(3) free-flyer joints |
| **Collision Detection** | Broad-phase AABB collision checker (pure JS, no native deps) |
| **URDF Parsing** | Pure-JS parser with fixed-joint reduction |
| **TypeScript** | Comprehensive `.d.ts` declarations for the WASM module, URDF parser, and collision checker |
| **Performance** | Zero-copy bulk data transfer, pre-allocated output buffers, `MinSizeRel` WASM binary (~457 KB) |
| **Cross-Platform** | Browsers (Chrome, Firefox, Safari) and Node.js; Windows/Linux/macOS build scripts |

---

## What is New (since v1.2.2)

> All features below were added after commit `1f88fd3` (v1.2.2). See [CHANGELOG.md](CHANGELOG.md) for the full history.

### v1.4.0 — Performance Maximization (spec 009)

- **Zero-copy data transfer** between JS and WASM using `emscripten::typed_memory_view` — eliminates intermediate buffer copies in all hot-path functions.
- **Pre-allocated output buffers** — every algorithm that returns a `Float64Array` now accepts an optional `outArray?: Float64Array` parameter. Passing a pre-allocated buffer produces **zero JS heap allocations** per call, enabling GC-free simulation loops.
  - Affected: `rnea`, `aba`, `abaWithForces`, `crba`, `centerOfMass`, `centerOfMassJacobian`, `computeGeneralizedGravity`, `nonLinearEffects`, `getJointJacobian`, `computeFrameJacobian`, `getFrameJacobian`, `getFrameVelocity`, `integrate`.
- **WASM binary size** reduced to **~457 KB** (`MinSizeRel` CMake target, ~44% reduction).
- **Collision checker** `updateCollisions` hot-path optimized — eliminates `Array.from`, `Array.map`, and per-link object allocations (~0.018 ms per configuration).
- **URDF parser** refactored with direct-child DOM traversal and indexed BFS joint lookups (O(N) vs O(N^2)).
- **Benchmark suites**: `tests/benchmark_algorithms.js` and `tests/benchmark-prealloc.js`.

### v1.3.0 — Bindings, TypeScript and Collision (specs 001-008)

#### Windows Build Support

- `build.ps1` — full-featured PowerShell build script with feature-parity with `build.sh`.
- `npm run build` now works cross-platform.

#### TypeScript Declarations (spec 001)

- `src/pinocchio.d.ts` — complete module type definitions.
- `src/urdf-parser.d.ts` — URDF data structure types.
- `src/collision-checker.d.ts` — collision checker types.
- `npm run test:types` — `tsc --noEmit` static type validation.

#### Broad-Phase Collision Detection (specs 002 and 003)

- `src/collision-checker.mjs` — pure-JS broad-phase AABB collision checker.
- `setIgnoredPairs(pairs)` — exclude custom link pairs; parent-child pairs auto-ignored at construction.
- `updateCollisions(q)` — runs FK internally.
- `checkCollisions()` returns `{ hasCollision: boolean, contacts: [{linkA, linkB}] }`.

#### ABA with External Forces (spec 004)

- `abaWithForces(model, data, q, v, tau, fext?)` — forward dynamics with optional spatial forces applied to arbitrary links.
- `fext`: `{ [linkIndex: number]: Float64Array(6) }` — wrenches `[tx, ty, tz, fx, fy, fz]` in the joint local frame.
- `Data.getVelocity(jointId)` and `Data.getAcceleration(jointId)` accessors added.

#### Forward Kinematics QVA (spec 005)

- `forwardKinematicsQVA(model, data, q, v, a)` — full FK pass populating positions, joint spatial velocities (`data.v[]`), and joint spatial accelerations (`data.a[]`). **Required prerequisite** for frame velocities and accelerations.

#### Frame-Level Jacobians (spec 006)

- `computeJointJacobians(model, data, q)` — computes all joint Jacobians into `data.J`.
- `getJointJacobian(model, data, jointId, refFrame)` — 6 x nv Jacobian (column-major).
- `computeFrameJacobian(model, data, q, frameId, refFrame)` — single-call frame Jacobian.
- `getFrameJacobian(model, data, frameId, refFrame)` — frame Jacobian after `computeJointJacobians`.
- `ReferenceFrame` enum: `WORLD = 0`, `LOCAL = 1`, `LOCAL_WORLD_ALIGNED = 2`.

#### Frame Spatial Velocities (spec 007)

- `getFrameVelocity(model, data, frameId, refFrame)` — returns `Float64Array(6)` `[vx, vy, vz, wx, wy, wz]`.

#### Frame Spatial Accelerations (spec 008)

- `getFrameAcceleration(model, data, frameName, refFrame, out_acceleration)` — zero-copy output into caller-allocated `Float64Array(6)` `[ax, ay, az, alpha_x, alpha_y, alpha_z]`.

#### Center of Mass Jacobian (spec 010)

- `centerOfMassJacobian(model, data, q)` — returns `Float64Array(3 x nv)` mapping joint velocities to CoM velocity.

#### Joint Configuration Integration (spec 011)

- `integrate(model, q, v, outArray?)` — on-manifold integration respecting joint topology:
  - Euclidean joints: `q_new = q + v`.
  - Free-flyer / spherical joints: correct SO(3) / SE(3) exponential map (unit quaternion preserved).
  - Full dimension validation with descriptive `RangeError` messages.

---

## Installation

### 1. NPM (Recommended)

```bash
npm install pinocchio-js
```

### 2. GitHub Releases

Download pre-compiled binaries (`pinocchio.js` and `pinocchio.wasm`) from the [Releases page](https://github.com/Mostafasaad1/pinocchio-js/releases).

### 3. Building from Source

#### Prerequisites

1. **Emscripten SDK (emsdk)**
   ```bash
   git clone https://github.com/emscripten-core/emsdk.git
   cd emsdk
   ./emsdk install latest
   ./emsdk activate latest
   source ./emsdk_env.sh  # Linux/macOS
   ```
2. **CMake** >= 3.10
3. **Python 3**

#### One-Command Build

**Windows (PowerShell):**
```powershell
npm run build
# or directly:
.\build.ps1
```

**Linux / macOS:**
```bash
./build.sh
```

The build script automatically locates Pinocchio v2, Eigen 3.4 (via CMake FetchContent), and Boost headers (via Emscripten Ports). Output: `build/pinocchio.js` and `build/pinocchio.wasm`.

---

## Quick Start

### Node.js

```js
const createPinocchioModule = require('./build/pinocchio.js');
const { parseURDF, buildPinocchioModel } = require('./src/urdf-parser.mjs');
const fs = require('fs');

async function main() {
    const pin = await createPinocchioModule();

    const urdfXml = fs.readFileSync('urdf/abb_irb120_support/urdf/irb120_3_58.urdf', 'utf8');
    const urdfData = parseURDF(urdfXml);
    const model = buildPinocchioModel(pin, urdfData);
    const data = new pin.Data(model);

    console.log(`Loaded: ${model.njoints} joints, nq=${model.nq}, nv=${model.nv}`);

    const q = pin.neutralConfiguration(model);
    const v = new Float64Array(model.nv);
    const a = new Float64Array(model.nv);

    // Inverse dynamics
    const tau = pin.rnea(model, data, q, v, a);
    console.log('Gravity torques:', tau);

    // Pre-allocated buffer — zero-allocation hot path
    const tauBuf = new Float64Array(model.nv);
    pin.rnea(model, data, q, v, a, tauBuf);
}
main();
```

### TypeScript

```typescript
import createPinocchioModule, { Model, Data, ReferenceFrame, ExternalForceMap } from 'pinocchio-js';
import { parseURDF, buildPinocchioModel } from 'pinocchio-js/src/urdf-parser.mjs';

async function main() {
    const pin = await createPinocchioModule();

    const urdfData = parseURDF(/* xml string */);
    const model: Model = buildPinocchioModel(pin, urdfData);
    const data: Data = new pin.Data(model);

    const q = new Float64Array(model.nq);
    const v = new Float64Array(model.nv);
    const a = new Float64Array(model.nv);

    // Full FK with velocities and accelerations
    pin.forwardKinematicsQVA(model, data, q, v, a);

    // Frame velocity
    const frameId = model.getFrameId('end_effector');
    const vel = pin.getFrameVelocity(model, data, frameId, ReferenceFrame.LOCAL_WORLD_ALIGNED);

    // Frame Jacobian
    const J = pin.computeFrameJacobian(model, data, q, frameId, ReferenceFrame.LOCAL_WORLD_ALIGNED);

    // External forces
    const fext: ExternalForceMap = { 3: new Float64Array([0, 0, 0, 10, 0, 0]) };
    const tau = new Float64Array(model.nv);
    pin.abaWithForces(model, data, q, v, tau, fext);

    // On-manifold integration — correct for free-flyer joints
    const v_cmd = new Float64Array(model.nv).fill(0.01);
    const q_next = pin.integrate(model, q, v_cmd);
}
```

### Browser

Serve the project root with any static file server:
```bash
python3 -m http.server 8080
```
Open `http://localhost:8080/tests/browser/index.html` for the full interactive browser test suite with per-test pass/fail badges and timing.

---

## API Reference

### Module Initialization

```js
const pin = await createPinocchioModule();
```

### Enumerations

```ts
enum ReferenceFrame {
    WORLD               = 0,
    LOCAL               = 1,
    LOCAL_WORLD_ALIGNED = 2,
}
```

### `pin.Model`

| Property / Method | Description |
|---|---|
| `nq` | Configuration space dimension |
| `nv` | Velocity space dimension |
| `njoints` | Number of joints (including universe) |
| `nframes` | Number of frames |
| `name` | Robot name |
| `existFrame(name)` | Check if a named frame exists |
| `getFrameId(name)` | Look up frame ID by name |
| `delete()` | Free WASM memory |

### `pin.Data`

| Method | Description |
|---|---|
| `getVelocity(jointId)` | 6-element spatial velocity of joint `i` from `data.v[i]` |
| `getAcceleration(jointId)` | 6-element spatial acceleration of joint `i` from `data.a[i]` |
| `oMf(frameId)` | World-frame SE3 placement of frame (after `updateFramePlacements`) |
| `delete()` | Free WASM memory |

### Geometry

```ts
SE3.identity(): SE3
SE3.fromRotationTranslation(rot: Float64Array, trans: Float64Array): SE3
SE3.fromXyzRpy(x, y, z, roll, pitch, yaw): SE3

Inertia.fromMassComInertia(mass, com[3], inertia[6]): Inertia
// inertia = [Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
```

### Joint Models

| Factory | Type | DOF |
|---|---|---|
| `JointModelRX()`, `JointModelRY()`, `JointModelRZ()` | Revolute | 1 |
| `JointModelPX()`, `JointModelPY()`, `JointModelPZ()` | Prismatic | 1 |
| `JointModelRevoluteUnaligned(ax, ay, az)` | Revolute, arbitrary axis | 1 |
| `JointModelPrismaticUnaligned(ax, ay, az)` | Prismatic, arbitrary axis | 1 |
| `JointModelFreeFlyer()` | 6-DOF free joint | 6 (nq=7) |
| `JointModelFixed()` | Welded / zero-DOF | 0 |

### Model Construction

```ts
addJoint(model, parentId, joint, placement, name): number
addJointWithLimits(model, parentId, joint, placement, name,
    maxEffort, maxVelocity, minConfig, maxConfig): number
appendBodyToJoint(model, jointId, inertia, bodyPlacement): void
```

### Dynamics Algorithms

All functions returning `Float64Array` accept an optional pre-allocated `outArray` parameter for zero-allocation operation.

| Function | Description | Output |
|---|---|---|
| `rnea(model, data, q, v, a, outArray?)` | Recursive Newton-Euler — inverse dynamics | `tau [nv]` |
| `aba(model, data, q, v, tau, outArray?)` | Articulated Body Algorithm — forward dynamics | `ddq [nv]` |
| `abaWithForces(model, data, q, v, tau, fext?, outArray?)` | ABA with spatial external forces | `ddq [nv]` |
| `crba(model, data, q, outArray?)` | Composite Rigid Body — joint-space inertia matrix | `M [nv x nv]` col-major |
| `computeKineticEnergy(model, data, q, v)` | Total kinetic energy | `number` (Joules) |
| `computePotentialEnergy(model, data, q)` | Total potential energy | `number` (Joules) |
| `computeGeneralizedGravity(model, data, q, outArray?)` | Gravity compensation torques | `g [nv]` |
| `nonLinearEffects(model, data, q, v, outArray?)` | Coriolis + centrifugal + gravity | `nle [nv]` |

**External Force Map** (`fext`):
```ts
type ExternalForceMap = { [linkIndex: number]: Float64Array };
// Each wrench: [tx, ty, tz, fx, fy, fz] in joint local frame (N*m, N)
// Duplicate link indices are accumulated additively
```

### Kinematics

```ts
forwardKinematics(model, data, q): void
// Position-only FK. Populates data.oMi[].

forwardKinematicsQVA(model, data, q, v, a): void
// Full FK with velocities and accelerations.
// Required before getFrameVelocity or getFrameAcceleration.

updateFramePlacements(model, data): void
// Propagates FK results to all URDF-defined frames (data.oMf[]).
// Call after forwardKinematics for non-joint frames.

getJointPlacement(data, jointId): { translation: Float64Array, rotation: Float64Array }
// World pose of joint after forwardKinematics.
// rotation is 3x3 matrix in column-major order.
```

### Jacobians

All Jacobians are returned in **column-major** `Float64Array` of length `6 x nv`.

```ts
computeJointJacobians(model, data, q): void
getJointJacobian(model, data, jointId, refFrame, outArray?): Float64Array
computeFrameJacobian(model, data, q, frameId, refFrame, outArray?): Float64Array
getFrameJacobian(model, data, frameId, refFrame, outArray?): Float64Array
```

### Frame Velocities and Accelerations

```ts
getFrameVelocity(model, data, frameId, refFrame, outArray?): Float64Array
// Returns [vx, vy, vz, wx, wy, wz]. Requires prior forwardKinematicsQVA.

getFrameAcceleration(model, data, frameName, refFrame, out_acceleration: Float64Array): void
// Zero-copy: writes [ax, ay, az, alpha_x, alpha_y, alpha_z] into caller-allocated buffer.
// Throws Error if frameName not found or forwardKinematicsQVA was not called.
```

### Center of Mass

```ts
centerOfMass(model, data, q, outArray?): Float64Array        // [x, y, z] in world frame
centerOfMassJacobian(model, data, q, outArray?): Float64Array // 3 x nv, column-major
computeTotalMass(model): number
```

### Configuration Utilities

```ts
randomConfiguration(model): Float64Array   // nq — random config respecting joint limits
neutralConfiguration(model): Float64Array  // nq — zero / identity configuration

integrate(model, q, v, outArray?): Float64Array
// On-manifold Lie group integration.
// Euclidean joints: q_new = q + v
// Free-flyer / spherical: correct SO(3)/SE(3) exponential map.
// Throws RangeError with descriptive message on dimension mismatch.
```

### Data Accessors

```ts
getTau(data): Float64Array        // data.tau — joint torques
getNle(data): Float64Array        // data.nle — non-linear effects
getComAt(data, idx): Float64Array // CoM of subtree at joint idx
```

### URDF Parser (`src/urdf-parser.mjs`)

```ts
import { parseURDF, buildPinocchioModel } from 'pinocchio-js/src/urdf-parser.mjs';

parseURDF(xmlString: string): URDFData
buildPinocchioModel(pin: PinocchioModule, urdfData: URDFData): Model
```

- **Fixed Joint Reduction**: URDF fixed joints are fused into their parent — eliminates zero-DOF joints and ensures numerical stability.
- Supported joint types: `revolute`, `continuous`, `prismatic`, `floating`, `fixed`.
- Runtime dependency: `xmldom ^0.6.0` for Node.js XML DOM support.

### Collision Checker (`src/collision-checker.mjs`)

```ts
import { CollisionChecker } from 'pinocchio-js/src/collision-checker.mjs';

const checker = new CollisionChecker(pin, model, data, urdfData);
checker.setIgnoredPairs([['link1', 'link2'], ['base_link', 'shoulder']]);
checker.updateCollisions(q);          // runs FK internally
const result = checker.checkCollisions();
// { hasCollision: boolean, contacts: [{ linkA: string, linkB: string }] }
```

- Broad-phase AABB check only (no narrow-phase geometry).
- Adjacent parent-child link pairs are automatically ignored at construction.
- Optimized hot-path: ~0.018 ms per configuration on typical robots.

---

## Testing

```bash
# Unit and integration tests (Node.js)
npm test

# TypeScript static type check
npm run test:types

# Smoke test (quick sanity check)
npm run test:smoke
```

| Test File | What It Tests |
|---|---|
| `tests/test_math.js` | SE3 and Inertia constructors |
| `tests/test_model.js` | Joint creation, model API |
| `tests/test_algo.js` | RNEA, ABA, CRBA, CoM, CoM Jacobian, frame Jacobians, pre-allocated buffers, `integrate` |
| `tests/test_urdf.js` | URDF loading (ABB IRB120, KUKA KR210, ABB YuMi) |
| `tests/test_collision.js` | Broad-phase collision detection |
| `tests/test_aba_fext.js` / `test_aba_fext_deep.js` | ABA with external forces |
| `tests/test_forward_kinematics.js` | FK QVA |
| `tests/test_frames.js` / `test_frames_deep.js` | Frame velocities, accelerations, Jacobians |
| `tests/typescript/test.ts` | TypeScript typed API |
| `tests/benchmark_algorithms.js` | Throughput / latency benchmarks |
| `tests/benchmark-prealloc.js` | Zero-allocation verification |

Browser tests: open `tests/browser/index.html` — shows per-test pass/fail badges, timing, and inline stack traces.

---

## Included URDF Assets

| Robot | Joints | DOF | Notes |
|---|---|---|---|
| **ABB IRB 120** (`urdf/abb_irb120_support/`) | 7 | 6 | STL collision + visual meshes |
| **KUKA KR 210** (`urdf/kuka_kr210_support/`) | 7 | 6 | DAE visual + STL collision meshes |
| **ABB YuMi** (`urdf/yumi_description/`) | 19 | 14 | Dual-arm; STL meshes |

---

## Feature Roadmap (specs)

| # | Spec | Status |
|---|---|---|
| 001 | TypeScript definitions | Shipped (v1.3.0) |
| 002 | Lightweight collision checking | Shipped (v1.3.0) |
| 003 | JS collision checking (AABB) | Shipped (v1.3.0) |
| 004 | ABA with external forces | Shipped (v1.3.0) |
| 005 | Forward kinematics QVA | Shipped (v1.3.0) |
| 006 | Frame-level Jacobians | Shipped (v1.3.0) |
| 007 | Frame velocities | Shipped (v1.3.0) |
| 008 | Frame accelerations | Shipped (v1.3.0) |
| 009 | Maximize performance | Shipped (v1.4.0) |
| 010 | CoM Jacobian | Shipped (v1.4.0) |
| 011 | Joint configuration integration | Shipped (v1.4.0) |

---

## Performance Tips

1. **Pre-allocate output buffers** once and reuse in simulation loops to avoid GC pressure:
   ```js
   const tauBuf = new Float64Array(model.nv);
   const JBuf   = new Float64Array(6 * model.nv);
   // In loop:
   pin.rnea(model, data, q, v, a, tauBuf);
   pin.getJointJacobian(model, data, jointId, refFrame, JBuf);
   ```

2. **Call `forwardKinematicsQVA` once** per time step if you need Jacobians, velocities, *and* accelerations — they all read from the same `data.v[]` / `data.a[]` arrays.

3. **Avoid calling `updateFramePlacements`** unless you need `data.oMf`. Most algorithms operate directly on joint data.

4. **Build with `MinSizeRel`** for deployment — the resulting WASM binary is ~457 KB gzipped.

---

## Contributing

1. Fork the repository.
2. Run `npm install` and `npm run build` (or `./build.sh`) to verify the environment.
3. Add tests for new features in `tests/`.
4. Run `npm test` and `npm run test:types` — all must pass.
5. Submit a Pull Request.

---

## License

BSD-2-Clause (c) Mostafa Saad
