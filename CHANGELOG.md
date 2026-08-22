# Changelog

All notable changes to **pinocchio-js** are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/) conventions.
Versioning follows [Semantic Versioning](https://semver.org/).

---

## [1.3.0] — 2026-08-22

### Added — Build System

- **Windows PowerShell build script** (`build.ps1`):
  - Full feature-parity with Linux/macOS `build.sh`.
  - Auto-locates the Emscripten SDK, Pinocchio v2 source, Eigen 3.4 (via CMake `FetchContent`), and Boost headers (via Emscripten Ports).
  - Produces `build/pinocchio.js` and `build/pinocchio.wasm` in a single command.
  - Integrated into `npm run build` for cross-platform builds.
- **`CMakeLists.txt`** updated for Windows path compatibility.
- **`.gitignore`** extended to exclude Windows-specific build artefacts.

### Added — TypeScript Declarations

- **`src/pinocchio.d.ts`** — complete TypeScript definitions for the WASM module:
  - `ReferenceFrame` enum: `WORLD = 0`, `LOCAL = 1`, `LOCAL_WORLD_ALIGNED = 2`.
  - Interface `EmbindObject` with mandatory `delete()` lifecycle method.
  - Interface `JointPlacement` — `{ translation: Float64Array; rotation: Float64Array }`.
  - Class declarations with full method signatures: `SE3`, `Inertia`, `JointModel`, `Model`, `Data`.
  - Free functions typed with `Float64Array | ArrayLike<number>` overloads for ergonomic usage.
  - `ExternalForceMap` type alias: `{ [linkIndex: number]: Float64Array | ArrayLike<number> }`.
  - `PinocchioModule` interface — full module shape enabling type-safe destructuring imports.
  - `createPinocchioModule(moduleOverrides?)` default export typed as `Promise<PinocchioModule>`.
- **`src/urdf-parser.d.ts`** extended with type declarations for URDF data structures.
- **`src/collision-checker.d.ts`** — TypeScript declarations for the collision checker:
  - `AABB` interface: `{ min: [number, number, number]; max: [number, number, number] }`.
  - `CollisionResult` interface: `{ hasCollision: boolean; contacts: Array<{ linkA: string; linkB: string }> }`.
  - `CollisionChecker` class with fully typed constructor and methods.
- **`tests/typescript/test.ts`** — TypeScript test suite exercising typed APIs.
- **`tests/typescript/tsconfig.json`** — TypeScript project config for the type test suite.
- **`npm run test:types`** script: runs `tsc --project tests/typescript/tsconfig.json --noEmit`.
- **devDependencies**: `typescript ^5.3.3`, `@types/node ^26.2.0`.

### Added — Collision Detection

- **`src/collision-checker.mjs`** — pure-JavaScript broad-phase AABB collision checker:
  - **`CollisionChecker` class** — constructor takes `(pin, model, data, urdfData)` and initializes axis-aligned bounding boxes from URDF `<collision>` geometries.
  - **`setIgnoredPairs(pairs)`** — configure custom link pairs to exclude from collision checks. Parent-child adjacent link pairs are automatically ignored at construction time.
  - **`updateCollisions(q)`** — runs `forwardKinematics` internally to place all bounding volumes in world space given joint configuration `q: Float64Array`.
  - **`checkCollisions()`** returns `{ hasCollision: boolean, contacts: [{linkA, linkB}] }`.
- `src/urdf-parser.mjs` extended to extract URDF `<collision>` geometries during parsing.
- `tests/test_collision.js` — collision detection test suite.

### Added — ABA with External Forces

- **`abaWithForces(model, data, q, v, tau, fext?): Float64Array`** — extended Articulated Body Algorithm that optionally applies spatial external forces to arbitrary links:
  - `fext` is an External Force Map: `{ [linkIndex: number]: Float64Array(6) }` where each wrench is `[tx, ty, tz, fx, fy, fz]` in the joint's local frame (torques N*m, forces N).
  - Graceful degradation: when `fext` is `undefined`, `null`, or empty, falls back to standard ABA.
  - Duplicate link keys are **accumulated additively**.
  - Full validation with JS-native `RangeError` for bad indices and wrong wrench lengths.
  - C++ helper `jsToForceVector(model, fext_js)` converts the JS object to `PINOCCHIO_ALIGNED_STD_VECTOR(pinocchio::Force)` of size `model.njoints`.
- **`Data.getVelocity(jointId): Float64Array`** — retrieve 6-element spatial velocity of joint from `data.v[]`. Throws `RangeError` on out-of-bounds.
- **`Data.getAcceleration(jointId): Float64Array`** — retrieve 6-element spatial acceleration of joint from `data.a[]`. Throws `RangeError` on out-of-bounds.
- `tests/test_aba_fext.js` and `tests/test_aba_fext_deep.js` test suites.
- `tests/run_deep.js` — runner for the extended deep test suite.

### Added — Forward Kinematics with Velocity and Acceleration

- **`forwardKinematicsQVA(model, data, q, v, a): void`** — full FK pass with positions, velocities, and accelerations:
  - Calls `pinocchio::forwardKinematics(model, data, q, v, a)`.
  - Populates `data.v[i]` (joint spatial velocity) and `data.a[i]` (joint spatial acceleration).
  - **Required prerequisite** before calling `getFrameVelocity` or `getFrameAcceleration`.
- `tests/test_forward_kinematics.js` test suite.

### Added — Frame-Level Jacobians

- **`computeJointJacobians(model, data, q): void`** — computes all joint Jacobians into `data.J`.
- **`getJointJacobian(model, data, jointId, refFrame): Float64Array`** — retrieve the 6*nv Jacobian matrix (column-major) for a specific joint.
- **`computeFrameJacobian(model, data, q, frameId, refFrame): Float64Array`** — single-call Jacobian for a specific frame. Validates frame bounds (throws `RangeError`).
- **`getFrameJacobian(model, data, frameId, refFrame): Float64Array`** — retrieve frame Jacobian after `computeJointJacobians`. Validates frame bounds.
- **`ReferenceFrame` enum**: `WORLD = 0`, `LOCAL = 1`, `LOCAL_WORLD_ALIGNED = 2`.
- `tests/test_frames.js` and `tests/test_frames_deep.js` test suites.

### Added — Modular Browser Test Interface

- **`tests/browser/app.mjs`** — test runner entry point wiring test suite to UI.
- **`tests/browser/test-suite.mjs`** — browser-runnable test cases for all algorithms.
- **`tests/browser/index.html`** refactored into a modular interface with improved diagnostics: per-test pass/fail badges, timing, and inline stack traces.

### Added — Frame Spatial Velocities

- **`getFrameVelocity(model, data, frameId, refFrame): Float64Array`** — retrieve the 6-element spatial velocity `[vx, vy, vz, wx, wy, wz]` of any model frame:
  - Resolves to the specified `ReferenceFrame`.
  - Requires prior `forwardKinematicsQVA` call to populate `data.v[]`.
  - Frame bounds validation — throws `RangeError` if `frameId >= model.frames.size()`.
- TypeScript declarations updated in `pinocchio.d.ts` and `PinocchioModule`.
- `tests/test_frames.js` and `tests/test_frames_deep.js` extended.

### Added — Frame Spatial Accelerations

- **`getFrameAcceleration(model, data, frameName, referenceFrame, out_acceleration): void`**:
  - Takes a **frame name** (string); `model.getFrameId(frameName)` is resolved internally.
  - **Zero-copy output**: writes into a caller-allocated `Float64Array(6)` buffer `[ax, ay, az, alpha_x, alpha_y, alpha_z]` (linear then angular acceleration).
  - **Frame existence guard**: throws `Error` if `frameName` is not found in the model.
  - **Initialization safety heuristic**: inspects `data.v[i]` and `data.a[i]`; if all are zero in a model with `njoints > 1`, throws `Error`: `"forwardKinematicsQVA must be called before getFrameAcceleration"`.
  - Requires prior `forwardKinematicsQVA` call.
- TypeScript declarations updated in `pinocchio.d.ts`.
- `tests/browser/test-suite.mjs`, `tests/test_frames.js`, and `tests/test_frames_deep.js` extended.
- **`validate_frame_accel.mjs`** standalone ESM validation script.

---

## [1.2.2] — 2026-03-01

### Changed

- **Package renamed** to `pinocchio-js`. Updated in `package.json`, `README.md`, and all documentation.
- **README** revised with new package name, updated installation instructions, and TypeScript usage examples via NPM.

---

## [1.2.1] — 2026-02-20

### Added

- **`getJointPlacement(data, jointId): JointPlacement`** — retrieve the world pose of a joint after forward kinematics:
  - Returns `{ translation: Float64Array[3], rotation: Float64Array[9] }` where `rotation` is a 3x3 rotation matrix in column-major order.
  - Reads from `data.oMi[jointId]` (populated by `forwardKinematics`).
- **`updateFramePlacements(model, data): void`** — propagate FK results to all URDF-defined frames stored in `data.oMf[]`:
  - Must be called after `forwardKinematics` to update non-joint frames (sensor frames, end-effector frames defined in the URDF).
  - Required for URDF-based workflows where frames are attached to fixed or floating links.
  - Enables `data.oMf(frameId)` to return correct world-frame SE3 placements.

---

## [1.2.0] — 2026-02-20

### Added

- Exposed `getJointPlacement` and `updateFramePlacements` to WASM bindings.

---

## [1.1.0] — 2026-02-20

### Added

- **`aba(model, data, q, v, tau): Float64Array`** — Articulated Body Algorithm (forward dynamics), computing joint accelerations `ddq`.
- **`crba(model, data, q): Float64Array`** — Composite Rigid Body Algorithm, computing joint-space inertia matrix `M` (symmetrized).
- **`computeKineticEnergy(model, data, q, v): number`** — calculates total kinetic energy in Joules.
- **`computePotentialEnergy(model, data, q): number`** — calculates total potential energy in Joules.
- **`computeGeneralizedGravity(model, data, q): Float64Array`** — computes generalized gravity compensation torques.
- **`nonLinearEffects(model, data, q, v): Float64Array`** — computes Coriolis, centrifugal, and gravity torques `nle`.

---

## [1.0.4] — 2026-02-20

### Infrastructure

- **Node.js upgraded to v24** in CI workflows for npm OIDC token authentication support.

---

## [1.0.2] — 2026-02-20

### Fixed

- **NPM OIDC authentication** in GitHub Actions release workflow.

---

## [1.0.0] — 2026-02-20

Initial public release of **pinocchio-js** — WebAssembly port of the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) rigid body dynamics library, compiled with Emscripten and exposed to JavaScript via Embind.

### Added — Core C++ Embind Layer (`src/pinocchio_embind.cpp`)

#### SE3 — Rigid Body Transformations

- **`SE3.identity()`** — construct the identity SE3 transform.
- **`SE3.fromRotationTranslation(rot, trans)`** — construct from a 9-element row-major rotation matrix and a 3-element translation vector.
- **`SE3.fromXyzRpy(x, y, z, roll, pitch, yaw)`** — construct from URDF-convention XYZ-RPY parameters (roll about X, pitch about Y, yaw about Z — ZYX Euler composition).

#### Inertia

- **`Inertia.fromMassComInertia(mass, com, inertia)`**:
  - `mass`: scalar in kg.
  - `com`: 3-element array (center of mass relative to joint frame).
  - `inertia`: 6-element array `[Ixx, Ixy, Ixz, Iyy, Iyz, Izz]` — unique elements of the symmetric 3x3 inertia tensor.

#### Joint Model Factories

Ten joint types exposed as factory functions returning an opaque `JointModel` handle:

| Function | Type | DOF |
|---|---|---|
| `JointModelRX()` | Revolute about X | 1 |
| `JointModelRY()` | Revolute about Y | 1 |
| `JointModelRZ()` | Revolute about Z | 1 |
| `JointModelPX()` | Prismatic along X | 1 |
| `JointModelPY()` | Prismatic along Y | 1 |
| `JointModelPZ()` | Prismatic along Z | 1 |
| `JointModelRevoluteUnaligned(ax, ay, az)` | Revolute, arbitrary axis | 1 |
| `JointModelPrismaticUnaligned(ax, ay, az)` | Prismatic, arbitrary axis | 1 |
| `JointModelFreeFlyer()` | 6-DOF free joint | 6 |
| `JointModelFixed()` | Welded / zero-DOF | 0 |

#### Model Construction

- **`Model`** class with read-only properties: `nq`, `nv`, `njoints`, `nframes`, `name`.
- **`addJoint(model, parentId, joint, placement, name): number`** — add joint to the kinematic tree; auto-registers a joint frame via `model.addJointFrame(jointId)`.
- **`addJointWithLimits(model, parentId, joint, placement, name, maxEffort, maxVelocity, minConfig, maxConfig): number`** — add a joint with per-axis actuator limits.
- **`appendBodyToJoint(model, jointId, inertia, bodyPlacement): void`** — attach a rigid body inertia to a joint.
- `Model.existFrame(name): boolean` — check if a named frame exists.
- `Model.getFrameId(name): number` — look up a frame's integer ID by name.

#### Data

- **`Data`** class — constructed from `Model`, holds scratchpad arrays.
- **`Data.oMf(frameId): SE3`** — world-frame placement of frame `frameId`.

#### Dynamics Algorithms

| Function | Algorithm | Output |
|---|---|---|
| `rnea(model, data, q, v, a)` | Recursive Newton-Euler (Inverse Dynamics) | `tau: Float64Array[nv]` |
| `aba(model, data, q, v, tau)` | Articulated Body Algorithm (Forward Dynamics) | `ddq: Float64Array[nv]` |
| `crba(model, data, q)` | Composite Rigid Body Algorithm | `M: Float64Array[nv*nv]` column-major, symmetrized |
| `computeKineticEnergy(model, data, q, v)` | Kinetic energy | `number` (Joules) |
| `computePotentialEnergy(model, data, q)` | Potential energy | `number` (Joules) |
| `computeGeneralizedGravity(model, data, q)` | Generalized gravity torques | `g: Float64Array[nv]` |
| `nonLinearEffects(model, data, q, v)` | Coriolis + gravity | `nle: Float64Array[nv]` |

#### Kinematics & Utilities

- **`forwardKinematics(model, data, q): void`** — position-only FK; populates `data.oMi[]`.
- **`centerOfMass(model, data, q): Float64Array[3]`** — system CoM `[x, y, z]` in world frame.
- **`computeTotalMass(model): number`** — total robot mass.
- **`randomConfiguration(model): Float64Array[nq]`** — random config respecting joint limits.
- **`neutralConfiguration(model): Float64Array[nq]`** — neutral (zero) configuration.

#### Data Accessors

- **`getTau(data): Float64Array`** — joint torques from `data.tau`.
- **`getNle(data): Float64Array`** — non-linear effects from `data.nle`.
- **`getComAt(data, idx): Float64Array[3]`** — CoM of subtree rooted at joint `idx`.

#### Eigen <-> JavaScript Type Bridge (Internal)

| Helper | Direction |
|---|---|
| `jsToVectorXd(arr)` | `Float64Array` -> `VectorXd` |
| `vectorXdToJs(v)` | `VectorXd` -> `Float64Array` (copy) |
| `jsToVector3d(arr)` | `Array[3]` -> `Vector3d` |
| `vector3dToJs(v)` | `Vector3d` -> `Float64Array[3]` |
| `jsToMatrix3d(arr)` | `Array[9]` row-major -> `Matrix3d` |
| `matrixXdToJs(m)` | `MatrixXd` -> `Float64Array` column-major |

### Added — URDF Parser (`src/urdf-parser.js`, `src/urdf-parser.mjs`)

Pure JavaScript URDF parser (no native dependencies) — available as both CJS and ESM:

- **`parseURDF(xmlString): URDFData`** — parse URDF XML into an intermediate JS object.
- **`buildPinocchioModel(pin, urdfData): Model`** — convert to a `pin.Model`:
  - **Fixed Joint Reduction**: fixed joints are fused into their parent for numerical stability.
  - Handles URDF joint types: `revolute`, `continuous`, `prismatic`, `floating`, `fixed`.
  - Applies `SE3.fromXyzRpy` for all joint placement origins.
  - Sets joint limits via `addJointWithLimits` when limits are defined.
- Runtime dependency: `xmldom ^0.6.0` for XML DOM support in Node.js.

### Added — Build System

- **`build.sh`** — Linux/macOS build script using `emcmake cmake`.
- **`CMakeLists.txt`** — CMake build definition:
  - Emscripten flags: `-lembind`, `ALLOW_MEMORY_GROWTH`, `MODULARIZE`, `EXPORT_ES6`, `-O3`.
- **Pinocchio shim headers** (`src/pinocchio/config.hpp`, `deprecated.hpp`, `warning.hpp`) — supply missing macros (`PINOCCHIO_DEPRECATED_MESSAGE`, etc.) for CI builds.

### Added — Test Suite

- `tests/runner.js` — Node.js test orchestrator.
- `tests/smoke_test.js` — end-to-end smoke test.
- `tests/test_math.js` — SE3 and Inertia factory tests.
- `tests/test_model.js` — joint construction and model API tests.
- `tests/test_algo.js` — RNEA, CoM, and mass tests.
- `tests/test_urdf.js` — URDF loading tests against bundled robot assets.
- `tests/browser/index.html` — browser test runner.

### Added — Robot URDF Assets (`urdf/`)

| Robot | Joints | DOF | Notes |
|---|---|---|---|
| **ABB IRB 120** (`urdf/abb_irb120_support/`) | 7 | 6 | STL collision + visual meshes |
| **KUKA KR 210** (`urdf/kuka_kr210_support/`) | 7 | 6 | DAE visual + STL collision meshes |
| **ABB YuMi** (`urdf/yumi_description/`) | 19 | 14 | Dual-arm; STL meshes |

### Infrastructure

- GitHub Actions workflows (`build.yml`, `release.yml`) for CI builds and NPM publishing.
- `package.json`: main entry `build/pinocchio.js`, published files include WASM binary and TypeScript declarations.
- `LICENSE`: BSD-2-Clause.

---

[1.3.0]: https://github.com/Mostafasaad1/pinocchio-js/compare/v1.2.2...v1.3.0
[1.2.2]: https://github.com/Mostafasaad1/pinocchio-js/compare/v1.2.1...v1.2.2
[1.2.1]: https://github.com/Mostafasaad1/pinocchio-js/compare/v1.2.0...v1.2.1
[1.2.0]: https://github.com/Mostafasaad1/pinocchio-js/compare/v1.1.0...v1.2.0
[1.1.0]: https://github.com/Mostafasaad1/pinocchio-js/compare/v1.0.4...v1.1.0
[1.0.4]: https://github.com/Mostafasaad1/pinocchio-js/compare/v1.0.2...v1.0.4
[1.0.2]: https://github.com/Mostafasaad1/pinocchio-js/compare/v1.0.0...v1.0.2
[1.0.0]: https://github.com/Mostafasaad1/pinocchio-js/releases/tag/v1.0.0
