# Pinocchio WASM

Pinocchio WASM is a port of the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) rigid body dynamics library to WebAssembly, enabling high-performance robot dynamics in the browser and Node.js.

## Features

-   **Core Algorithms:** RNEA (Inverse Dynamics), Forward Kinematics, Center of Mass, Jacobian.
-   **URDF Parsing:** Custom JavaScript parser ensuring compatibility with Pinocchio's model structure.
-   **Cross-Platform:** Runs in Browsers (Chrome, Firefox, Safari) and Node.js.
-   **Lightweight:** ~600KB (gzipped) WASM binary.
-   **Zero-Dependency Runtime:** No external logical dependencies for the basic WASM module (excluding the optional URDF parser).

## Installation

### 1. NPM (Recommended)

The easiest way to use the library in your Node.js or browser projects is to install the pre-compiled NPM package:

```bash
npm install pinocchio-js
```

### 2. GitHub Releases

You can download the pre-compiled WebAssembly binaries (`pinocchio.js` and `pinocchio.wasm`) directly from the [Releases page](https://github.com/Mostafasaad1/pinocchio-js/releases).

### 3. Building from Source

#### Prerequisites

1.  **Emscripten SDK (emsdk):** Required for compiling C++ to WASM.
    ```bash
    git clone https://github.com/emscripten-core/emsdk.git
    cd emsdk
    ./emsdk install latest
    ./emsdk activate latest
    source ./emsdk_env.sh
    ```
2.  **CMake:** Version 3.10 or higher.
3.  **Python 3:** For build scripts.

## Building from Source

We provide a one-step build script `build.sh`.

```bash
./build.sh
```

This will:
1.  Download dependencies (Pinocchio, Eigen3, Boost headers) automatically.
2.  Configure the project with `emcmake`.
3.  Compile to `build/pinocchio.js` and `build/pinocchio.wasm`.

## Usage

### 1. Browser

Serve the project root:
```bash
python3 -m http.server 8080
```
Open `http://localhost:8080/tests/browser/index.html`.

### 2. Node.js

Install the XML DOM polyfill (required for URDF parsing in Node):
```bash
npm install
```

Run the smoke test:
```bash
npm run test:smoke
```

## API Reference

### `pin.Model`
-   `nq`, `nv`, `njoints`: System dimensions.
-   `addJoint(...)`: Adds a joint (Internal use).

### `pin.Data`
-   Constructed from `Model`.
-   Stores scratchpad data (velocities, accelerations, forces).

### Algorithms
-   `pin.rnea(model, data, q, v, a)`: Recursive Newton-Euler Algorithm (Inverse Dynamics). Returns torques.
-   `pin.forwardKinematics(model, data, q)`: Updates joint placements.
-   `pin.centerOfMass(model, data, q)`: Computes CoM position.
-   `pin.computeTotalMass(model)`: Returns total mass.

### URDF Parser (`src/urdf-parser.mjs`)
-   `parseURDF(xmlString)`: Parses raw XML into intermediate JS object.
-   `buildPinocchioModel(pin, urdfData)`: Converts intermediate object to `pin.Model`.
    *   **Note:** Implements "Fixed Joint Reduction". Fixed joints in URDF are fused into their parent joints to ensure numerical stability and correct behavior in Pinocchio.

## Testing

Run the full test suite (Node.js required):

```bash
npm test
```

This runs:
-   Math tests (SE3, Inertia)
-   Model tests (Joint creation)
-   Algo tests (RNEA, COM)
-   URDF tests (Loading real robot descriptions)

## Contributing

1.  Fork the repository.
2.  Run `npm install` and `./build.sh` to ensure clean environment.
3.  Add tests for new features in `tests/`.
4.  Submit a Pull Request.

## License

BSD-2-Clause
