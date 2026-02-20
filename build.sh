#!/bin/bash
# ──────────────────────────────────────────────
# Pinocchio WASM — Build Script
# ──────────────────────────────────────────────
# Prerequisites:
#   - Emscripten SDK (emsdk) activated in your shell
#   - Eigen3 headers installed (or provide -DEIGEN3_INCLUDE_DIR=...)
#   - Boost headers installed (or provide -DBoost_INCLUDE_DIR=...)
#   - Pinocchio source tree (default: parent directory)
#
# Usage:
#   ./build.sh                    # Release build (fast)
#   ./build.sh MinSizeRel         # Smallest binary
#   ./build.sh Debug              # Debug build
# ──────────────────────────────────────────────

set -e

BUILD_TYPE="${1:-Release}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "╔═══════════════════════════════════════╗"
echo "║   Pinocchio WASM Build                ║"
echo "║   Build type: ${BUILD_TYPE}                 ║"
echo "╚═══════════════════════════════════════╝"

# Check for emscripten
if ! command -v emcmake &> /dev/null; then
    echo "❌ Error: Emscripten (emcmake) not found."
    echo "   Install: https://emscripten.org/docs/getting_started/downloads.html"
    echo "   Then run: source emsdk/emsdk_env.sh"
    exit 1
fi

# Create build directory
mkdir -p "${SCRIPT_DIR}/build"
cd "${SCRIPT_DIR}/build"

if [ -f "${SCRIPT_DIR}/build/CMakeCache.txt" ]; then
    echo "⚡ Cache found in build/, skipping configuration."
    echo "   (To force reconfigure: rm -rf build/)"
else
    echo "📋 Configuring with Emscripten..."
    # Determine Pinocchio source directory path
    PINOCCHIO_SOURCE_DIR="${SCRIPT_DIR}/../pinocchio"
    if [ ! -d "${PINOCCHIO_SOURCE_DIR}/include" ]; then
        echo "❌ Error: Pinocchio source directory not found at ${PINOCCHIO_SOURCE_DIR}"
        echo "   Expected structure: ../pinocchio/include/pinocchio/"
        echo "   You can override with: export PINOCCHIO_SOURCE_DIR=/path/to/pinocchio"
        exit 1
    fi
    echo "🎯 Using Pinocchio source: ${PINOCCHIO_SOURCE_DIR}"
    emcmake cmake "${SCRIPT_DIR}" \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DPINOCCHIO_SOURCE_DIR="${PINOCCHIO_SOURCE_DIR}" \
        -DCMAKE_CXX_FLAGS="-I${SCRIPT_DIR}/emsdk/upstream/emscripten/cache/sysroot/include"
    echo "✅ Configuration complete"
fi

echo "📦 Step 2/4: Building (this may take 5-30 minutes depending on your system)..."
cmake --build . --config "${BUILD_TYPE}" --parallel $(nproc 2>/dev/null || echo 4) --verbose 2>&1 | tee build.log
echo "✅ Build complete"

# Report results
echo ""
echo "✅ Build complete!"
echo ""
if [ -f "pinocchio.wasm" ]; then
    WASM_SIZE=$(wc -c < pinocchio.wasm)
    WASM_KB=$((WASM_SIZE / 1024))
    echo "📦 pinocchio.wasm: ${WASM_KB} KB (${WASM_SIZE} bytes)"
fi
if [ -f "pinocchio.js" ]; then
    JS_SIZE=$(wc -c < pinocchio.js)
    JS_KB=$((JS_SIZE / 1024))
    echo "📦 pinocchio.js:   ${JS_KB} KB (${JS_SIZE} bytes)"
fi
echo ""
echo "To test in browser:"
echo "  cd build && python3 -m http.server 8080"
echo "  Open: http://localhost:8080/test.html"
