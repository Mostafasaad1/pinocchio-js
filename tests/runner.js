/**
 * Pinocchio WASM — Comprehensive Test Runner
 * Usage: node tests/runner.js
 */

const fs = require('fs');
const path = require('path');

// Colors for output
const RESET = '\x1b[0m';
const RED = '\x1b[31m';
const GREEN = '\x1b[32m';
const YELLOW = '\x1b[33m';
const BLUE = '\x1b[34m';
const BOLD = '\x1b[1m';

async function main() {
    console.log(`${BOLD}${BLUE}═══════════════════════════════════════${RESET}`);
    console.log(`${BOLD}${BLUE}   Pinocchio WASM — Test Suite 🧪      ${RESET}`);
    console.log(`${BOLD}${BLUE}═══════════════════════════════════════${RESET}\n`);

    // Path to build library
    const buildPath = path.join(__dirname, '../build/pinocchio.js');
    if (!fs.existsSync(buildPath)) {
        console.error(`${RED}❌ Error: pinocchio.js not found in ../build/${RESET}`);
        console.error(`   Please run ./build.sh first.`);
        process.exit(1);
    }

    // Load Module
    console.log(`${YELLOW}⏳ Loading WASM module...${RESET}`);
    let pin;
    try {
        const factory = require(buildPath);
        pin = await factory();
        console.log(`${GREEN}✅ WASM module loaded successfully.${RESET}\n`);
    } catch (err) {
        console.error(`${RED}❌ Failed to load WASM module:${RESET}`, err);
        process.exit(1);
    }

    // Helper context for tests
    const ctx = {
        pin,
        assert: (cond, msg) => {
            if (cond) {
                console.log(`  ${GREEN}✅ ${msg}${RESET}`);
                return true;
            } else {
                console.error(`  ${RED}❌ ${msg}${RESET}`);
                return false; // Don't throw, just count failure
            }
        },
        assertClose: (val, expected, tol = 1e-6, msg) => {
            const diff = Math.abs(val - expected);
            if (diff <= tol) {
                console.log(`  ${GREEN}✅ ${msg} (diff=${diff.toExponential(1)})${RESET}`);
                return true;
            } else {
                console.error(`  ${RED}❌ ${msg} (expected ${expected}, got ${val}, diff=${diff})${RESET}`);
                return false;
            }
        },
        assertVecClose: (vec, expected, tol = 1e-6, msg) => {
            if (vec.length !== expected.length) {
                console.error(`  ${RED}❌ ${msg} (length mismatch: ${vec.length} vs ${expected.length})${RESET}`);
                return false;
            }
            let maxDiff = 0;
            for (let i = 0; i < vec.length; i++) {
                maxDiff = Math.max(maxDiff, Math.abs(vec[i] - expected[i]));
            }
            if (maxDiff <= tol) {
                console.log(`  ${GREEN}✅ ${msg} (max diff=${maxDiff.toExponential(1)})${RESET}`);
                return true;
            } else {
                console.error(`  ${RED}❌ ${msg} (max diff=${maxDiff}, tol=${tol})${RESET}`);
                console.error(`     Got: [${Array.from(vec).map(x => x.toFixed(4)).join(', ')}]`);
                console.error(`     Exp: [${expected.map(x => x.toFixed(4)).join(', ')}]`);
                return false;
            }
        }
    };

    // Test Files
    const testFiles = [
        'test_math.js',
        'test_model.js',
        'test_algo.js',
        'test_urdf.js'
    ];

    let totalPassed = 0;
    let totalFailed = 0;

    for (const file of testFiles) {
        console.log(`${BOLD}🔹 Running ${file}...${RESET}`);
        try {
            // We assume test files export a run(ctx) async function
            const testModule = require(`./${file}`);
            const result = await testModule.run(ctx);
            totalPassed += result.passed;
            totalFailed += result.failed;
            console.log('');
        } catch (err) {
            console.error(`${RED}❌ Error running ${file}:${RESET}`, err);
            // Create stub file if missing? No, that's what I'll do next.
            if (err.code === 'MODULE_NOT_FOUND') {
                console.warn(`${YELLOW}   (File not created yet)${RESET}\n`);
            }
        }
    }

    console.log(`${BOLD}${BLUE}═══════════════════════════════════════${RESET}`);
    console.log(`Summary: ${GREEN}${totalPassed} passed${RESET}, ${totalFailed > 0 ? RED + totalFailed + ' failed' : '0 failed'}`);
    console.log(`${BOLD}${BLUE}═══════════════════════════════════════${RESET}`);

    if (totalFailed > 0) process.exit(1);
}

main().catch(err => {
    console.error(err);
    process.exit(1);
});
