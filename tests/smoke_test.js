/**
 * Pinocchio WASM — Node.js Smoke Test
 *
 * Usage:
 *   cd tests
 *   node smoke_test.js
 */

const path = require('path');

async function main() {
    console.log('═══════════════════════════════════════');
    console.log(' Pinocchio WASM — Smoke Test (Node.js)');
    console.log('═══════════════════════════════════════\n');

    let passed = 0;
    let failed = 0;

    function assert(condition, name) {
        if (condition) {
            console.log(`  ✅ ${name}`);
            passed++;
        } else {
            console.log(`  ❌ ${name}`);
            failed++;
        }
    }

    // Load the WASM module
    // Path reduced by one level relative to root, but increase by one since we are in tests/
    const modulePath = path.join(__dirname, '../build', 'pinocchio.js');
    try {
        const PinocchioModule = require(modulePath);
        var pin = await PinocchioModule();
    } catch (e) {
        console.error("Failed to load WASM module from: " + modulePath);
        throw e;
    }

    console.log('Module loaded.\n');

    // ── Test 1: Model creation ──
    console.log('Test 1: Model Creation');
    {
        const model = new pin.Model();
        assert(model.nq === 0, 'Empty model nq = 0');
        assert(model.nv === 0, 'Empty model nv = 0');
        assert(model.njoints === 1, 'Empty model has universe joint (njoints = 1)');
    }

    // ── Test 2: Add joints ──
    console.log('\nTest 2: Add Joints');
    {
        const model = new pin.Model();
        const se3 = pin.SE3.fromXyzRpy(0, 0, 0.5, 0, 0, 0);
        const inertia = pin.Inertia.fromMassComInertia(
            1.0, [0, 0, 0.2], [0.01, 0, 0, 0.01, 0, 0.005]
        );

        const j1 = pin.addJoint(model, 0, pin.JointModelRX(), se3, 'j1');
        pin.appendBodyToJoint(model, j1, inertia, pin.SE3.identity());

        const j2 = pin.addJoint(model, j1, pin.JointModelRY(), se3, 'j2');
        pin.appendBodyToJoint(model, j2, inertia, pin.SE3.identity());

        assert(model.nq === 2, 'Two revolute joints → nq = 2');
        assert(model.nv === 2, 'Two revolute joints → nv = 2');
        assert(model.njoints === 3, 'Universe + 2 joints → njoints = 3');
    }

    // ── Test 3: RNEA ──
    console.log('\nTest 3: Inverse Dynamics (RNEA)');
    {
        const model = new pin.Model();
        const se3 = pin.SE3.fromXyzRpy(0, 0, 0.5, 0, 0, 0);
        const inertia = pin.Inertia.fromMassComInertia(
            1.0, [0, 0, 0.2], [0.01, 0, 0, 0.01, 0, 0.005]
        );

        const j1 = pin.addJoint(model, 0, pin.JointModelRZ(), se3, 'j1');
        pin.appendBodyToJoint(model, j1, inertia, pin.SE3.identity());

        const data = new pin.Data(model);

        // Zero configuration, zero velocity, zero acceleration
        // Should return gravity compensation torques
        const q = new Float64Array([0.0]);
        const v = new Float64Array([0.0]);
        const a = new Float64Array([0.0]);

        const tau = pin.rnea(model, data, q, v, a);
        assert(tau.length === 1, 'RNEA returns 1 torque value');
        // Revolute Z with gravity along -Z: gravity torque should be non-zero
        // if COM has an offset from the joint axis
        assert(typeof tau[0] === 'number' && !isNaN(tau[0]), 'Torque is a valid number');
        console.log(`    τ = [${tau[0].toFixed(6)}]`);
    }

    // ── Test 4: Center of Mass ──
    console.log('\nTest 4: Center of Mass');
    {
        const model = new pin.Model();
        const se3 = pin.SE3.fromXyzRpy(0, 0, 1.0, 0, 0, 0);
        const inertia = pin.Inertia.fromMassComInertia(
            5.0, [0, 0, 0.5], [0.1, 0, 0, 0.1, 0, 0.05]
        );

        const j1 = pin.addJoint(model, 0, pin.JointModelRZ(), se3, 'j1');
        pin.appendBodyToJoint(model, j1, inertia, pin.SE3.identity());

        const data = new pin.Data(model);
        const q = new Float64Array([0.0]);

        const com = pin.centerOfMass(model, data, q);
        assert(com.length === 3, 'COM returns 3 values');
        assert(com[2] > 0, 'COM Z is positive (above ground)');
        console.log(`    COM = [${Array.from(com).map(x => x.toFixed(4)).join(', ')}]`);
    }

    // ── Test 5: Performance ──
    console.log('\nTest 5: Performance');
    {
        const model = new pin.Model();
        const se3 = pin.SE3.fromXyzRpy(0, 0, 0.3, 0, 0, 0);
        const inertia = pin.Inertia.fromMassComInertia(
            0.5, [0, 0, 0.1], [0.005, 0, 0, 0.005, 0, 0.003]
        );

        // Build a 7-DOF arm
        let parent = 0;
        const jointTypes = [
            pin.JointModelRZ, pin.JointModelRY, pin.JointModelRX,
            pin.JointModelRZ, pin.JointModelRY, pin.JointModelRX,
            pin.JointModelRZ
        ];
        for (let i = 0; i < 7; i++) {
            parent = pin.addJoint(model, parent, jointTypes[i](), se3, `j${i + 1}`);
            pin.appendBodyToJoint(model, parent, inertia, pin.SE3.identity());
        }

        const data = new pin.Data(model);
        const q = new Float64Array(model.nq).fill(0);
        const v = new Float64Array(model.nv).fill(0);
        const a = new Float64Array(model.nv).fill(0);

        // Warmup
        for (let i = 0; i < 100; i++) pin.rnea(model, data, q, v, a);

        const ITERS = 100000;
        const t0 = performance.now();
        for (let i = 0; i < ITERS; i++) {
            pin.rnea(model, data, q, v, a);
        }
        const dt = performance.now() - t0;
        const usPerCall = (dt / ITERS) * 1000;

        console.log(`    7-DOF RNEA: ${ITERS} calls in ${dt.toFixed(1)}ms`);
        console.log(`    → ${usPerCall.toFixed(2)} µs/call`);
        assert(usPerCall < 1000, 'RNEA < 1ms per call');
    }

    // ── Summary ──
    console.log('\n═══════════════════════════════════════');
    console.log(` Results: ${passed} passed, ${failed} failed`);
    console.log('═══════════════════════════════════════\n');

    process.exit(failed > 0 ? 1 : 0);
}

main().catch(err => {
    console.error('Fatal error:', err);
    process.exit(1);
});
