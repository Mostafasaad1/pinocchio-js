/**
 * Benchmark script for measuring Pinocchio-js algorithm performance.
 * Tests zero-copy JS <-> WASM data transfer.
 * Usage: node tests/benchmark_algorithms.js
 */

const path = require('path');
const PinocchioModule = require('../build/pinocchio.js');

async function runBenchmark() {
    console.log('═══════════════════════════════════════════════════');
    console.log('   Pinocchio-JS Algorithm Performance Benchmark    ');
    console.log('═══════════════════════════════════════════════════\n');

    const pin = await PinocchioModule();

    // Create a 6-DOF robot model
    const model = new pin.Model();
    let prevJoint = 0;
    for (let i = 0; i < 6; i++) {
        const placement = pin.SE3.fromXyzRpy(0.1 * i, 0, 0.2, 0, 0, 0);
        const joint = (i % 2 === 0) ? pin.JointModelRZ() : pin.JointModelRY();
        const jid = pin.addJoint(model, prevJoint, joint, placement, `joint_${i+1}`);
        const inertia = pin.Inertia.fromMassComInertia(
            1.0,
            new Float64Array([0, 0, 0.1]),
            new Float64Array([0.01, 0, 0, 0.01, 0, 0.01])
        );
        pin.appendBodyToJoint(model, jid, inertia, pin.SE3.identity());
        prevJoint = jid;
    }

    const data = new pin.Data(model);
    const nq = model.nq;
    const nv = model.nv;

    const q = new Float64Array(nq).fill(0.1);
    const v = new Float64Array(nv).fill(0.2);
    const a = new Float64Array(nv).fill(0.3);
    const tau = new Float64Array(nv).fill(0.4);

    const WARMUP_ITERS = 1000;
    const BENCH_ITERS = 20000;

    // Warmup
    for (let i = 0; i < WARMUP_ITERS; i++) {
        pin.rnea(model, data, q, v, a);
        pin.aba(model, data, q, v, tau);
        pin.crba(model, data, q);
    }

    const benchmarks = [
        { name: 'RNEA (Inverse Dynamics)', fn: () => pin.rnea(model, data, q, v, a) },
        { name: 'ABA (Forward Dynamics)', fn: () => pin.aba(model, data, q, v, tau) },
        { name: 'CRBA (Mass Matrix)', fn: () => pin.crba(model, data, q) },
        { name: 'Center of Mass', fn: () => pin.centerOfMass(model, data, q) },
        { name: 'Generalized Gravity', fn: () => pin.computeGeneralizedGravity(model, data, q) },
        { name: 'Nonlinear Effects (Cor/Cen/Grav)', fn: () => pin.nonLinearEffects(model, data, q, v) },
        { name: 'Joint Jacobian', fn: () => pin.getJointJacobian(model, data, 6, pin.ReferenceFrame.LOCAL) },
        { name: 'Frame Velocity', fn: () => pin.getFrameVelocity(model, data, 6, pin.ReferenceFrame.LOCAL) }
    ];

    console.log(`Running ${BENCH_ITERS.toLocaleString()} iterations per algorithm (6-DOF robot):\n`);

    for (const b of benchmarks) {
        const start = performance.now();
        for (let i = 0; i < BENCH_ITERS; i++) {
            b.fn();
        }
        const totalMs = performance.now() - start;
        const perCallUs = (totalMs * 1000 / BENCH_ITERS).toFixed(2);
        const perCallMs = (totalMs / BENCH_ITERS).toFixed(5);
        const callsPerSec = Math.round(BENCH_ITERS / (totalMs / 1000)).toLocaleString();

        console.log(`  ⚡ ${b.name.padEnd(36)} : ${perCallUs.padStart(8)} µs/call (${perCallMs} ms) | ${callsPerSec.padStart(10)} ops/sec`);
    }

    console.log('\n═══════════════════════════════════════════════════');
    console.log('   Benchmark Completed Successfully!               ');
    console.log('═══════════════════════════════════════════════════');
}

runBenchmark().catch(console.error);
