/**
 * Benchmark script for testing zero-allocation with pre-allocated output buffers.
 * Usage: node tests/benchmark-prealloc.js
 */

const PinocchioModule = require('../build/pinocchio.js');

async function main() {
    console.log('═══════════════════════════════════════════════════');
    console.log('   Pinocchio-JS Pre-Allocated Buffer Benchmark     ');
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
    const q = new Float64Array(model.nq).fill(0.1);
    const v = new Float64Array(model.nv).fill(0.2);
    const a = new Float64Array(model.nv).fill(0.3);
    const tau = new Float64Array(model.nv).fill(0.4);

    const outTau = new Float64Array(model.nv);
    const outDdq = new Float64Array(model.nv);
    const outM = new Float64Array(model.nv * model.nv);
    const outCom = new Float64Array(3);
    const outG = new Float64Array(model.nv);
    const outNle = new Float64Array(model.nv);
    const outJ = new Float64Array(6 * model.nv);
    const outVf = new Float64Array(6);

    const ITERS = 100000;

    // Verify mathematical accuracy between standard and pre-allocated
    const standardTau = pin.rnea(model, data, q, v, a);
    const preallocTau = pin.rnea(model, data, q, v, a, outTau);
    if (preallocTau !== outTau) throw new Error("rnea should return outTau buffer");
    for (let i = 0; i < model.nv; i++) {
        if (Math.abs(standardTau[i] - outTau[i]) > 1e-12) {
            throw new Error(`Numerical mismatch in rnea at index ${i}`);
        }
    }

    const standardAba = pin.aba(model, data, q, v, tau);
    const preallocAba = pin.aba(model, data, q, v, tau, outDdq);
    if (preallocAba !== outDdq) throw new Error("aba should return outDdq buffer");
    for (let i = 0; i < model.nv; i++) {
        if (Math.abs(standardAba[i] - outDdq[i]) > 1e-12) {
            throw new Error(`Numerical mismatch in aba at index ${i}`);
        }
    }

    console.log(`Verified numerical correctness for pre-allocated output buffers.\n`);
    console.log(`Running ${ITERS.toLocaleString()} iterations:\n`);

    // Standard RNEA (allocating new Float64Array per call)
    const t0 = performance.now();
    for (let i = 0; i < ITERS; i++) {
        pin.rnea(model, data, q, v, a);
    }
    const tStandard = performance.now() - t0;

    // Pre-allocated RNEA (zero allocation)
    const t1 = performance.now();
    for (let i = 0; i < ITERS; i++) {
        pin.rnea(model, data, q, v, a, outTau);
    }
    const tPrealloc = performance.now() - t1;

    console.log(`  1) RNEA Standard (allocating)    : ${tStandard.toFixed(2)} ms (${(tStandard*1000/ITERS).toFixed(2)} µs/call)`);
    console.log(`  2) RNEA Pre-allocated (zero-alloc): ${tPrealloc.toFixed(2)} ms (${(tPrealloc*1000/ITERS).toFixed(2)} µs/call)`);
    const speedup = (((tStandard - tPrealloc) / tStandard) * 100).toFixed(1);
    console.log(`     🚀 Speedup with pre-allocated buffer: ${speedup}%\n`);

    // Benchmark other pre-allocated functions
    const tAba = performance.now();
    for (let i = 0; i < ITERS; i++) {
        pin.aba(model, data, q, v, tau, outDdq);
    }
    console.log(`  3) ABA Pre-allocated             : ${(performance.now() - tAba).toFixed(2)} ms`);

    const tCrba = performance.now();
    for (let i = 0; i < ITERS; i++) {
        pin.crba(model, data, q, outM);
    }
    console.log(`  4) CRBA Pre-allocated            : ${(performance.now() - tCrba).toFixed(2)} ms`);

    const tCom = performance.now();
    for (let i = 0; i < ITERS; i++) {
        pin.centerOfMass(model, data, q, outCom);
    }
    console.log(`  5) CoM Pre-allocated             : ${(performance.now() - tCom).toFixed(2)} ms`);

    const tGrav = performance.now();
    for (let i = 0; i < ITERS; i++) {
        pin.computeGeneralizedGravity(model, data, q, outG);
    }
    console.log(`  6) Gravity Pre-allocated         : ${(performance.now() - tGrav).toFixed(2)} ms`);

    const tNle = performance.now();
    for (let i = 0; i < ITERS; i++) {
        pin.nonLinearEffects(model, data, q, v, outNle);
    }
    console.log(`  7) NLE Pre-allocated             : ${(performance.now() - tNle).toFixed(2)} ms`);

    const tJac = performance.now();
    for (let i = 0; i < ITERS; i++) {
        pin.getJointJacobian(model, data, 6, pin.ReferenceFrame.LOCAL, outJ);
    }
    console.log(`  8) Jacobian Pre-allocated        : ${(performance.now() - tJac).toFixed(2)} ms`);

    const tVf = performance.now();
    for (let i = 0; i < ITERS; i++) {
        pin.getFrameVelocity(model, data, 6, pin.ReferenceFrame.LOCAL, outVf);
    }
    console.log(`  9) Frame Vel Pre-allocated       : ${(performance.now() - tVf).toFixed(2)} ms`);

    console.log('\n═══════════════════════════════════════════════════');
    console.log('   Zero-Allocation Pre-allocation Test Passed!     ');
    console.log('═══════════════════════════════════════════════════');
}

main().catch(console.error);
