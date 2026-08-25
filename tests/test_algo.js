/**
 * Pinocchio WASM — Algorithm Tests
 */

module.exports = {
    run: async (ctx) => {
        const { pin, assert, assertClose, assertVecClose } = ctx;
        let passed = 0;
        let failed = 0;

        function check(cond) {
            if (cond) passed++; else failed++;
        }

        console.log('  --- Setup: 2-Link Arm ---');
        // Create a simple 2-link arm (RX, RY) with known inertia
        const model = new pin.Model();
        const id = pin.SE3.identity();
        const inertia = pin.Inertia.fromMassComInertia(1.0, [0, 0, 0.5], [0.1, 0, 0, 0.1, 0, 0.01]);

        // Joint 1: RX
        const j1 = pin.addJoint(model, 0, pin.JointModelRX(), id, "j1");
        pin.appendBodyToJoint(model, j1, inertia, id);

        // Joint 2: RY (offset by 1m in Z)
        const se3_2 = pin.SE3.fromXyzRpy(0, 0, 1.0, 0, 0, 0);
        const j2 = pin.addJoint(model, j1, pin.JointModelRY(), se3_2, "j2");
        pin.appendBodyToJoint(model, j2, inertia, id);

        const data = new pin.Data(model);
        check(assert(model.nq === 2 && model.nv === 2, 'Model created (nq=2)'));

        // 1. Neutral Config
        console.log('  --- Neutral Configuration ---');
        try {
            const q0 = pin.neutralConfiguration(model);
            check(assertVecClose(q0, [0, 0], 1e-9, 'Neutral is zero'));
        } catch (e) { console.error(e); failed++; }

        // 2. RNEA (Gravity only)
        console.log('  --- RNEA (Gravity) ---');
        try {
            const q = new Float64Array([0, 0]);
            const v = new Float64Array([0, 0]);
            const a = new Float64Array([0, 0]);
            const tau = pin.rnea(model, data, q, v, a);

            // J1 (RX): CoM1 is at (0,0,0.5). CoM2 is at (0,0,1.5).
            // Gravity is -9.81 Z. 
            // J1 axis is X. 
            // Torque = cross(r, mg).
            // CoM1: r=(0,0,0.5), F=(0,0,-9.81). cross(r,F) = (0, 4.9, 0). X-torque = 0.
            // CoM2: r=(0,0,1.5), F=(0,0,-9.81). cross(r,F) = (0, 14.7, 0). X-torque = 0.
            // Wait, is gravity in Z?
            // Standard Pinocchio gravity is usually -9.81 Z? Or need to set it?
            // RNEA defaults to model.gravity.
            // Let's assume standard gravity.
            // RX rotates around X. Gravity along Z. CoM along Z.
            // No moment arm for gravity around X. So Tau1 should be 0.
            // RY rotates around Y.
            // CoMs are on Z axis. Gravity along Z.
            // No moment arm for gravity around Y. So Tau2 should be 0.

            // Let's change configuration to get torque.
            // q = [PI/2, 0].
            // J1 rotated 90 deg around X.
            // Z axis becomes -Y axis.
            // CoM1 at (0, -0.5, 0). CoM2 at (0, -1.5, 0).
            // Gravity -Z.
            // J1 (X-axis). F=(0,0,-mg). r=(0,-0.5,0). cross(r,F) = (-(-0.5)*(-mg), 0, 0) ?
            // i j k
            // 0 -0.5 0
            // 0 0 -g
            // i(0.5g) - j(0) + k(0).
            // Torque X = 0.5 * 1.0 * 9.81 = 4.905.

            const q_pose = new Float64Array([Math.PI / 2, 0]);
            const tau_pose = pin.rnea(model, data, q_pose, v, a);

            // We expect tau[0] approx 4.905 + 14.715 (link 2) ?
            // Link 2 is at 1.0 local Z (now -Y). plus 0.5 CoM. -> -1.5 Y.
            // Torque 2 = 1.5 * 1.0 * 9.81 = 14.715.
            // Total Tau1 = 4.905 + 14.715 = 19.62.

            // Let's verify non-zero at least.
            check(assert(Math.abs(tau_pose[0]) > 1.0, `RNEA torque non-zero (${tau_pose[0].toFixed(2)})`));

        } catch (e) { console.error(e); failed++; }

        // 3. Center of Mass
        console.log('  --- Center of Mass ---');
        try {
            const q = new Float64Array([0, 0]);
            const com = pin.centerOfMass(model, data, q);
            // CoM1 at 0.5. Mass 1.
            // CoM2 at 1.5. Mass 1.
            // Total Mass 2.
            // CoM Z = (0.5*1 + 1.5*1) / 2 = 1.0.

            check(assertClose(com[2], 1.0, 1e-6, `CoM Z is 1.0`));

        } catch (e) { console.error(e); failed++; }

        // 4. Center of Mass Jacobian
        console.log('  --- Center of Mass Jacobian ---');
        try {
            const q0 = new Float64Array([0, 0]);
            const Jcom0 = pin.centerOfMassJacobian(model, data, q0);
            check(assert(Jcom0.length === 3 * model.nv, `CoM Jacobian size is 3x${model.nv} (${3 * model.nv})`));
            
            // Analytical verification at neutral config
            // Col 0: [0, -1.0, 0], Col 1: [0.25, 0, 0]
            const expectedJcom0 = [0.0, -1.0, 0.0, 0.25, 0.0, 0.0];
            check(assertVecClose(Jcom0, expectedJcom0, 1e-6, 'CoM Jacobian matches analytical values at neutral config'));

            // Finite differences verification at non-zero configuration
            const q = new Float64Array([0.3, -0.5]);
            const v = new Float64Array([0.4, -0.2]);
            const Jcom = pin.centerOfMassJacobian(model, data, q);

            const eps = 1e-7;
            const q_plus = new Float64Array([q[0] + eps * v[0], q[1] + eps * v[1]]);
            const q_minus = new Float64Array([q[0] - eps * v[0], q[1] - eps * v[1]]);
            const com_plus = pin.centerOfMass(model, data, q_plus);
            const com_minus = pin.centerOfMass(model, data, q_minus);

            const v_com_fd = [
                (com_plus[0] - com_minus[0]) / (2 * eps),
                (com_plus[1] - com_minus[1]) / (2 * eps),
                (com_plus[2] - com_minus[2]) / (2 * eps)
            ];

            const v_com_analytic = [
                Jcom[0] * v[0] + Jcom[3] * v[1],
                Jcom[1] * v[0] + Jcom[4] * v[1],
                Jcom[2] * v[0] + Jcom[5] * v[1]
            ];

            check(assertVecClose(v_com_analytic, v_com_fd, 1e-5, 'CoM Jacobian * v matches finite-difference d(com)/dt'));
        } catch (e) { console.error(e); failed++; }

        // 5. Jacobian
        console.log('  --- Jacobian ---');
        try {
            const q = new Float64Array([0, 0]);
            pin.computeJointJacobians(model, data, q);
            const J = pin.getJointJacobian(model, data, 2, pin.ReferenceFrame.LOCAL); // Joint 2
            // J is 6x2 flat array.
            // J should allow motion in Y (DoF 2) and X (DoF 1, rotated).

            check(assert(J.length === 12, 'Jacobian size 6x2 (12)'));
        } catch (e) { console.error(e); failed++; }

        // 6. Pre-allocated Buffer Tests (US3)
        console.log('  --- Pre-allocated Output Buffer Tests (US3) ---');
        try {
            const q = new Float64Array([0.1, 0.2]);
            const v = new Float64Array([0.3, 0.4]);
            const a = new Float64Array([0.5, 0.6]);
            const tau = new Float64Array([0.7, 0.8]);

            // RNEA pre-allocated
            const outTau = new Float64Array(model.nv);
            const resRnea = pin.rnea(model, data, q, v, a, outTau);
            check(assert(resRnea === outTau, 'rnea writes to and returns provided outArray'));
            const expectedTau = pin.rnea(model, data, q, v, a);
            check(assertVecClose(outTau, expectedTau, 1e-12, 'rnea outArray matches standard result'));

            // ABA pre-allocated
            const outDdq = new Float64Array(model.nv);
            const resAba = pin.aba(model, data, q, v, tau, outDdq);
            check(assert(resAba === outDdq, 'aba writes to and returns provided outArray'));
            const expectedDdq = pin.aba(model, data, q, v, tau);
            check(assertVecClose(outDdq, expectedDdq, 1e-12, 'aba outArray matches standard result'));

            // CRBA pre-allocated
            const outM = new Float64Array(model.nv * model.nv);
            const resCrba = pin.crba(model, data, q, outM);
            check(assert(resCrba === outM, 'crba writes to and returns provided outArray'));
            const expectedM = pin.crba(model, data, q);
            check(assertVecClose(outM, expectedM, 1e-12, 'crba outArray matches standard result'));

            // CoM pre-allocated
            const outCom = new Float64Array(3);
            const resCom = pin.centerOfMass(model, data, q, outCom);
            check(assert(resCom === outCom, 'centerOfMass writes to and returns provided outArray'));
            const expectedCom = pin.centerOfMass(model, data, q);
            check(assertVecClose(outCom, expectedCom, 1e-12, 'centerOfMass outArray matches standard result'));

            // CoM Jacobian pre-allocated
            const outJcom = new Float64Array(3 * model.nv);
            const resJcom = pin.centerOfMassJacobian(model, data, q, outJcom);
            check(assert(resJcom === outJcom, 'centerOfMassJacobian writes to and returns provided outArray'));
            const expectedJcom = pin.centerOfMassJacobian(model, data, q);
            check(assertVecClose(outJcom, expectedJcom, 1e-12, 'centerOfMassJacobian outArray matches standard result'));

            // Generalized Gravity pre-allocated
            const outG = new Float64Array(model.nv);
            const resG = pin.computeGeneralizedGravity(model, data, q, outG);
            check(assert(resG === outG, 'computeGeneralizedGravity writes to and returns provided outArray'));
            const expectedG = pin.computeGeneralizedGravity(model, data, q);
            check(assertVecClose(outG, expectedG, 1e-12, 'computeGeneralizedGravity outArray matches standard result'));

            // Non-linear Effects pre-allocated
            const outNle = new Float64Array(model.nv);
            const resNle = pin.nonLinearEffects(model, data, q, v, outNle);
            check(assert(resNle === outNle, 'nonLinearEffects writes to and returns provided outArray'));
            const expectedNle = pin.nonLinearEffects(model, data, q, v);
            check(assertVecClose(outNle, expectedNle, 1e-12, 'nonLinearEffects outArray matches standard result'));

            // Joint Jacobian pre-allocated
            pin.computeJointJacobians(model, data, q);
            const outJ = new Float64Array(6 * model.nv);
            const resJ = pin.getJointJacobian(model, data, 2, pin.ReferenceFrame.LOCAL, outJ);
            check(assert(resJ === outJ, 'getJointJacobian writes to and returns provided outArray'));
            const expectedJ = pin.getJointJacobian(model, data, 2, pin.ReferenceFrame.LOCAL);
            check(assertVecClose(outJ, expectedJ, 1e-12, 'getJointJacobian outArray matches standard result'));

        } catch (e) { console.error(e); failed++; }

        // ─── Joint Configuration Integration Tests (US1 & US2) ───
        console.log('  --- Joint Configuration Integration (integrate) ---');
        try {
            // US1: Euclidean joint integration (allocating)
            const q_euc = new Float64Array([0.2, -0.5]);
            const v_euc = new Float64Array([0.1, 0.3]);
            const q_next_euc = pin.integrate(model, q_euc, v_euc);
            check(assert(q_next_euc.length === 2, 'integrate returns array of length model.nq'));
            check(assertVecClose(q_next_euc, [0.3, -0.2], 1e-12, 'Euclidean integrate matches q + v'));

            // US1: Euclidean joint integration (in-place)
            const outQ_euc = new Float64Array(model.nq);
            const resQ_euc = pin.integrate(model, q_euc, v_euc, outQ_euc);
            check(assert(resQ_euc === outQ_euc, 'integrate writes to and returns provided outArray'));
            check(assertVecClose(outQ_euc, q_next_euc, 1e-12, 'in-place integrate matches allocating result'));

            // US1: Dimension validation checks
            let caughtQ = false;
            try {
                pin.integrate(model, new Float64Array([0]), v_euc);
            } catch (err) {
                caughtQ = true;
                check(assert(err.message.includes('Configuration vector q has invalid dimension'), 'Clear error on invalid q dimension'));
            }
            check(assert(caughtQ, 'Throws error when q dimension is invalid'));

            let caughtV = false;
            try {
                pin.integrate(model, q_euc, new Float64Array([0, 0, 0]));
            } catch (err) {
                caughtV = true;
                check(assert(err.message.includes('Velocity vector v has invalid dimension'), 'Clear error on invalid v dimension'));
            }
            check(assert(caughtV, 'Throws error when v dimension is invalid'));

            let caughtOut = false;
            try {
                pin.integrate(model, q_euc, v_euc, new Float64Array(5));
            } catch (err) {
                caughtOut = true;
                check(assert(err.message.includes('Output vector outArray has invalid dimension'), 'Clear error on invalid outArray dimension'));
            }
            check(assert(caughtOut, 'Throws error when outArray dimension is invalid'));

            // US2: Free-Flyer Non-Euclidean Integration (SE3 / SO3)
            const ffModel = new pin.Model();
            const j_ff = pin.addJoint(ffModel, 0, pin.JointModelFreeFlyer(), id, "freeflyer");
            pin.appendBodyToJoint(ffModel, j_ff, inertia, id);
            check(assert(ffModel.nq === 7 && ffModel.nv === 6, 'Free-flyer model created (nq=7, nv=6)'));

            const q0_ff = pin.neutralConfiguration(ffModel); // [0, 0, 0, 0, 0, 0, 1]
            check(assertVecClose(q0_ff, [0, 0, 0, 0, 0, 0, 1], 1e-12, 'Free-flyer neutral configuration'));

            // Pure translation
            const v_trans = new Float64Array([1.5, -2.0, 3.5, 0, 0, 0]);
            const q_trans = pin.integrate(ffModel, q0_ff, v_trans);
            check(assertVecClose(q_trans, [1.5, -2.0, 3.5, 0, 0, 0, 1], 1e-12, 'Free-flyer pure translation integrate'));

            // Pure rotation around X by 90 degrees
            const v_rot = new Float64Array([0, 0, 0, Math.PI / 2, 0, 0]);
            const q_rot = pin.integrate(ffModel, q0_ff, v_rot);
            const halfAngle = Math.PI / 4;
            const expectedQuatX = [0, 0, 0, Math.sin(halfAngle), 0, 0, Math.cos(halfAngle)];
            check(assertVecClose(q_rot, expectedQuatX, 1e-12, 'Free-flyer pure rotation integrate (90 deg around X)'));
            const rotQuatNorm = Math.hypot(q_rot[3], q_rot[4], q_rot[5], q_rot[6]);
            check(assertClose(rotQuatNorm, 1.0, 1e-12, 'Free-flyer rotation preserves unit quaternion norm'));

            // Combined motion with in-place buffer
            const v_comb = new Float64Array([0.1, -0.2, 0.3, 0.2, 0.4, -0.1]);
            const q_comb_alloc = pin.integrate(ffModel, q0_ff, v_comb);
            const out_comb = new Float64Array(7);
            const res_comb = pin.integrate(ffModel, q0_ff, v_comb, out_comb);
            check(assert(res_comb === out_comb, 'Free-flyer in-place returns provided buffer'));
            check(assertVecClose(out_comb, q_comb_alloc, 1e-12, 'Free-flyer in-place matches allocating'));
            const combQuatNorm = Math.hypot(out_comb[3], out_comb[4], out_comb[5], out_comb[6]);
            check(assertClose(combQuatNorm, 1.0, 1e-12, 'Free-flyer combined motion preserves unit quaternion norm'));

        } catch (e) { console.error(e); failed++; }

        return { passed, failed };
    }
};
