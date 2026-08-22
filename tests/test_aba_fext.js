/**
 * Pinocchio WASM — ABA with External Forces Tests
 * Feature: 004-aba-external-forces
 */

module.exports = {
    run: async (ctx) => {
        const { pin, assert, assertClose, assertVecClose } = ctx;
        let passed = 0;
        let failed = 0;

        function check(cond) {
            if (cond) passed++; else failed++;
        }

        console.log('  --- Setup: Models for ABA External Forces ---');
        const id = pin.SE3.identity();
        const inertia = pin.Inertia.fromMassComInertia(1.0, [0, 0, 0.5], [0.1, 0, 0, 0.1, 0, 0.01]);

        // 1-DOF RX Pendulum
        const model1Dof = new pin.Model();
        const j1_rx = pin.addJoint(model1Dof, 0, pin.JointModelRX(), id, "j1");
        pin.appendBodyToJoint(model1Dof, j1_rx, inertia, id);
        const data1Dof = new pin.Data(model1Dof);
        const data1DofBase = new pin.Data(model1Dof);

        check(assert(model1Dof.nq === 1 && model1Dof.nv === 1, '1-DOF Pendulum model created'));

        // 2-Link Arm (RX, RY)
        const model2Link = new pin.Model();
        const j1 = pin.addJoint(model2Link, 0, pin.JointModelRX(), id, "j1");
        pin.appendBodyToJoint(model2Link, j1, inertia, id);

        const se3_2 = pin.SE3.fromXyzRpy(0, 0, 1.0, 0, 0, 0);
        const j2 = pin.addJoint(model2Link, j1, pin.JointModelRY(), se3_2, "j2");
        pin.appendBodyToJoint(model2Link, j2, inertia, id);

        const data2Link = new pin.Data(model2Link);
        const data2LinkBase = new pin.Data(model2Link);

        check(assert(model2Link.nq === 2 && model2Link.nv === 2, '2-Link Arm model created'));

        // ─────────────────────────────────────────────────────────────
        // Phase 3 / User Story 1: Simulate Contact Wrench During Motion
        // ─────────────────────────────────────────────────────────────
        console.log('  --- US1: Backward Compatibility & Zero-Wrench (SC-001 / FR-003) ---');
        try {
            const q = new Float64Array([Math.PI / 4]);
            const v = new Float64Array([0.5]);
            const tau = new Float64Array([1.0]);

            const ddq_base = pin.aba(model1Dof, data1DofBase, q, v, tau);
            const ddq_empty_map = pin.abaWithForces(model1Dof, data1Dof, q, v, tau, {});
            const ddq_no_arg = pin.abaWithForces(model1Dof, data1Dof, q, v, tau);
            const ddq_zero_wrench = pin.abaWithForces(model1Dof, data1Dof, q, v, tau, {
                1: new Float64Array(6)
            });

            check(assertClose(ddq_empty_map[0], ddq_base[0], 1e-10, 'Empty map matches baseline aba'));
            check(assertClose(ddq_no_arg[0], ddq_base[0], 1e-10, 'Omitted fext arg matches baseline aba'));
            check(assertClose(ddq_zero_wrench[0], ddq_base[0], 1e-10, 'Zero wrench matches baseline aba'));
        } catch (e) { console.error(e); failed++; }

        console.log('  --- US1: Known Wrench Analytical Verification (SC-002 / FR-002) ---');
        try {
            // 1-DOF pendulum at q = PI/2, v = 0, tau = 0
            // Inertia around joint X-axis: I_xx = 0.1 + mass * (com_z)^2 = 0.1 + 1.0 * 0.25 = 0.35 kg·m²
            // Applying local torque tx = 5.0 N·m at joint 1 produces delta_ddq = 5.0 / 0.35 = 14.285714285714286 rad/s²
            const q = new Float64Array([Math.PI / 2]);
            const v = new Float64Array([0]);
            const tau = new Float64Array([0]);

            const ddq_base = pin.aba(model1Dof, data1DofBase, q, v, tau);
            const wrench = new Float64Array([5.0, 0, 0, 0, 0, 0]); // [tx=5, ty=0, tz=0, fx=0, fy=0, fz=0]
            const ddq_loaded = pin.abaWithForces(model1Dof, data1Dof, q, v, tau, { 1: wrench });

            const delta_ddq = ddq_loaded[0] - ddq_base[0];
            const expected_delta = 5.0 / 0.35;

            check(assertClose(delta_ddq, expected_delta, 1e-10, `ddq shift matches analytical delta (${expected_delta.toFixed(6)})`));
        } catch (e) { console.error(e); failed++; }

        console.log('  --- US1: Root-Link Wrench Non-Throwing (FR-005) ---');
        try {
            const q = new Float64Array([0]);
            const v = new Float64Array([0]);
            const tau = new Float64Array([0]);

            const ddq_root = pin.abaWithForces(model1Dof, data1Dof, q, v, tau, {
                0: new Float64Array([1, 2, 3, 4, 5, 6])
            });

            check(assert(ddq_root instanceof Float64Array && ddq_root.length === 1, 'Root-link wrench completes without error'));
        } catch (e) { console.error(e); failed++; }

        // ─────────────────────────────────────────────────────────────
        // Phase 4 / User Story 2: Tool Payload & Multi-Link Accumulation
        // ─────────────────────────────────────────────────────────────
        console.log('  --- US2: Multi-Link Force Superposition (FR-006 / SC-006) ---');
        try {
            const q = new Float64Array([0, 0]);
            const v = new Float64Array([0, 0]);
            const tau = new Float64Array([0, 0]);

            const ddq_base = pin.aba(model2Link, data2LinkBase, q, v, tau);

            const w1 = new Float64Array([3.0, 0, 0, 0, 0, 0]);      // Torque at link 1
            const w2 = new Float64Array([0, 4.0, 0, 0, 5.0, 0]);   // Torque & force at link 2

            const ddq_l1 = pin.abaWithForces(model2Link, data2Link, q, v, tau, { 1: w1 });
            const ddq_l2 = pin.abaWithForces(model2Link, data2Link, q, v, tau, { 2: w2 });
            const ddq_both = pin.abaWithForces(model2Link, data2Link, q, v, tau, { 1: w1, 2: w2 });

            // Linear superposition of independent generalized forces: delta_both = delta_l1 + delta_l2
            const delta1_exp = (ddq_l1[0] - ddq_base[0]) + (ddq_l2[0] - ddq_base[0]);
            const delta2_exp = (ddq_l1[1] - ddq_base[1]) + (ddq_l2[1] - ddq_base[1]);

            const delta1_act = ddq_both[0] - ddq_base[0];
            const delta2_act = ddq_both[1] - ddq_base[1];

            check(assertClose(delta1_act, delta1_exp, 1e-10, 'Multi-link force superposition on joint 1'));
            check(assertClose(delta2_act, delta2_exp, 1e-10, 'Multi-link force superposition on joint 2'));
        } catch (e) { console.error(e); failed++; }

        console.log('  --- US2: Additive Accumulation of Spatial Wrenches (FR-009) ---');
        try {
            const q = new Float64Array([0, 0]);
            const v = new Float64Array([0, 0]);
            const tau = new Float64Array([0, 0]);

            const w_part1 = new Float64Array([2.0, 1.0, 0, 0, 3.0, 0]);
            const w_part2 = new Float64Array([1.0, 2.0, 0, 0, 2.0, 0]);
            const w_sum = new Float64Array([3.0, 3.0, 0, 0, 5.0, 0]);

            const ddq_sum = pin.abaWithForces(model2Link, data2Link, q, v, tau, { 2: w_sum });
            const ddq_p1 = pin.abaWithForces(model2Link, data2Link, q, v, tau, { 2: w_part1 });
            const ddq_p2 = pin.abaWithForces(model2Link, data2Link, q, v, tau, { 2: w_part2 });
            const ddq_base = pin.aba(model2Link, data2LinkBase, q, v, tau);

            const expected_shift_0 = (ddq_p1[0] - ddq_base[0]) + (ddq_p2[0] - ddq_base[0]);
            const actual_shift_0 = ddq_sum[0] - ddq_base[0];

            check(assertClose(actual_shift_0, expected_shift_0, 1e-10, 'Pre-summed spatial wrench matches combined effect'));
        } catch (e) { console.error(e); failed++; }

        console.log('  --- US2: Tool Payload Gravity Consistency ---');
        try {
            const q = new Float64Array([0, 0]);
            const v = new Float64Array([0, 0]);
            const tau = new Float64Array([0, 0]);

            const ddq_no_load = pin.aba(model2Link, data2LinkBase, q, v, tau);
            // Apply downward force on distal link (in local Y direction which produces moment around J1 X-axis)
            const ddq_payload = pin.abaWithForces(model2Link, data2Link, q, v, tau, {
                2: new Float64Array([0, 0, 0, 0, 10.0, 0])
            });

            check(assert(Math.abs(ddq_payload[0] - ddq_no_load[0]) > 1.0, 'Tool payload creates significant joint 1 acceleration'));
        } catch (e) { console.error(e); failed++; }

        // ─────────────────────────────────────────────────────────────
        // Phase 5 / User Story 3: Multi-Contact & Fixed Joints
        // ─────────────────────────────────────────────────────────────
        console.log('  --- US3: N-Wrench Multi-Contact Coverage (FR-006 / SC-006) ---');
        try {
            const q = new Float64Array([0.1, -0.2]);
            const v = new Float64Array([0.3, 0.1]);
            const tau = new Float64Array([0.5, -0.5]);

            const ddq_all_links = pin.abaWithForces(model2Link, data2Link, q, v, tau, {
                1: new Float64Array([1, 2, 3, 4, 5, 6]),
                2: new Float64Array([6, 5, 4, 3, 2, 1])
            });

            check(assert(ddq_all_links.length === 2 && !isNaN(ddq_all_links[0]) && !isNaN(ddq_all_links[1]), 'Simultaneous wrenches on all links execute cleanly'));
        } catch (e) { console.error(e); failed++; }

        console.log('  --- US3: Fixed-Joint Model Acceptance (FR-010) ---');
        try {
            const modelFixed = new pin.Model();
            const j1_f = pin.addJoint(modelFixed, 0, pin.JointModelRX(), id, "j1_f");
            pin.appendBodyToJoint(modelFixed, j1_f, inertia, id);

            const j2_fixed = pin.addJoint(modelFixed, j1_f, pin.JointModelFixed(), se3_2, "j2_fixed");
            pin.appendBodyToJoint(modelFixed, j2_fixed, inertia, id);

            const j3_f = pin.addJoint(modelFixed, j2_fixed, pin.JointModelRY(), se3_2, "j3_f");
            pin.appendBodyToJoint(modelFixed, j3_f, inertia, id);

            const dataFixed = new pin.Data(modelFixed);
            const qFixed = new Float64Array(modelFixed.nq);
            const vFixed = new Float64Array(modelFixed.nv);
            const tauFixed = new Float64Array(modelFixed.nv);

            const ddq_fixed = pin.abaWithForces(modelFixed, dataFixed, qFixed, vFixed, tauFixed, {
                [j2_fixed]: new Float64Array([1, 0, 0, 0, 5, 0])
            });

            check(assert(ddq_fixed.length === modelFixed.nv, `Fixed-joint wrench accepted without error (nv=${modelFixed.nv})`));
        } catch (e) { console.error(e); failed++; }

        // ─────────────────────────────────────────────────────────────
        // Phase 6: Polish, Error Handling & Performance (SC-003/SC-005)
        // ─────────────────────────────────────────────────────────────
        console.log('  --- Polish: Out-of-Range Index Error Handling (FR-007 / SC-005) ---');
        try {
            let threw = false;
            let errMsg = '';
            try {
                const q = new Float64Array([0]);
                const v = new Float64Array([0]);
                const tau = new Float64Array([0]);
                pin.abaWithForces(model1Dof, data1Dof, q, v, tau, { 999: new Float64Array(6) });
            } catch (e) {
                threw = true;
                errMsg = e.message;
            }
            check(assert(threw && (errMsg.includes('999') || errMsg.includes('out of range')),
                `Out-of-range index rejected with descriptive error: "${errMsg}"`));
        } catch (e) { console.error(e); failed++; }

        console.log('  --- Polish: Malformed Wrench Dimension Error Handling (FR-008 / SC-005) ---');
        try {
            let threw = false;
            let errMsg = '';
            try {
                const q = new Float64Array([0]);
                const v = new Float64Array([0]);
                const tau = new Float64Array([0]);
                pin.abaWithForces(model1Dof, data1Dof, q, v, tau, { 1: new Float64Array(5) });
            } catch (e) {
                threw = true;
                errMsg = e.message;
            }
            check(assert(threw && errMsg.includes('6 components'),
                `Malformed wrench rejected with descriptive error: "${errMsg}"`));
        } catch (e) { console.error(e); failed++; }

        console.log('  --- Polish: Performance Benchmark (SC-003 / SC-004) ---');
        try {
            const q = new Float64Array([0.1, -0.2]);
            const v = new Float64Array([0.3, 0.1]);
            const tau = new Float64Array([0.5, -0.5]);
            const fext = { 2: new Float64Array([0, 0, 0, 0, 5.0, 0]) };

            const iterations = 1000;

            // Warmup
            for (let i = 0; i < 50; i++) {
                pin.aba(model2Link, data2LinkBase, q, v, tau);
                pin.abaWithForces(model2Link, data2Link, q, v, tau, fext);
            }

            const t0_base = performance.now();
            for (let i = 0; i < iterations; i++) {
                pin.aba(model2Link, data2LinkBase, q, v, tau);
            }
            const t1_base = performance.now();

            const t0_fext = performance.now();
            for (let i = 0; i < iterations; i++) {
                pin.abaWithForces(model2Link, data2Link, q, v, tau, fext);
            }
            const t1_fext = performance.now();

            const timeBaseMs = (t1_base - t0_base) / iterations;
            const timeFextMs = (t1_fext - t0_fext) / iterations;

            console.log(`      aba: ${timeBaseMs.toFixed(4)} ms/call, abaWithForces: ${timeFextMs.toFixed(4)} ms/call`);

            check(assert(timeFextMs < 1.0, `abaWithForces per-call time < 1ms (${timeFextMs.toFixed(4)} ms)`));
        } catch (e) { console.error(e); failed++; }

        return { passed, failed };
    }
};
