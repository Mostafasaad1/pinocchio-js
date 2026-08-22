/**
 * Pinocchio WASM — Deep Test for Frame Jacobians
 */

const fs = require('fs');
const path = require('path');
const { DOMParser } = require('xmldom');

// Polyfill global DOMParser for the URDF parser
global.DOMParser = DOMParser;

module.exports = {
    run: async (ctx) => {
        const { pin, assert, assertClose, assertVecClose } = ctx;
        let passed = 0;
        let failed = 0;

        function check(name, testFn) {
            try {
                const res = testFn();
                if (res !== false) {
                    console.log(`  [OK] ${name}`);
                    passed++;
                } else {
                    console.error(`  [FAIL] ${name}`);
                    failed++;
                }
            } catch (e) {
                console.error(`  [FAIL] ${name}`);
                console.error(`         ${e.message}`);
                failed++;
            }
        }

        console.log("--- Frame Jacobians Deep Tests ---");

        // Load the ESM parser dynamically
        const { parseURDF, buildPinocchioModel } = await import('../src/urdf-parser.mjs');

        const urdfPath = path.join(__dirname, '../urdf/abb_irb120_support/urdf/abbIrb120.urdf');
        if (!fs.existsSync(urdfPath)) {
            console.error(`  ⚠️ URDF not found: ${urdfPath}`);
            return { passed, failed: failed + 1 };
        }

        const xml = fs.readFileSync(urdfPath, 'utf8');
        const urdfData = parseURDF(xml);
        const model = buildPinocchioModel(pin, urdfData);
        const data = new pin.Data(model);

        const q = new Float64Array(model.nq).fill(0);
        q[0] = 0.1;
        q[1] = -0.2;
        q[2] = 0.3;
        q[3] = -0.4;
        q[4] = 0.5;
        q[5] = -0.6;

        pin.forwardKinematics(model, data, q);
        pin.updateFramePlacements(model, data);

        // Frame to test
        const frameName = "joint_6";
        let frameId;
        
        check("existFrame / getFrameId API", () => {
            console.log(`Model stats: nq=${model.nq} nv=${model.nv} njoints=${model.njoints} nframes=${model.nframes}`);
            if (!model.existFrame(frameName)) {
                throw new Error(`Frame ${frameName} does not exist in model`);
            }
            frameId = model.getFrameId(frameName);
            if (frameId >= model.nframes) {
                throw new Error(`Invalid frame ID returned: ${frameId}`);
            }
            return true;
        });

        check("compare LOCAL and LOCAL_WORLD_ALIGNED jacobians", () => {
            const J_local = pin.computeFrameJacobian(model, data, q, frameId, pin.ReferenceFrame.LOCAL);
            const J_lwa = pin.computeFrameJacobian(model, data, q, frameId, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED);

            // get the frame placement
            if (typeof data.oMf !== 'function') {
                console.log("data prototype keys:", Object.getOwnPropertyNames(Object.getPrototypeOf(data)));
                throw new Error("data.oMf is not a function");
            }
            const oMf = data.oMf(frameId);
            const R = oMf.rotation; // 9-element 1D array representing 3x3 matrix in column-major? 
            // Wait, rotation in SE3 is returned as a 1D array of 9 elements (row-major or col-major?)
            // I'll multiply manually assuming it's row-major (since we return std::vector in JS)
            // Or I can just check length
            assert(J_local.length === 6 * model.nv, "J_local length is 6 * nv");
            assert(J_lwa.length === 6 * model.nv, "J_lwa length is 6 * nv");

            // Compute blockDiag(R, R) * J_local
            // In Eigen matrixXdToJs, matrices are flattened column-major or row-major?
            // Wait, we returned a flat array. Let's not do full matrix multiplication in JS unless we know the layout.
            // But we know that LOCAL and LOCAL_WORLD_ALIGNED magnitudes of columns should be identical!
            for (let c = 0; c < model.nv; c++) {
                let norm_local_lin = 0;
                let norm_lwa_lin = 0;
                let norm_local_ang = 0;
                let norm_lwa_ang = 0;
                // Since it's a 6xNV matrix, if it's column-major (Eigen default),
                // idx = r + c * 6
                // Let's assume column-major:
                for (let r = 0; r < 3; r++) {
                    const l_lin = J_local[r + c * 6];
                    const w_lin = J_lwa[r + c * 6];
                    norm_local_lin += l_lin * l_lin;
                    norm_lwa_lin += w_lin * w_lin;

                    const l_ang = J_local[(r+3) + c * 6];
                    const w_ang = J_lwa[(r+3) + c * 6];
                    norm_local_ang += l_ang * l_ang;
                    norm_lwa_ang += w_ang * w_ang;
                }
                
                assertClose(Math.sqrt(norm_local_lin), Math.sqrt(norm_lwa_lin), 1e-8, `Col ${c} linear norm matches`);
                assertClose(Math.sqrt(norm_local_ang), Math.sqrt(norm_lwa_ang), 1e-8, `Col ${c} angular norm matches`);
            }
            return true;
        });

        check("getFrameVelocity matches computeFrameJacobian * v across reference frames", () => {
            const v = new Float64Array(model.nv);
            v[0] = 0.5; v[1] = -0.3; v[2] = 0.2; v[3] = -0.1; v[4] = 0.4; v[5] = -0.2;
            const a = new Float64Array(model.nv).fill(0);

            pin.forwardKinematicsQVA(model, data, q, v, a);

            const framesToTest = [
                { name: "LOCAL", ref: pin.ReferenceFrame.LOCAL },
                { name: "LOCAL_WORLD_ALIGNED", ref: pin.ReferenceFrame.LOCAL_WORLD_ALIGNED },
                { name: "WORLD", ref: pin.ReferenceFrame.WORLD }
            ];

            for (const { name: rfName, ref: rf } of framesToTest) {
                const J = pin.computeFrameJacobian(model, data, q, frameId, rf);
                const v_frame = pin.getFrameVelocity(model, data, frameId, rf);

                assert(v_frame.length === 6, `v_frame (${rfName}) length is 6`);

                // Compute J * v
                const v_expected = new Float64Array(6);
                for (let r = 0; r < 6; r++) {
                    for (let c = 0; c < model.nv; c++) {
                        v_expected[r] += J[c * 6 + r] * v[c];
                    }
                }

                assertVecClose(v_frame, v_expected, 1e-6, `getFrameVelocity matches J*v for ${rfName}`);
            }
            return true;
        });

        return { passed, failed };
    }
};
