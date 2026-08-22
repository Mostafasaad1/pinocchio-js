/**
 * Pinocchio WASM — Frame Jacobians Tests
 */

module.exports = {
    run: async (ctx) => {
        const { pin } = ctx;
        const assert = require('assert');
        let passed = 0;
        let failed = 0;

        function check(name, testFn) {
            try {
                testFn();
                console.log(`  [OK] ${name}`);
                passed++;
            } catch (e) {
                console.error(`  [FAIL] ${name}`);
                console.error(`         ${e.message}`);
                failed++;
            }
        }

        console.log("--- Frame Jacobians ---");

        const model = new pin.Model();
        // create a basic model
        const jointId = pin.addJoint(model, 0, pin.JointModelRX(), pin.SE3.identity(), "joint1");
        
        const data = new pin.Data(model);
        const q = pin.neutralConfiguration(model);

        // Frame ID 0 is universe. Frame ID 1 should be the joint.
        // We will test on frameId = 0.
        const frameId = 0;

        check("computeFrameJacobian - invalid frame ID throws", () => {
            assert.throws(() => {
                pin.computeFrameJacobian(model, data, q, 99999, pin.ReferenceFrame.LOCAL);
            }, /Invalid frame ID/);
        });

        check("computeFrameJacobian - executes on valid frame", () => {
            const J = pin.computeFrameJacobian(model, data, q, frameId, pin.ReferenceFrame.LOCAL);
            assert.strictEqual(J.length, 6 * model.nv); // Since matrixXdToJs probably flattens it or we check correctly
        });

        check("getFrameJacobian - invalid frame ID throws", () => {
            assert.throws(() => {
                pin.getFrameJacobian(model, data, 99999, pin.ReferenceFrame.LOCAL);
            }, /Invalid frame ID/);
        });

        check("getFrameJacobian - executes on valid frame", () => {
            pin.forwardKinematics(model, data, q);
            const J_get = pin.getFrameJacobian(model, data, frameId, pin.ReferenceFrame.LOCAL);
            assert.strictEqual(J_get.length, 6 * model.nv);
        });

        check("getFrameVelocity - invalid frame ID throws", () => {
            assert.throws(() => {
                pin.getFrameVelocity(model, data, 99999, pin.ReferenceFrame.LOCAL);
            }, /Invalid frame ID/);
        });

        check("getFrameVelocity - executes on valid frame", () => {
            const v = new Float64Array(model.nv).fill(0.5);
            const a = new Float64Array(model.nv).fill(0.0);
            pin.forwardKinematicsQVA(model, data, q, v, a);
            const v_frame = pin.getFrameVelocity(model, data, frameId, pin.ReferenceFrame.LOCAL);
            assert.strictEqual(v_frame.length, 6);
        });

        return { passed, failed };
    }
};
