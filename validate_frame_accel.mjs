import pinocchio from './build/pinocchio.js';

// 1. Initialize module
const pin = await pinocchio();

// 2. Setup model
const model = new pin.Model();
// Add a joint to act as a frame point
const jointId = pin.addJoint(model, 0, pin.JointModelRX(), pin.SE3.identity(), "test_joint");

const data = new pin.Data(model);

// 3. Prepare inputs
const q = new Float64Array(model.nq).fill(0);
const v = new Float64Array(model.nv).fill(1.0);
const a = new Float64Array(model.nv).fill(0.5);
const outAccel = new Float64Array(6);

// 4. Test Error Case: Before QVA
try {
    pin.getFrameAcceleration(model, data, "test_joint", pin.ReferenceFrame.LOCAL, outAccel);
    console.error("FAIL: Did not throw before QVA");
} catch (e) {
    console.log("PASS: Threw expected error before QVA:", e.message);
}

// 5. Test Error Case: Invalid Frame Name
try {
    pin.forwardKinematicsQVA(model, data, q, v, a);
    pin.getFrameAcceleration(model, data, "bad_frame", pin.ReferenceFrame.LOCAL, outAccel);
    console.error("FAIL: Did not throw for invalid frame name");
} catch (e) {
    console.log("PASS: Threw expected error for invalid name:", e.message);
}

// 6. Test Success Case
try {
    pin.getFrameAcceleration(model, data, "test_joint", pin.ReferenceFrame.LOCAL, outAccel);
    console.log("PASS: Frame acceleration computed successfully:", outAccel);
} catch (e) {
    console.error("FAIL: Success case threw error:", e.message);
}

// Ensure memory is not leaked
console.log("Validation complete.");
