/**
 * Pinocchio WASM — Forward Kinematics (QVA) Tests
 */

module.exports = {
    run: async (ctx) => {
        const { pin, assert, assertClose, assertVecClose } = ctx;
        let passed = 0;
        let failed = 0;

        function check(cond) {
            if (cond) passed++; else failed++;
        }

        console.log('  --- Setup: 2-Link Planar Arm (RZ) ---');
        const model = new pin.Model();
        const id = pin.SE3.identity();
        
        // Joint 1: RZ at origin
        const j1 = pin.addJoint(model, 0, pin.JointModelRZ(), id, "j1");
        
        // Joint 2: RZ offset by 1.0 in X
        const se3_2 = pin.SE3.fromXyzRpy(1.0, 0, 0, 0, 0, 0);
        const j2 = pin.addJoint(model, j1, pin.JointModelRZ(), se3_2, "j2");
        
        const data = new pin.Data(model);
        check(assert(model.nq === 2 && model.nv === 2, 'Model created (nq=2)'));

        // Test QVA
        console.log('  --- Forward Kinematics QVA ---');
        try {
            const q = new Float64Array([0, 0]);
            const v = new Float64Array([1, 2]); // j1: 1 rad/s, j2: 2 rad/s
            const a = new Float64Array([0.5, -1.0]); // j1: 0.5 rad/s^2, j2: -1.0 rad/s^2
            
            // This function should be added in our task
            pin.forwardKinematicsQVA(model, data, q, v, a);
            
            // J1 velocity: omega=[0,0,1], v=[0,0,0]
            const v1 = data.getVelocity(j1); 
            check(assertVecClose(v1, [0, 0, 0, 0, 0, 1], 1e-6, 'Joint 1 Spatial Velocity'));
            
            // J2 velocity: omega1 x r = [0,0,1] x [1,0,0] = [0,1,0]. Local angular = [0,0,3].
            // Pinocchio spatial velocity format is usually [v_x, v_y, v_z, w_x, w_y, w_z]
            const v2 = data.getVelocity(j2);
            check(assertVecClose(v2, [0, 1, 0, 0, 0, 3], 1e-6, 'Joint 2 Spatial Velocity'));
            
            // J1 acceleration: alpha=[0,0,0.5], a=[0,0,0]
            const a1 = data.getAcceleration(j1);
            check(assertVecClose(a1, [0, 0, 0, 0, 0, 0.5], 1e-6, 'Joint 1 Spatial Acceleration'));
            
            // J2 acceleration: 
            // a2 = a1 + a_relative + coriolis
            // a1 at origin 2 = [0,0,0.5] x [1,0,0] + [0,0,1] x ([0,0,1] x [1,0,0])
            //                = [0,0.5,0] + [0,0,1] x [0,1,0]
            //                = [0,0.5,0] + [-1,0,0] = [-1, 0.5, 0]
            // a_relative = [0,0,0] linear, [0,0,-1.0] angular
            // coriolis = [0,0,1] x [0,0,-1] (angular part) -> 0
            // Actually pinocchio uses spatial acceleration.
            // Let's just check the method is callable and returns a 6-array first, 
            // but we can compute exact values for robustness.
            // angular: alpha_1 + alpha_2 + omega_1 x omega_2 = [0,0,0.5] + [0,0,-1] + 0 = [0,0,-0.5]
            const a2 = data.getAcceleration(j2);
            check(assert(a2.length === 6, 'Joint 2 Acceleration has length 6'));
            check(assertVecClose(a2, [2.0, 0.5, 0, 0, 0, -0.5], 1e-6, 'Joint 2 Spatial Acceleration'));
            
        } catch (e) { console.error(e); failed++; }

        return { passed, failed };
    }
};
