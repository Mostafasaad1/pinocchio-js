import { CollisionChecker } from '../../src/collision-checker.mjs';
import { parseURDF, buildPinocchioModel } from '../../src/urdf-parser.mjs';

export const browserTests = [
    {
        id: "test1",
        title: "Test 1 — 3-Joint Arm (RNEA)",
        run: (pin) => {
            const model = new pin.Model();
            const se3_1 = pin.SE3.fromXyzRpy(0, 0, 0.5, 0, 0, 0);
            const se3_2 = pin.SE3.fromXyzRpy(0, 0, 0.4, 0, 0, 0);
            const se3_3 = pin.SE3.fromXyzRpy(0, 0, 0.3, 0, 0, 0);

            const inertia = pin.Inertia.fromMassComInertia(
                1.0, [0, 0, 0.15], [0.01, 0, 0, 0.01, 0, 0.005]
            );

            const j1 = pin.addJoint(model, 0, pin.JointModelRX(), se3_1, "joint1");
            pin.appendBodyToJoint(model, j1, inertia, pin.SE3.identity());

            const j2 = pin.addJoint(model, j1, pin.JointModelRY(), se3_2, "joint2");
            pin.appendBodyToJoint(model, j2, inertia, pin.SE3.identity());

            const j3 = pin.addJoint(model, j2, pin.JointModelRZ(), se3_3, "joint3");
            pin.appendBodyToJoint(model, j3, inertia, pin.SE3.identity());

            const data = new pin.Data(model);

            const q = new Float64Array([0.5, -0.3, 0.1]);
            const v = new Float64Array([0.1, 0.0, -0.1]);
            const a = new Float64Array([0, 0, 0]);

            // Warmup
            pin.rnea(model, data, q, v, a);

            // Timed run
            const t0 = performance.now();
            const ITERS = 10000;
            let tau;
            for (let i = 0; i < ITERS; i++) {
                tau = pin.rnea(model, data, q, v, a);
            }
            const dt = performance.now() - t0;

            const result = `Model: ${model.njoints} joints, nq=${model.nq}, nv=${model.nv}\n` +
                `q   = [${Array.from(q).map(x => x.toFixed(3)).join(', ')}]\n` +
                `tau = [${Array.from(tau).map(x => x.toFixed(6)).join(', ')}]`;

            const timing = `${ITERS} RNEA calls in ${dt.toFixed(1)}ms (${(dt / ITERS * 1000).toFixed(1)} µs/call)`;

            return { result, timing, status: 'pass' };
        }
    },
    {
        id: "test2",
        title: "Test 2 — Center of Mass",
        run: (pin) => {
            const model = new pin.Model();
            const se3 = pin.SE3.fromXyzRpy(0, 0, 0.5, 0, 0, 0);
            const inertia = pin.Inertia.fromMassComInertia(
                2.0, [0, 0, 0.25], [0.02, 0, 0, 0.02, 0, 0.01]
            );

            const j1 = pin.addJoint(model, 0, pin.JointModelRZ(), se3, "joint1");
            pin.appendBodyToJoint(model, j1, inertia, pin.SE3.identity());

            const data = new pin.Data(model);
            const q = new Float64Array([0.0]);

            const t0 = performance.now();
            const ITERS = 10000;
            let com;
            for (let i = 0; i < ITERS; i++) {
                com = pin.centerOfMass(model, data, q);
            }
            const dt = performance.now() - t0;

            const mass = pin.computeTotalMass(model);

            const result = `Total mass: ${mass.toFixed(3)} kg\n` +
                `COM = [${Array.from(com).map(x => x.toFixed(6)).join(', ')}]`;

            const timing = `${ITERS} COM calls in ${dt.toFixed(1)}ms (${(dt / ITERS * 1000).toFixed(1)} µs/call)`;
            return { result, timing, status: 'pass' };
        }
    },
    {
        id: "test3",
        title: "Test 3 — Jacobian",
        run: (pin) => {
            const model = new pin.Model();
            const se3 = pin.SE3.fromXyzRpy(0, 0, 0.5, 0, 0, 0);
            const inertia = pin.Inertia.fromMassComInertia(
                1.0, [0, 0, 0.2], [0.01, 0, 0, 0.01, 0, 0.005]
            );

            const j1 = pin.addJoint(model, 0, pin.JointModelRX(), se3, "j1");
            pin.appendBodyToJoint(model, j1, inertia, pin.SE3.identity());
            const j2 = pin.addJoint(model, j1, pin.JointModelRY(), se3, "j2");
            pin.appendBodyToJoint(model, j2, inertia, pin.SE3.identity());

            const data = new pin.Data(model);
            const q = new Float64Array([0.3, -0.2]);

            pin.computeJointJacobians(model, data, q);
            const J = pin.getJointJacobian(model, data, 2, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED.value);

            const nv = model.nv;
            let matStr = '';
            for (let row = 0; row < 6; row++) {
                const rowVals = [];
                for (let col = 0; col < nv; col++) {
                    rowVals.push(J[col * 6 + row].toFixed(4));
                }
                matStr += `  [${rowVals.join(', ')}]\n`;
            }

            const result = `Jacobian of joint 2 (6×${nv}):\n${matStr}`;
            return { result, timing: '', status: 'pass' };
        }
    },
    {
        id: "test4",
        title: "Test 4 — Collisions",
        run: (pin) => {
            const dummyURDF = `
            <robot name="joint_test">
                <link name="base_link">
                    <collision><geometry><box size="1 1 1"/></geometry></collision>
                </link>
                <link name="link1">
                    <collision>
                        <origin xyz="1 0 0" rpy="0 0 0"/>
                        <geometry><box size="1 1 1"/></geometry>
                    </collision>
                </link>
                <link name="link2">
                    <collision>
                        <origin xyz="2 0 0" rpy="0 0 0"/>
                        <geometry><box size="1 1 1"/></geometry>
                    </collision>
                </link>
                <joint name="j1" type="prismatic">
                    <parent link="base_link"/><child link="link1"/>
                    <origin xyz="1 0 0"/><axis xyz="1 0 0"/>
                </joint>
                <joint name="j2" type="prismatic">
                    <parent link="link1"/><child link="link2"/>
                    <origin xyz="1 0 0"/><axis xyz="1 0 0"/>
                </joint>
            </robot>`;
            const urdfData = parseURDF(dummyURDF);
            const model = buildPinocchioModel(pin, urdfData);
            const data = new pin.Data(model);
            const checker = new CollisionChecker(pin, model, data, urdfData);

            const qSafe = new Float64Array([0, 0]);
            checker.updateCollisions(qSafe);
            const resSafe = checker.checkCollisions();

            const qColliding = new Float64Array([0, -4]); // Folds back to base
            
            const t0 = performance.now();
            const ITERS = 1000;
            let resColliding;
            for (let i = 0; i < ITERS; i++) {
                checker.updateCollisions(qColliding);
                resColliding = checker.checkCollisions();
            }
            const dt = performance.now() - t0;

            const contactStr = resColliding.contacts.map(c => `[${c.linkA}, ${c.linkB}]`).join(', ');

            const result = `q=[0, 0] Collision: ${resSafe.hasCollision}\n` +
                `q=[0, -4] Collision: ${resColliding.hasCollision}\n` +
                `Contacts: ${contactStr}`;

            const timing = `${ITERS} queries in ${dt.toFixed(1)}ms (${(dt / ITERS * 1000).toFixed(1)} µs/call)`;
            return { result, timing, status: 'pass' };
        }
    },
    {
        id: "test5",
        title: "Test 5 — Forward Dynamics with Forces (ABA)",
        run: (pin) => {
            const model = new pin.Model();
            const se3 = pin.SE3.fromXyzRpy(0, 0, 0.5, 0, 0, 0);
            const inertia = pin.Inertia.fromMassComInertia(
                1.0, [0, 0, 0.25], [0.01, 0, 0, 0.01, 0, 0.005]
            );

            const j1 = pin.addJoint(model, 0, pin.JointModelRX(), se3, "j1");
            pin.appendBodyToJoint(model, j1, inertia, pin.SE3.identity());
            const j2 = pin.addJoint(model, j1, pin.JointModelRY(), se3, "j2");
            pin.appendBodyToJoint(model, j2, inertia, pin.SE3.identity());

            const data1 = new pin.Data(model);
            const data2 = new pin.Data(model);
            const q = new Float64Array([0.2, -0.3]);
            const v = new Float64Array([0.1, 0.0]);
            const tau = new Float64Array([0.0, 0.0]);

            const fext = {
                2: new Float64Array([0, 0, 0, 0, 0, -9.81]) // 1kg load in -Z
            };

            const ddq_base = pin.aba(model, data1, q, v, tau);

            const t0 = performance.now();
            const ITERS = 10000;
            let ddq_fext;
            for (let i = 0; i < ITERS; i++) {
                ddq_fext = pin.abaWithForces(model, data2, q, v, tau, fext);
            }
            const dt = performance.now() - t0;

            const result = `Baseline aba:        [${Array.from(ddq_base).map(x => x.toFixed(4)).join(', ')}]\n` +
                `With External Force: [${Array.from(ddq_fext).map(x => x.toFixed(4)).join(', ')}]\n` +
                `Applied at link 2:   [0, 0, 0, 0, 0, -9.81] N`;

            const timing = `${ITERS} ABA calls with forces in ${dt.toFixed(1)}ms (${(dt / ITERS * 1000).toFixed(1)} µs/call)`;
            return { result, timing, status: 'pass' };
        }
    },
    {
        id: "test6",
        title: "Test 6 — Forward Kinematics (QVA)",
        run: (pin) => {
            const model = new pin.Model();
            const se3 = pin.SE3.fromXyzRpy(0, 0, 0.5, 0, 0, 0);
            const inertia = pin.Inertia.fromMassComInertia(
                1.0, [0, 0, 0.25], [0.01, 0, 0, 0.01, 0, 0.005]
            );

            const j1 = pin.addJoint(model, 0, pin.JointModelRX(), se3, "j1");
            pin.appendBodyToJoint(model, j1, inertia, pin.SE3.identity());
            const j2 = pin.addJoint(model, j1, pin.JointModelRY(), se3, "j2");
            pin.appendBodyToJoint(model, j2, inertia, pin.SE3.identity());

            const data = new pin.Data(model);
            const q = new Float64Array([0.2, -0.3]);
            const v = new Float64Array([0.1, -0.1]);
            const a = new Float64Array([0.5, 0.5]);

            const t0 = performance.now();
            const ITERS = 10000;
            for (let i = 0; i < ITERS; i++) {
                pin.forwardKinematicsQVA(model, data, q, v, a);
            }
            const dt = performance.now() - t0;

            const spatialVel = data.getVelocity(2);
            const spatialAcc = data.getAcceleration(2);

            const result = `q = [0.2, -0.3], v = [0.1, -0.1], a = [0.5, 0.5]\n` +
                `Link 2 Spatial Velocity:\n  [${Array.from(spatialVel).map(x => x.toFixed(4)).join(', ')}]\n` +
                `Link 2 Spatial Acceleration:\n  [${Array.from(spatialAcc).map(x => x.toFixed(4)).join(', ')}]`;

            const timing = `${ITERS} QVA calls in ${dt.toFixed(1)}ms (${(dt / ITERS * 1000).toFixed(1)} µs/call)`;
            return { result, timing, status: 'pass' };
        }
    },
    {
        id: "test7",
        title: "Test 7 — Frame Jacobians",
        run: (pin) => {
            const model = new pin.Model();
            const se3 = pin.SE3.fromXyzRpy(0, 0, 0.5, 0, 0, 0);
            const inertia = pin.Inertia.fromMassComInertia(
                1.0, [0, 0, 0.25], [0.01, 0, 0, 0.01, 0, 0.005]
            );

            const j1 = pin.addJoint(model, 0, pin.JointModelRX(), se3, "j1");
            pin.appendBodyToJoint(model, j1, inertia, pin.SE3.identity());
            const j2 = pin.addJoint(model, j1, pin.JointModelRY(), se3, "j2");
            pin.appendBodyToJoint(model, j2, inertia, pin.SE3.identity());

            const data = new pin.Data(model);
            const q = new Float64Array([0.2, -0.3]);
            
            const t0 = performance.now();
            const ITERS = 10000;
            const frameId = model.getFrameId("j2");
            let J_lwa;
            for (let i = 0; i < ITERS; i++) {
                J_lwa = pin.computeFrameJacobian(model, data, q, frameId, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED.value);
            }
            const dt = performance.now() - t0;

            const nv = model.nv;
            let matStr = '';
            for (let row = 0; row < 6; row++) {
                const rowVals = [];
                for (let col = 0; col < nv; col++) {
                    rowVals.push(J_lwa[col * 6 + row].toFixed(4));
                }
                matStr += `  [${rowVals.join(', ')}]\n`;
            }

            const result = `Frame Jacobian of "j2" (6×${nv}):\n${matStr}`;
            const timing = `${ITERS} computeFrameJacobian calls in ${dt.toFixed(1)}ms (${(dt / ITERS * 1000).toFixed(1)} µs/call)`;
            return { result, timing, status: 'pass' };
        }
    }
];
