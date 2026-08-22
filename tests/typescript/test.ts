import PinocchioModule, {
    Model,
    Data,
    ReferenceFrame,
    SE3,
    Inertia,
    JointModel,
    JointModelRX,
    JointModelRY,
    JointModelFixed,
    addJoint,
    appendBodyToJoint,
    rnea,
    forwardKinematics,
    centerOfMass,
    neutralConfiguration,
    getJointJacobian,
    computeJointJacobians,
    getFrameVelocity,
    computeFrameJacobian,
    getFrameJacobian
} from '../../src/pinocchio';
import { parseURDF, buildPinocchioModel, URDFData } from '../../src/urdf-parser';
import { CollisionChecker, CollisionResult } from '../../src/collision-checker';

async function runTest(): Promise<void> {
    const pin = await PinocchioModule();

    // Model & Data initialization
    const model = new pin.Model();
    const id = pin.SE3.identity();
    const inertia = pin.Inertia.fromMassComInertia(1.0, [0, 0, 0.5], [0.1, 0, 0, 0.1, 0, 0.01]);

    const j1 = pin.addJoint(model, 0, pin.JointModelRX(), id, 'j1');
    pin.appendBodyToJoint(model, j1, inertia, id);

    const se3_2 = pin.SE3.fromXyzRpy(0, 0, 1.0, 0, 0, 0);
    const j2 = pin.addJoint(model, j1, pin.JointModelRY(), se3_2, 'j2');
    pin.appendBodyToJoint(model, j2, inertia, id);

    const data = new pin.Data(model);

    // Neutral configuration
    const q0: Float64Array = pin.neutralConfiguration(model);

    // RNEA
    const q = new Float64Array([0, 0]);
    const v = new Float64Array([0, 0]);
    const a = new Float64Array([0, 0]);
    const tau: Float64Array = pin.rnea(model, data, q, v, a);

    // Forward kinematics & Center of mass
    pin.forwardKinematics(model, data, q);
    const com: Float64Array = pin.centerOfMass(model, data, q);

    // Jacobians & Frame Velocities
    pin.computeJointJacobians(model, data, q);
    const J: Float64Array = pin.getJointJacobian(model, data, 2, pin.ReferenceFrame.LOCAL);
    const frameId = model.getFrameId('j2');
    const J_frame: Float64Array = pin.computeFrameJacobian(model, data, q, frameId, pin.ReferenceFrame.LOCAL);
    const v_frame: Float64Array = pin.getFrameVelocity(model, data, frameId, pin.ReferenceFrame.LOCAL);

    // Access properties
    const nq: number = model.nq;
    const nv: number = model.nv;
    const njoints: number = model.njoints;

    // URDF Parser testing
    const sampleUrdf = `
        <robot name="simple_robot">
            <link name="base_link" />
            <link name="link1">
                <inertial>
                    <mass value="1.0" />
                    <origin xyz="0 0 0.5" />
                    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01" />
                </inertial>
            </link>
            <joint name="joint1" type="revolute">
                <parent link="base_link" />
                <child link="link1" />
                <origin xyz="0 0 0" rpy="0 0 0" />
                <axis xyz="1 0 0" />
                <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0" />
            </joint>
        </robot>
    `;

    const parsedData: URDFData = parseURDF(sampleUrdf);
    const urdfModel: Model = buildPinocchioModel(pin, parsedData);

    // Collision Checker testing
    const collisionChecker = new CollisionChecker(pin, urdfModel, data, parsedData);
    collisionChecker.setIgnoredPairs([['base_link', 'link1']]);
    collisionChecker.updateCollisions(q0);
    const colResult: CollisionResult = collisionChecker.checkCollisions();

    console.log({
        nq,
        nv,
        njoints,
        tauLength: tau.length,
        comLength: com.length,
        jLength: J.length,
        urdfRobotName: parsedData.robotName,
        urdfNq: urdfModel.nq,
        hasCollision: colResult.hasCollision
    });
}

runTest().catch((err) => {
    console.error(err);
    process.exit(1);
});
