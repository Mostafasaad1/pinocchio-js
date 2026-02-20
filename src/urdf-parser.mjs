/**
 * ──────────────────────────────────────────────────────────────
 * Pinocchio WASM — URDF Parser
 * Parses URDF XML using browser's DOMParser and builds a
 * Pinocchio model via the Embind API.
 * ──────────────────────────────────────────────────────────────
 */

/**
 * Parse a URDF XML string into a structured JavaScript object.
 *
 * @param {string} urdfString - The URDF XML content
 * @returns {{ links: Object[], joints: Object[], rootLink: string }}
 */
export function parseURDF(urdfString) {
    const parser = new DOMParser();
    const doc = parser.parseFromString(urdfString, 'text/xml');

    const errors = doc.getElementsByTagName('parsererror');
    if (errors.length > 0) {
        throw new Error(`URDF parse error: ${errors[0].textContent}`);
    }

    const robots = doc.getElementsByTagName('robot');
    if (robots.length === 0) throw new Error('No <robot> element found in URDF');
    const robot = robots[0];

    const robotName = robot.getAttribute('name') || 'unnamed';

    // ── Parse links ──
    const links = {};
    const linkNodes = Array.from(robot.getElementsByTagName('link'));
    for (const linkEl of linkNodes) {
        const name = linkEl.getAttribute('name');
        const link = { name, mass: 0, com: [0, 0, 0], inertia: [0, 0, 0, 0, 0, 0] };

        const inertialNodes = linkEl.getElementsByTagName('inertial');
        if (inertialNodes.length > 0) {
            const inertialEl = inertialNodes[0];
            // Mass
            const massNodes = inertialEl.getElementsByTagName('mass');
            if (massNodes.length > 0) {
                link.mass = parseFloat(massNodes[0].getAttribute('value')) || 0;
            }

            // Center of mass origin
            const originNodes = inertialEl.getElementsByTagName('origin');
            if (originNodes.length > 0) {
                link.com = parseXyz(originNodes[0]);
            }

            // Inertia tensor (6 unique elements)
            const inertiaNodes = inertialEl.getElementsByTagName('inertia');
            if (inertiaNodes.length > 0) {
                const inertiaEl = inertiaNodes[0];
                link.inertia = [
                    parseFloat(inertiaEl.getAttribute('ixx')) || 0,
                    parseFloat(inertiaEl.getAttribute('ixy')) || 0,
                    parseFloat(inertiaEl.getAttribute('ixz')) || 0,
                    parseFloat(inertiaEl.getAttribute('iyy')) || 0,
                    parseFloat(inertiaEl.getAttribute('iyz')) || 0,
                    parseFloat(inertiaEl.getAttribute('izz')) || 0,
                ];
            }
        }

        links[name] = link;
    }

    // ── Parse joints ──
    const joints = [];
    const childToJoint = {};  // child_link_name → joint

    const jointNodes = Array.from(robot.getElementsByTagName('joint'));
    for (const jointEl of jointNodes) {
        const joint = {
            name: jointEl.getAttribute('name'),
            type: jointEl.getAttribute('type'),
            parentLink: '',
            childLink: '',
            origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
            axis: [0, 0, 1],  // default Z axis
            limits: { lower: 0, upper: 0, effort: 0, velocity: 0 }
        };

        const parentNodes = jointEl.getElementsByTagName('parent');
        if (parentNodes.length > 0) joint.parentLink = parentNodes[0].getAttribute('link');

        const childNodes = jointEl.getElementsByTagName('child');
        if (childNodes.length > 0) joint.childLink = childNodes[0].getAttribute('link');

        const originNodes = jointEl.getElementsByTagName('origin');
        if (originNodes.length > 0) {
            joint.origin.xyz = parseXyz(originNodes[0]);
            joint.origin.rpy = parseRpy(originNodes[0]);
        }

        const axisNodes = jointEl.getElementsByTagName('axis');
        if (axisNodes.length > 0) {
            joint.axis = parseXyz(axisNodes[0]);
        }

        const limitNodes = jointEl.getElementsByTagName('limit');
        if (limitNodes.length > 0) {
            const limitEl = limitNodes[0];
            joint.limits.lower = parseFloat(limitEl.getAttribute('lower')) || 0;
            joint.limits.upper = parseFloat(limitEl.getAttribute('upper')) || 0;
            joint.limits.effort = parseFloat(limitEl.getAttribute('effort')) || 0;
            joint.limits.velocity = parseFloat(limitEl.getAttribute('velocity')) || 0;
        }

        joints.push(joint);
        childToJoint[joint.childLink] = joint;
    }

    // ── Find root link (a link that is never a child) ──
    const childLinks = new Set(joints.map(j => j.childLink));
    const rootLink = Object.keys(links).find(name => !childLinks.has(name)) || '';

    return { robotName, links, joints, rootLink };
}


/**
 * Build a Pinocchio Model from parsed URDF data.
 *
 * Implements "Fixed Joint Reduction" in JavaScript:
 * URDF Fixed joints are NOT added to the Pinocchio model as joints.
 * Instead, their kinematic transformation is composed into the placement
 * of the child link (and its subsequent children).
 *
 * @param {Object} pin - The Pinocchio WASM module
 * @param {Object} urdfData - Output of parseURDF()
 * @returns {Object} The constructed Pinocchio Model
 */
export function buildPinocchioModel(pin, urdfData) {
    const { links, joints, rootLink } = urdfData;
    const model = new pin.Model();

    // Map link names → Pinocchio parent joint ID
    // For fixed joints, the child link shares the same parent joint ID as its parent link.
    const linkToParentJointId = {};
    linkToParentJointId[rootLink] = 0; // Universe

    // Map link names → SE3 transform relative to the parent joint frame
    // For moving joints, this resets to Identity for the child (since the joint defines the frame).
    // For fixed joints, this accumulates the offset.
    const linkToJointTransform = {};
    linkToJointTransform[rootLink] = createIdentityTransform();

    // BFS Queue
    const queue = [rootLink];
    const visited = new Set([rootLink]);

    while (queue.length > 0) {
        const parentLinkName = queue.shift();
        const parentJointId = linkToParentJointId[parentLinkName];

        // Transform of the parent link frame relative to the parent joint frame
        const parentOffset = linkToJointTransform[parentLinkName] || createIdentityTransform();

        // Find all child joints
        const childJoints = joints.filter(j => j.parentLink === parentLinkName);

        for (const joint of childJoints) {
            if (visited.has(joint.childLink)) continue;
            // console.log(`Processing joint: ${joint.name} (type: ${joint.type})`);
            visited.add(joint.childLink);

            // 1. Calculate absolute placement of this joint/link relative to the PARENT JOINT frame
            // T_child = T_parent_offset * T_joint_origin
            const jointOrigin = createTransformFromXyzRpy(joint.origin.xyz, joint.origin.rpy);
            const placement = composeTransforms(parentOffset, jointOrigin);

            // 2. Handle Joint Type
            if (joint.type === 'fixed') {
                // REDUCTION: Do NOT add a Pinocchio joint.
                // The child link is rigidly attached to the parent joint.
                // We propagate the placement validation.

                linkToParentJointId[joint.childLink] = parentJointId;
                linkToJointTransform[joint.childLink] = placement;

                // Add inertia to the parent joint (transformed by placement)
                addBodyInertia(pin, model, links[joint.childLink], parentJointId, placement);

            } else {
                // MOVING JOINT: Add a real joint to Pinocchio

                // Convert JS transform to Pinocchio SE3 object
                const placementSE3 = toPinocchioSE3(pin, placement);
                const jointModel = createJointModel(pin, joint.type, joint.axis);

                let jointId;
                if (joint.limits.effort > 0 || joint.limits.velocity > 0) {
                    const nq = (joint.type === 'floating') ? 7 : 1;
                    const nv = (joint.type === 'floating') ? 6 : 1;
                    jointId = pin.addJointWithLimits(
                        model, parentJointId, jointModel, placementSE3, joint.name,
                        new Float64Array(nv).fill(joint.limits.effort),
                        new Float64Array(nv).fill(joint.limits.velocity),
                        new Float64Array(nq).fill(joint.limits.lower),
                        new Float64Array(nq).fill(joint.limits.upper)
                    );
                } else {
                    jointId = pin.addJoint(model, parentJointId, jointModel, placementSE3, joint.name);
                }

                // Register new joint
                linkToParentJointId[joint.childLink] = jointId;
                // The new link frame is the new joint frame (Identity offset)
                linkToJointTransform[joint.childLink] = createIdentityTransform();

                // Add inertia to the NEW joint (Identity placement)
                addBodyInertia(pin, model, links[joint.childLink], jointId, createIdentityTransform());
            }

            queue.push(joint.childLink);
        }
    }

    return model;
}

// ─── Internal Helpers: Physics ───────────────────────────────

function addBodyInertia(pin, model, linkData, jointId, transform) {
    if (!linkData || linkData.mass <= 0) return;

    // Construct Pinocchio Inertia object
    const inertia = pin.Inertia.fromMassComInertia(
        linkData.mass,
        linkData.com,
        linkData.inertia
    );

    // Convert JS transform to Pinocchio SE3
    const placement = toPinocchioSE3(pin, transform);

    // Append (Pinocchio will use the placement to transform inertia to joint frame)
    pin.appendBodyToJoint(model, jointId, inertia, placement);
}


// ─── Internal Helpers: Math (SE3 Composition) ────────────────

function createIdentityTransform() {
    return {
        R: [1, 0, 0, 0, 1, 0, 0, 0, 1], // Row-major
        t: [0, 0, 0]
    };
}

function createTransformFromXyzRpy(xyz, rpy) {
    const [x, y, z] = xyz;
    const [roll, pitch, yaw] = rpy;

    // ZYX Euler angles
    const cr = Math.cos(roll), sr = Math.sin(roll);
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const cy = Math.cos(yaw), sy = Math.sin(yaw);

    // Row-major rotation matrix
    const R = [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr
    ];

    return { R, t: [x, y, z] };
}

// Composition: T_AB = T_A * T_B
// R_AB = R_A * R_B
// t_AB = R_A * t_B + t_A
function composeTransforms(T1, T2) {
    const R1 = T1.R;
    const R2 = T2.R;
    const t1 = T1.t;
    const t2 = T2.t;

    // R = R1 * R2
    const R = [
        R1[0] * R2[0] + R1[1] * R2[3] + R1[2] * R2[6], R1[0] * R2[1] + R1[1] * R2[4] + R1[2] * R2[7], R1[0] * R2[2] + R1[1] * R2[5] + R1[2] * R2[8],
        R1[3] * R2[0] + R1[4] * R2[3] + R1[5] * R2[6], R1[3] * R2[1] + R1[4] * R2[4] + R1[5] * R2[7], R1[3] * R2[2] + R1[4] * R2[5] + R1[5] * R2[8],
        R1[6] * R2[0] + R1[7] * R2[3] + R1[8] * R2[6], R1[6] * R2[1] + R1[7] * R2[4] + R1[8] * R2[7], R1[6] * R2[2] + R1[7] * R2[5] + R1[8] * R2[8]
    ];

    // t = R1 * t2 + t1
    const t = [
        R1[0] * t2[0] + R1[1] * t2[1] + R1[2] * t2[2] + t1[0],
        R1[3] * t2[0] + R1[4] * t2[1] + R1[5] * t2[2] + t1[1],
        R1[6] * t2[0] + R1[7] * t2[1] + R1[8] * t2[2] + t1[2]
    ];

    return { R, t };
}

function toPinocchioSE3(pin, T) {
    return pin.SE3.fromRotationTranslation(
        new Float64Array(T.R),
        new Float64Array(T.t)
    );
}

/**
 * Map URDF joint type + axis to a Pinocchio JointModel.
 */
function createJointModel(pin, type, axis) {
    const [ax, ay, az] = axis;

    switch (type) {
        case 'revolute':
        case 'continuous':
            // Use aligned models when axis matches a principal direction
            if (ax === 1 && ay === 0 && az === 0) return pin.JointModelRX();
            if (ax === 0 && ay === 1 && az === 0) return pin.JointModelRY();
            if (ax === 0 && ay === 0 && az === 1) return pin.JointModelRZ();
            return pin.JointModelRevoluteUnaligned(ax, ay, az);

        case 'prismatic':
            if (ax === 1 && ay === 0 && az === 0) return pin.JointModelPX();
            if (ax === 0 && ay === 1 && az === 0) return pin.JointModelPY();
            if (ax === 0 && ay === 0 && az === 1) return pin.JointModelPZ();
            return pin.JointModelPrismaticUnaligned(ax, ay, az);

        case 'floating':
            return pin.JointModelFreeFlyer();

        case 'fixed':
            // Should not happen with reduction logic, but if used directly:
            return pin.JointModelFixed();

        default:
            console.warn(`Unknown joint type: ${type}, defaulting to revolute Z`);
            return pin.JointModelRZ();
    }
}

/**
 * Parse xyz attribute from a URDF element.
 * @returns {number[]} [x, y, z]
 */
function parseXyz(el) {
    const xyz = el.getAttribute('xyz');
    if (!xyz) return [0, 0, 0];
    return xyz.trim().split(/\s+/).map(Number);
}

/**
 * Parse rpy attribute from a URDF element.
 * @returns {number[]} [roll, pitch, yaw]
 */
function parseRpy(el) {
    const rpy = el.getAttribute('rpy');
    if (!rpy) return [0, 0, 0];
    return rpy.trim().split(/\s+/).map(Number);
}
