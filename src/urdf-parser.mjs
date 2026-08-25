/**
 * ──────────────────────────────────────────────────────────────
 * Pinocchio WASM — URDF Parser
 * Parses URDF XML using browser's DOMParser and builds a
 * Pinocchio model via the Embind API.
 * ──────────────────────────────────────────────────────────────
 */

function getDirectChildren(el, tagName) {
    const res = [];
    if (!el || !el.childNodes) return res;
    const lower = tagName.toLowerCase();
    for (let i = 0; i < el.childNodes.length; i++) {
        const child = el.childNodes[i];
        if (child.nodeType === 1 && (child.tagName === tagName || child.nodeName === tagName || child.tagName?.toLowerCase() === lower)) {
            res.push(child);
        }
    }
    return res;
}

function getFirstDirectChild(el, tagName) {
    if (!el || !el.childNodes) return null;
    const lower = tagName.toLowerCase();
    for (let i = 0; i < el.childNodes.length; i++) {
        const child = el.childNodes[i];
        if (child.nodeType === 1 && (child.tagName === tagName || child.nodeName === tagName || child.tagName?.toLowerCase() === lower)) {
            return child;
        }
    }
    return null;
}

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
    let linkNodes = getDirectChildren(robot, 'link');
    if (linkNodes.length === 0) {
        linkNodes = Array.from(robot.getElementsByTagName('link'));
    }

    for (let l = 0; l < linkNodes.length; l++) {
        const linkEl = linkNodes[l];
        const name = linkEl.getAttribute('name');
        const link = { name, mass: 0, com: [0, 0, 0], inertia: [0, 0, 0, 0, 0, 0], collisions: [] };

        const inertialEl = getFirstDirectChild(linkEl, 'inertial');
        if (inertialEl) {
            // Mass
            const massEl = getFirstDirectChild(inertialEl, 'mass');
            if (massEl) {
                link.mass = parseFloat(massEl.getAttribute('value')) || 0;
            }

            // Center of mass origin
            const originEl = getFirstDirectChild(inertialEl, 'origin');
            if (originEl) {
                link.com = parseXyz(originEl);
            }

            // Inertia tensor (6 unique elements)
            const inertiaEl = getFirstDirectChild(inertialEl, 'inertia');
            if (inertiaEl) {
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

        // ── Parse collisions ──
        let collisionNodes = getDirectChildren(linkEl, 'collision');
        if (collisionNodes.length === 0) {
            collisionNodes = Array.from(linkEl.getElementsByTagName('collision'));
        }

        for (let c = 0; c < collisionNodes.length; c++) {
            const collisionEl = collisionNodes[c];
            const collision = {
                origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
                geometry: { min: [0, 0, 0], max: [0, 0, 0] }
            };

            const originEl = getFirstDirectChild(collisionEl, 'origin');
            if (originEl) {
                collision.origin.xyz = parseXyz(originEl);
                collision.origin.rpy = parseRpy(originEl);
            }

            const geometryEl = getFirstDirectChild(collisionEl, 'geometry');
            if (geometryEl) {
                const boxEl = getFirstDirectChild(geometryEl, 'box');
                if (boxEl) {
                    const sizeAttr = boxEl.getAttribute('size');
                    const size = sizeAttr ? parseNumbers(sizeAttr) : [0, 0, 0];
                    collision.geometry.min = [-size[0]/2, -size[1]/2, -size[2]/2];
                    collision.geometry.max = [size[0]/2, size[1]/2, size[2]/2];
                } else {
                    const cylinderEl = getFirstDirectChild(geometryEl, 'cylinder');
                    if (cylinderEl) {
                        const radius = parseFloat(cylinderEl.getAttribute('radius')) || 0;
                        const length = parseFloat(cylinderEl.getAttribute('length')) || 0;
                        collision.geometry.min = [-radius, -radius, -length/2];
                        collision.geometry.max = [radius, radius, length/2];
                    } else {
                        const sphereEl = getFirstDirectChild(geometryEl, 'sphere');
                        if (sphereEl) {
                            const radius = parseFloat(sphereEl.getAttribute('radius')) || 0;
                            collision.geometry.min = [-radius, -radius, -radius];
                            collision.geometry.max = [radius, radius, radius];
                        } else {
                            const meshEl = getFirstDirectChild(geometryEl, 'mesh');
                            if (meshEl) {
                                const scaleAttr = meshEl.getAttribute('scale');
                                const scaleVals = scaleAttr ? parseNumbers(scaleAttr) : [1, 1, 1];
                                collision.geometry.min = [-scaleVals[0]/2, -scaleVals[1]/2, -scaleVals[2]/2];
                                collision.geometry.max = [scaleVals[0]/2, scaleVals[1]/2, scaleVals[2]/2];
                            }
                        }
                    }
                }
            }

            link.collisions.push(collision);
        }

        links[name] = link;
    }

    // ── Parse joints ──
    const joints = [];
    const childToJoint = {};

    let jointNodes = getDirectChildren(robot, 'joint');
    if (jointNodes.length === 0) {
        jointNodes = Array.from(robot.getElementsByTagName('joint'));
    }

    for (let j = 0; j < jointNodes.length; j++) {
        const jointEl = jointNodes[j];
        const joint = {
            name: jointEl.getAttribute('name'),
            type: jointEl.getAttribute('type'),
            parentLink: '',
            childLink: '',
            origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
            axis: [0, 0, 1],  // default Z axis
            limits: { lower: 0, upper: 0, effort: 0, velocity: 0 }
        };

        const parentEl = getFirstDirectChild(jointEl, 'parent');
        if (parentEl) joint.parentLink = parentEl.getAttribute('link');

        const childEl = getFirstDirectChild(jointEl, 'child');
        if (childEl) joint.childLink = childEl.getAttribute('link');

        const originEl = getFirstDirectChild(jointEl, 'origin');
        if (originEl) {
            joint.origin.xyz = parseXyz(originEl);
            joint.origin.rpy = parseRpy(originEl);
        }

        const axisEl = getFirstDirectChild(jointEl, 'axis');
        if (axisEl) {
            joint.axis = parseXyz(axisEl);
        }

        const limitEl = getFirstDirectChild(jointEl, 'limit');
        if (limitEl) {
            joint.limits.lower = parseFloat(limitEl.getAttribute('lower')) || 0;
            joint.limits.upper = parseFloat(limitEl.getAttribute('upper')) || 0;
            joint.limits.effort = parseFloat(limitEl.getAttribute('effort')) || 0;
            joint.limits.velocity = parseFloat(limitEl.getAttribute('velocity')) || 0;
        }

        joints.push(joint);
        childToJoint[joint.childLink] = joint;
    }

    // ── Find root link (a link that is never a child) ──
    const childLinks = new Set();
    for (let i = 0; i < joints.length; i++) {
        childLinks.add(joints[i].childLink);
    }
    const linkKeys = Object.keys(links);
    let rootLink = '';
    for (let i = 0; i < linkKeys.length; i++) {
        if (!childLinks.has(linkKeys[i])) {
            rootLink = linkKeys[i];
            break;
        }
    }

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
    const linkToParentJointId = {};
    linkToParentJointId[rootLink] = 0; // Universe

    // Map link names → SE3 transform relative to the parent joint frame
    const linkToJointTransform = {};
    linkToJointTransform[rootLink] = createIdentityTransform();

    // Index joints by parentLink for O(1) lookups instead of O(N) filtering
    const parentToChildJoints = {};
    for (let i = 0; i < joints.length; i++) {
        const j = joints[i];
        if (!parentToChildJoints[j.parentLink]) {
            parentToChildJoints[j.parentLink] = [];
        }
        parentToChildJoints[j.parentLink].push(j);
    }

    // BFS Queue
    const queue = [rootLink];
    const visited = new Set([rootLink]);

    while (queue.length > 0) {
        const parentLinkName = queue.shift();
        const parentJointId = linkToParentJointId[parentLinkName];

        const parentOffset = linkToJointTransform[parentLinkName] || createIdentityTransform();
        const childJoints = parentToChildJoints[parentLinkName] || [];

        for (let i = 0; i < childJoints.length; i++) {
            const joint = childJoints[i];
            if (visited.has(joint.childLink)) continue;
            visited.add(joint.childLink);

            // 1. Calculate absolute placement of this joint/link relative to the PARENT JOINT frame
            const jointOrigin = createTransformFromXyzRpy(joint.origin.xyz, joint.origin.rpy);
            const placement = composeTransforms(parentOffset, jointOrigin);

            // 2. Handle Joint Type
            if (joint.type === 'fixed') {
                linkToParentJointId[joint.childLink] = parentJointId;
                linkToJointTransform[joint.childLink] = placement;

                // Add inertia to the parent joint (transformed by placement)
                addBodyInertia(pin, model, links[joint.childLink], parentJointId, placement);
            } else {
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

                linkToParentJointId[joint.childLink] = jointId;
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

    const inertia = pin.Inertia.fromMassComInertia(
        linkData.mass,
        linkData.com,
        linkData.inertia
    );

    const placement = toPinocchioSE3(pin, transform);
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

    const cr = Math.cos(roll), sr = Math.sin(roll);
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const cy = Math.cos(yaw), sy = Math.sin(yaw);

    const R = [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr
    ];

    return { R, t: [x, y, z] };
}

function composeTransforms(T1, T2) {
    const R1 = T1.R;
    const R2 = T2.R;
    const t1 = T1.t;
    const t2 = T2.t;

    const R = [
        R1[0] * R2[0] + R1[1] * R2[3] + R1[2] * R2[6], R1[0] * R2[1] + R1[1] * R2[4] + R1[2] * R2[7], R1[0] * R2[2] + R1[1] * R2[5] + R1[2] * R2[8],
        R1[3] * R2[0] + R1[4] * R2[3] + R1[5] * R2[6], R1[3] * R2[1] + R1[4] * R2[4] + R1[5] * R2[7], R1[3] * R2[2] + R1[4] * R2[5] + R1[5] * R2[8],
        R1[6] * R2[0] + R1[7] * R2[3] + R1[8] * R2[6], R1[6] * R2[1] + R1[7] * R2[4] + R1[8] * R2[7], R1[6] * R2[2] + R1[7] * R2[5] + R1[8] * R2[8]
    ];

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
            return pin.JointModelFixed();

        default:
            console.warn(`Unknown joint type: ${type}, defaulting to revolute Z`);
            return pin.JointModelRZ();
    }
}

function parseNumbers(str) {
    const trimmed = str.trim();
    if (!trimmed) return [];
    const parts = trimmed.split(/\s+/);
    const result = new Array(parts.length);
    for (let i = 0; i < parts.length; i++) {
        result[i] = parseFloat(parts[i]) || 0;
    }
    return result;
}

function parseXyz(el) {
    const xyz = el.getAttribute('xyz');
    if (!xyz) return [0, 0, 0];
    const parts = xyz.trim().split(/\s+/);
    return [parseFloat(parts[0]) || 0, parseFloat(parts[1]) || 0, parseFloat(parts[2]) || 0];
}

function parseRpy(el) {
    const rpy = el.getAttribute('rpy');
    if (!rpy) return [0, 0, 0];
    const parts = rpy.trim().split(/\s+/);
    return [parseFloat(parts[0]) || 0, parseFloat(parts[1]) || 0, parseFloat(parts[2]) || 0];
}
