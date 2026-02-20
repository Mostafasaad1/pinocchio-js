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

  const errorNode = doc.querySelector('parsererror');
  if (errorNode) {
    throw new Error(`URDF parse error: ${errorNode.textContent}`);
  }

  const robot = doc.querySelector('robot');
  if (!robot) throw new Error('No <robot> element found in URDF');

  const robotName = robot.getAttribute('name') || 'unnamed';

  // ── Parse links ──
  const links = {};
  for (const linkEl of robot.querySelectorAll('link')) {
    const name = linkEl.getAttribute('name');
    const link = { name, mass: 0, com: [0, 0, 0], inertia: [0, 0, 0, 0, 0, 0] };

    const inertialEl = linkEl.querySelector('inertial');
    if (inertialEl) {
      // Mass
      const massEl = inertialEl.querySelector('mass');
      if (massEl) {
        link.mass = parseFloat(massEl.getAttribute('value')) || 0;
      }

      // Center of mass origin
      const originEl = inertialEl.querySelector('origin');
      if (originEl) {
        link.com = parseXyz(originEl);
      }

      // Inertia tensor (6 unique elements)
      const inertiaEl = inertialEl.querySelector('inertia');
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

    links[name] = link;
  }

  // ── Parse joints ──
  const joints = [];
  const childToJoint = {};  // child_link_name → joint

  for (const jointEl of robot.querySelectorAll('joint')) {
    const joint = {
      name: jointEl.getAttribute('name'),
      type: jointEl.getAttribute('type'),
      parentLink: '',
      childLink: '',
      origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
      axis: [0, 0, 1],  // default Z axis
      limits: { lower: 0, upper: 0, effort: 0, velocity: 0 }
    };

    const parentEl = jointEl.querySelector('parent');
    if (parentEl) joint.parentLink = parentEl.getAttribute('link');

    const childEl = jointEl.querySelector('child');
    if (childEl) joint.childLink = childEl.getAttribute('link');

    const originEl = jointEl.querySelector('origin');
    if (originEl) {
      joint.origin.xyz = parseXyz(originEl);
      joint.origin.rpy = parseRpy(originEl);
    }

    const axisEl = jointEl.querySelector('axis');
    if (axisEl) {
      joint.axis = parseXyz(axisEl);
    }

    const limitEl = jointEl.querySelector('limit');
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
  const childLinks = new Set(joints.map(j => j.childLink));
  const rootLink = Object.keys(links).find(name => !childLinks.has(name)) || '';

  return { robotName, links, joints, rootLink };
}


/**
 * Build a Pinocchio Model from parsed URDF data.
 *
 * @param {Object} pin - The Pinocchio WASM module
 * @param {Object} urdfData - Output of parseURDF()
 * @returns {Object} The constructed Pinocchio Model
 */
export function buildPinocchioModel(pin, urdfData) {
  const { links, joints, rootLink } = urdfData;
  const model = new pin.Model();

  // Map link names → Pinocchio joint indices
  const linkToJointId = {};
  linkToJointId[rootLink] = 0;  // Root link maps to universe joint (index 0)

  // BFS from root to process joints in tree order
  const queue = [rootLink];
  const visited = new Set([rootLink]);

  while (queue.length > 0) {
    const parentLinkName = queue.shift();
    const parentJointId = linkToJointId[parentLinkName];

    // Find all joints where this link is the parent
    const childJoints = joints.filter(j => j.parentLink === parentLinkName);

    for (const joint of childJoints) {
      if (visited.has(joint.childLink)) continue;
      visited.add(joint.childLink);

      // Create SE3 placement from joint origin
      const [x, y, z] = joint.origin.xyz;
      const [roll, pitch, yaw] = joint.origin.rpy;
      const placement = pin.SE3.fromXyzRpy(x, y, z, roll, pitch, yaw);

      // Create joint model based on type + axis
      const jointModel = createJointModel(pin, joint.type, joint.axis);

      // Add joint to model
      let jointId;
      if (joint.type === 'fixed') {
        // For fixed joints, we still need to track them but they add no DOFs
        // Use addJoint with a fixed representation
        jointId = pin.addJoint(model, parentJointId, jointModel, placement, joint.name);
      } else if (joint.limits.effort > 0 || joint.limits.velocity > 0) {
        const nq = (joint.type === 'floating') ? 7 : 1;
        const nv = (joint.type === 'floating') ? 6 : 1;
        jointId = pin.addJointWithLimits(
          model, parentJointId, jointModel, placement, joint.name,
          new Float64Array(nv).fill(joint.limits.effort),
          new Float64Array(nv).fill(joint.limits.velocity),
          new Float64Array(nq).fill(joint.limits.lower),
          new Float64Array(nq).fill(joint.limits.upper)
        );
      } else {
        jointId = pin.addJoint(model, parentJointId, jointModel, placement, joint.name);
      }

      // Add body inertia from child link
      const childLink = links[joint.childLink];
      if (childLink && childLink.mass > 0) {
        const inertia = pin.Inertia.fromMassComInertia(
          childLink.mass,
          childLink.com,
          childLink.inertia
        );
        pin.appendBodyToJoint(model, jointId, inertia, pin.SE3.identity());
      }

      linkToJointId[joint.childLink] = jointId;
      queue.push(joint.childLink);
    }
  }

  return model;
}


// ─── Internal Helpers ───────────────────────────────────────────

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
