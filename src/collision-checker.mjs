/**
 * @fileoverview CollisionChecker for JavaScript broad-phase checking.
 */

export class CollisionChecker {
    /**
     * Initializes the collision checker.
     * Extracts collision geometries from urdfData and prepares internal AABBs.
     * Automatically ignores adjacent parent-child link pairs.
     *
     * @param {Object} pin - The Pinocchio WASM module instance
     * @param {Object} model - The Pinocchio Model object
     * @param {Object} data - The Pinocchio Data object
     * @param {Object} urdfData - Parsed URDF data from urdf-parser
     */
    constructor(pin, model, data, urdfData) {
        this.pin = pin;
        this.model = model;
        this.data = data;
        this.urdfData = urdfData;
        this.ignoredPairs = new Set();
        this.activeCollisions = [];
        
        this._buildLinkMapping();

        for (const [linkName, link] of Object.entries(urdfData.links)) {
            if (!link.collisions || link.collisions.length === 0) continue;
            
            const linkMapping = this.linkToJoint[linkName];
            if (!linkMapping) continue;

            for (let i = 0; i < link.collisions.length; i++) {
                const col = link.collisions[i];
                const T_link = linkMapping.transform;
                const T_col_local = createTransformFromXyzRpy(col.origin.xyz, col.origin.rpy);
                const T_col_joint = composeTransforms(T_link, T_col_local);

                this.activeCollisions.push({
                    linkName: linkName,
                    jointId: linkMapping.jointId,
                    localOffset: T_col_joint,
                    localAABB: col.geometry,
                    globalAABB: { min: [0,0,0], max: [0,0,0] }
                });
            }
        }

        // Auto-ignore adjacent parent-child link pairs
        for (const joint of urdfData.joints) {
            this.ignoredPairs.add(this._pairKey(joint.parentLink, joint.childLink));
        }
    }

    _pairKey(linkA, linkB) {
        return linkA < linkB ? `${linkA}:::${linkB}` : `${linkB}:::${linkA}`;
    }

    _buildLinkMapping() {
        const { joints, rootLink } = this.urdfData;
        this.linkToJoint = {};
        
        this.linkToJoint[rootLink] = { jointId: 0, transform: createIdentityTransform() };
        
        const queue = [rootLink];
        const visited = new Set([rootLink]);
        let nextJointId = 1;
        
        while (queue.length > 0) {
            const parentLinkName = queue.shift();
            const parentJointId = this.linkToJoint[parentLinkName].jointId;
            const parentOffset = this.linkToJoint[parentLinkName].transform;

            const childJoints = joints.filter(j => j.parentLink === parentLinkName);

            for (const joint of childJoints) {
                if (visited.has(joint.childLink)) continue;
                visited.add(joint.childLink);

                const jointOrigin = createTransformFromXyzRpy(joint.origin.xyz, joint.origin.rpy);
                const placement = composeTransforms(parentOffset, jointOrigin);

                if (joint.type === 'fixed') {
                    this.linkToJoint[joint.childLink] = {
                        jointId: parentJointId,
                        transform: placement
                    };
                } else {
                    this.linkToJoint[joint.childLink] = {
                        jointId: nextJointId++,
                        transform: createIdentityTransform()
                    };
                }
                queue.push(joint.childLink);
            }
        }
    }

    /**
     * Specifies an array of link pairs to ignore during collision checking.
     * @param {Array<[string, string]>} pairs - Array of link name tuples, e.g., [['link1', 'link2'], ['base_link', 'link3']]
     */
    setIgnoredPairs(pairs) {
        for (const [linkA, linkB] of pairs) {
            this.ignoredPairs.add(this._pairKey(linkA, linkB));
        }
    }

    /**
     * Updates the global positions of all bounding volumes based on the given joint configuration.
     * Computes forward kinematics internally to determine link placements.
     * @param {Float64Array} q - Float64Array representing the joint configuration.
     */
    updateCollisions(q) {
        this.pin.forwardKinematics(this.model, this.data, q);
        
        for (let i = 0; i < this.activeCollisions.length; i++) {
            const col = this.activeCollisions[i];
            const jointPlacement = this.pin.getJointPlacement(this.data, col.jointId);
            
            const R_joint = Array.from(jointPlacement.rotation);
            const t_joint = Array.from(jointPlacement.translation);
            const T_joint = { R: R_joint, t: t_joint };
            
            const T_abs = composeTransforms(T_joint, col.localOffset);
            
            const min = col.localAABB.min;
            const max = col.localAABB.max;
            
            const cx = (max[0] + min[0]) / 2;
            const cy = (max[1] + min[1]) / 2;
            const cz = (max[2] + min[2]) / 2;
            
            const hx = (max[0] - min[0]) / 2;
            const hy = (max[1] - min[1]) / 2;
            const hz = (max[2] - min[2]) / 2;
            
            const R = T_abs.R;
            const t = T_abs.t;
            const ncx = R[0]*cx + R[1]*cy + R[2]*cz + t[0];
            const ncy = R[3]*cx + R[4]*cy + R[5]*cz + t[1];
            const ncz = R[6]*cx + R[7]*cy + R[8]*cz + t[2];
            
            const absR = R.map(Math.abs);
            const nhx = absR[0]*hx + absR[1]*hy + absR[2]*hz;
            const nhy = absR[3]*hx + absR[4]*hy + absR[5]*hz;
            const nhz = absR[6]*hx + absR[7]*hy + absR[8]*hz;
            
            col.globalAABB.min[0] = ncx - nhx;
            col.globalAABB.min[1] = ncy - nhy;
            col.globalAABB.min[2] = ncz - nhz;
            col.globalAABB.max[0] = ncx + nhx;
            col.globalAABB.max[1] = ncy + nhy;
            col.globalAABB.max[2] = ncz + nhz;
        }
    }

    /**
     * Checks for broad-phase AABB collisions between all active (non-ignored) link pairs.
     * Must be called after `updateCollisions`.
     * @returns {{ hasCollision: boolean, contacts: Array<{linkA: string, linkB: string}> }} CollisionResult containing a boolean flag and a list of intersecting pairs.
     */
    checkCollisions() {
        const result = {
            hasCollision: false,
            contacts: []
        };

        for (let i = 0; i < this.activeCollisions.length; i++) {
            for (let j = i + 1; j < this.activeCollisions.length; j++) {
                const colA = this.activeCollisions[i];
                const colB = this.activeCollisions[j];
                
                if (colA.linkName === colB.linkName) continue;
                
                if (this.ignoredPairs.has(this._pairKey(colA.linkName, colB.linkName))) continue;
                
                const a = colA.globalAABB;
                const b = colB.globalAABB;
                
                if (a.min[0] <= b.max[0] && a.max[0] >= b.min[0] &&
                    a.min[1] <= b.max[1] && a.max[1] >= b.min[1] &&
                    a.min[2] <= b.max[2] && a.max[2] >= b.min[2]) {
                    
                    result.hasCollision = true;
                    result.contacts.push({ linkA: colA.linkName, linkB: colB.linkName });
                }
            }
        }
        
        return result;
    }
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
