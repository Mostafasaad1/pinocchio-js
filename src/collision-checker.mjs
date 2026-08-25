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

                const min = col.geometry.min;
                const max = col.geometry.max;

                this.activeCollisions.push({
                    linkName: linkName,
                    jointId: linkMapping.jointId,
                    localOffset: T_col_joint,
                    localAABB: col.geometry,
                    cx: (max[0] + min[0]) / 2,
                    cy: (max[1] + min[1]) / 2,
                    cz: (max[2] + min[2]) / 2,
                    hx: (max[0] - min[0]) / 2,
                    hy: (max[1] - min[1]) / 2,
                    hz: (max[2] - min[2]) / 2,
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
            
            const R1 = jointPlacement.rotation;
            const t1 = jointPlacement.translation;
            const R2 = col.localOffset.R;
            const t2 = col.localOffset.t;
            
            const R0 = R1[0]*R2[0] + R1[1]*R2[3] + R1[2]*R2[6];
            const R1_val = R1[0]*R2[1] + R1[1]*R2[4] + R1[2]*R2[7];
            const R2_val = R1[0]*R2[2] + R1[1]*R2[5] + R1[2]*R2[8];

            const R3 = R1[3]*R2[0] + R1[4]*R2[3] + R1[5]*R2[6];
            const R4 = R1[3]*R2[1] + R1[4]*R2[4] + R1[5]*R2[7];
            const R5 = R1[3]*R2[2] + R1[4]*R2[5] + R1[5]*R2[8];

            const R6 = R1[6]*R2[0] + R1[7]*R2[3] + R1[8]*R2[6];
            const R7 = R1[6]*R2[1] + R1[7]*R2[4] + R1[8]*R2[7];
            const R8 = R1[6]*R2[2] + R1[7]*R2[5] + R1[8]*R2[8];

            const t0 = R1[0]*t2[0] + R1[1]*t2[1] + R1[2]*t2[2] + t1[0];
            const t1_val = R1[3]*t2[0] + R1[4]*t2[1] + R1[5]*t2[2] + t1[1];
            const t2_val = R1[6]*t2[0] + R1[7]*t2[1] + R1[8]*t2[2] + t1[2];
            
            const cx = col.cx, cy = col.cy, cz = col.cz;
            const hx = col.hx, hy = col.hy, hz = col.hz;
            
            const ncx = R0*cx + R1_val*cy + R2_val*cz + t0;
            const ncy = R3*cx + R4*cy + R5*cz + t1_val;
            const ncz = R6*cx + R7*cy + R8*cz + t2_val;
            
            const nhx = Math.abs(R0)*hx + Math.abs(R1_val)*hy + Math.abs(R2_val)*hz;
            const nhy = Math.abs(R3)*hx + Math.abs(R4)*hy + Math.abs(R5)*hz;
            const nhz = Math.abs(R6)*hx + Math.abs(R7)*hy + Math.abs(R8)*hz;
            
            const gmin = col.globalAABB.min;
            const gmax = col.globalAABB.max;

            gmin[0] = ncx - nhx;
            gmin[1] = ncy - nhy;
            gmin[2] = ncz - nhz;
            gmax[0] = ncx + nhx;
            gmax[1] = ncy + nhy;
            gmax[2] = ncz + nhz;
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
