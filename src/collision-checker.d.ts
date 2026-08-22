export interface AABB {
  min: [number, number, number];
  max: [number, number, number];
}

export interface CollisionResult {
  hasCollision: boolean;
  contacts: Array<{ linkA: string; linkB: string }>;
}

export class CollisionChecker {
  /**
   * Initializes the collision checker.
   * Extracts collision geometries from urdfData and prepares internal AABBs.
   * Automatically ignores adjacent parent-child link pairs.
   */
  constructor(pin: any, model: any, data: any, urdfData: any);

  /**
   * Specifies an array of link pairs to ignore during collision checking.
   * @param pairs Array of link name tuples, e.g., [['link1', 'link2'], ['base_link', 'link3']]
   */
  setIgnoredPairs(pairs: Array<[string, string]>): void;

  /**
   * Updates the global positions of all bounding volumes based on the given joint configuration.
   * This computes forward kinematics internally to determine link placements.
   * @param q Float64Array representing the joint configuration.
   */
  updateCollisions(q: Float64Array): void;

  /**
   * Checks for broad-phase AABB collisions between all active (non-ignored) link pairs.
   * Must be called after `updateCollisions`.
   * @returns CollisionResult containing a boolean flag and a list of intersecting pairs.
   */
  checkCollisions(): CollisionResult;
}
