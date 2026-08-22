/**
 * Pinocchio WASM — TypeScript Definitions
 * High-performance WebAssembly bindings for Pinocchio rigid body dynamics.
 */

export enum ReferenceFrame {
    WORLD = 0,
    LOCAL = 1,
    LOCAL_WORLD_ALIGNED = 2,
}

export interface EmbindObject {
    delete(): void;
}

export interface JointPlacement {
    translation: Float64Array;
    rotation: Float64Array;
}

export declare class SE3 implements EmbindObject {
    static identity(): SE3;
    static fromRotationTranslation(rot: Float64Array | ArrayLike<number>, trans: Float64Array | ArrayLike<number>): SE3;
    static fromXyzRpy(x: number, y: number, z: number, roll: number, pitch: number, yaw: number): SE3;
    delete(): void;
}

export declare class Inertia implements EmbindObject {
    static fromMassComInertia(
        mass: number,
        com: Float64Array | ArrayLike<number>,
        inertia: Float64Array | ArrayLike<number>
    ): Inertia;
    delete(): void;
}

export declare class JointModel implements EmbindObject {
    delete(): void;
}

export declare class Model implements EmbindObject {
    constructor();
    readonly nq: number;
    readonly nv: number;
    readonly njoints: number;
    readonly name: string;
    delete(): void;
}

export declare class Data implements EmbindObject {
    constructor(model: Model);
    getVelocity(jointId: number): Float64Array;
    getAcceleration(jointId: number): Float64Array;
    delete(): void;
}

export declare function JointModelRX(): JointModel;
export declare function JointModelRY(): JointModel;
export declare function JointModelRZ(): JointModel;
export declare function JointModelPX(): JointModel;
export declare function JointModelPY(): JointModel;
export declare function JointModelPZ(): JointModel;
export declare function JointModelRevoluteUnaligned(ax: number, ay: number, az: number): JointModel;
export declare function JointModelPrismaticUnaligned(ax: number, ay: number, az: number): JointModel;
export declare function JointModelFreeFlyer(): JointModel;
export declare function JointModelFixed(): JointModel;

export declare function addJoint(
    model: Model,
    parentId: number,
    joint: JointModel,
    placement: SE3,
    name: string
): number;

export declare function addJointWithLimits(
    model: Model,
    parentId: number,
    joint: JointModel,
    placement: SE3,
    name: string,
    maxEffort: Float64Array | ArrayLike<number>,
    maxVelocity: Float64Array | ArrayLike<number>,
    minConfig: Float64Array | ArrayLike<number>,
    maxConfig: Float64Array | ArrayLike<number>
): number;

export declare function appendBodyToJoint(
    model: Model,
    jointId: number,
    inertia: Inertia,
    bodyPlacement: SE3
): void;

export declare function getTau(data: Data): Float64Array;
export declare function getNle(data: Data): Float64Array;
export declare function getComAt(data: Data, idx: number): Float64Array;

export declare function rnea(
    model: Model,
    data: Data,
    q: Float64Array | ArrayLike<number>,
    v: Float64Array | ArrayLike<number>,
    a: Float64Array | ArrayLike<number>
): Float64Array;

export declare function aba(
    model: Model,
    data: Data,
    q: Float64Array | ArrayLike<number>,
    v: Float64Array | ArrayLike<number>,
    tau: Float64Array | ArrayLike<number>
): Float64Array;

export type ExternalForceMap = {
    [linkIndex: number]: Float64Array | ArrayLike<number>;
};

export declare function abaWithForces(
    model: Model,
    data: Data,
    q: Float64Array | ArrayLike<number>,
    v: Float64Array | ArrayLike<number>,
    tau: Float64Array | ArrayLike<number>,
    fext?: ExternalForceMap
): Float64Array;

export declare function crba(
    model: Model,
    data: Data,
    q: Float64Array | ArrayLike<number>
): Float64Array;

export declare function computeKineticEnergy(
    model: Model,
    data: Data,
    q: Float64Array | ArrayLike<number>,
    v: Float64Array | ArrayLike<number>
): number;

export declare function computePotentialEnergy(
    model: Model,
    data: Data,
    q: Float64Array | ArrayLike<number>
): number;

export declare function computeGeneralizedGravity(
    model: Model,
    data: Data,
    q: Float64Array | ArrayLike<number>
): Float64Array;

export declare function nonLinearEffects(
    model: Model,
    data: Data,
    q: Float64Array | ArrayLike<number>,
    v: Float64Array | ArrayLike<number>
): Float64Array;

export declare function forwardKinematics(
    model: Model,
    data: Data,
    q: Float64Array | ArrayLike<number>
): void;

export declare function forwardKinematicsQVA(
    model: Model,
    data: Data,
    q: Float64Array | ArrayLike<number>,
    v: Float64Array | ArrayLike<number>,
    a: Float64Array | ArrayLike<number>
): void;

export declare function updateFramePlacements(model: Model, data: Data): void;

export declare function getJointPlacement(data: Data, jointId: number): JointPlacement;

export declare function computeJointJacobians(
    model: Model,
    data: Data,
    q: Float64Array | ArrayLike<number>
): void;

export declare function getJointJacobian(
    model: Model,
    data: Data,
    jointId: number,
    refFrame: ReferenceFrame | number
): Float64Array;

export declare function centerOfMass(
    model: Model,
    data: Data,
    q: Float64Array | ArrayLike<number>
): Float64Array;

export declare function computeTotalMass(model: Model): number;
export declare function randomConfiguration(model: Model): Float64Array;
export declare function neutralConfiguration(model: Model): Float64Array;

export interface PinocchioModule {
    ReferenceFrame: typeof ReferenceFrame;
    SE3: typeof SE3;
    Inertia: typeof Inertia;
    JointModel: typeof JointModel;
    Model: typeof Model;
    Data: typeof Data;

    JointModelRX: typeof JointModelRX;
    JointModelRY: typeof JointModelRY;
    JointModelRZ: typeof JointModelRZ;
    JointModelPX: typeof JointModelPX;
    JointModelPY: typeof JointModelPY;
    JointModelPZ: typeof JointModelPZ;
    JointModelRevoluteUnaligned: typeof JointModelRevoluteUnaligned;
    JointModelPrismaticUnaligned: typeof JointModelPrismaticUnaligned;
    JointModelFreeFlyer: typeof JointModelFreeFlyer;
    JointModelFixed: typeof JointModelFixed;

    addJoint: typeof addJoint;
    addJointWithLimits: typeof addJointWithLimits;
    appendBodyToJoint: typeof appendBodyToJoint;

    getTau: typeof getTau;
    getNle: typeof getNle;
    getComAt: typeof getComAt;

    rnea: typeof rnea;
    aba: typeof aba;
    abaWithForces: typeof abaWithForces;
    crba: typeof crba;
    computeKineticEnergy: typeof computeKineticEnergy;
    computePotentialEnergy: typeof computePotentialEnergy;
    computeGeneralizedGravity: typeof computeGeneralizedGravity;
    nonLinearEffects: typeof nonLinearEffects;
    forwardKinematics: typeof forwardKinematics;
    forwardKinematicsQVA: typeof forwardKinematicsQVA;
    updateFramePlacements: typeof updateFramePlacements;
    getJointPlacement: typeof getJointPlacement;
    computeJointJacobians: typeof computeJointJacobians;
    getJointJacobian: typeof getJointJacobian;
    centerOfMass: typeof centerOfMass;
    computeTotalMass: typeof computeTotalMass;
    randomConfiguration: typeof randomConfiguration;
    neutralConfiguration: typeof neutralConfiguration;
}

declare function createPinocchioModule(moduleOverrides?: object): Promise<PinocchioModule>;

export default createPinocchioModule;
