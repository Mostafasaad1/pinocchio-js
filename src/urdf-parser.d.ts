/**
 * Pinocchio WASM — URDF Parser TypeScript Definitions
 * Type definitions for parsing URDF XML strings and constructing Pinocchio models.
 */

import { Model, PinocchioModule } from './pinocchio';

export interface AABB {
    min: [number, number, number];
    max: [number, number, number];
}

export interface URDFCollision {
    origin: URDFOrigin;
    geometry: AABB;
}

export interface URDFLink {
    name: string;
    mass: number;
    com: [number, number, number];
    inertia: [number, number, number, number, number, number];
    collisions?: URDFCollision[];
}

export interface URDFOrigin {
    xyz: [number, number, number];
    rpy: [number, number, number];
}

export interface URDFLimits {
    lower: number;
    upper: number;
    effort: number;
    velocity: number;
}

export interface URDFJoint {
    name: string;
    type: string;
    parentLink: string;
    childLink: string;
    origin: URDFOrigin;
    axis: [number, number, number];
    limits: URDFLimits;
}

export interface URDFData {
    robotName: string;
    links: Record<string, URDFLink>;
    joints: URDFJoint[];
    rootLink: string;
}

/**
 * Parse a URDF XML string into a structured JavaScript object.
 *
 * @param urdfString - The URDF XML content
 * @returns The parsed URDF data including robotName, links, joints, and rootLink
 */
export function parseURDF(urdfString: string): URDFData;

/**
 * Build a Pinocchio Model from parsed URDF data.
 *
 * Implements "Fixed Joint Reduction" in JavaScript:
 * URDF Fixed joints are NOT added to the Pinocchio model as joints.
 * Instead, their kinematic transformation is composed into the placement
 * of the child link (and its subsequent children).
 *
 * @param pin - The Pinocchio WASM module instance
 * @param urdfData - Output of parseURDF()
 * @returns The constructed Pinocchio Model
 */
export function buildPinocchioModel(pin: PinocchioModule, urdfData: URDFData): Model;
