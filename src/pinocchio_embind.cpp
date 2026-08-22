// ──────────────────────────────────────────────────────────────────
// Pinocchio WASM — Embind Wrapper
// Exposes Pinocchio's core C++ API to JavaScript via Emscripten Embind.
// ──────────────────────────────────────────────────────────────────

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <stdexcept>
#include <string>
#include <vector>

// Pinocchio WASM config - must be included before pinocchio headers
// to define missing macros like PINOCCHIO_DEPRECATED_MESSAGE
#include "pinocchio_config.hpp"

// Pinocchio headers
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/energy.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>

using namespace emscripten;

// ─── Type Aliases ────────────────────────────────────────────────

using Model   = pinocchio::Model;
using Data    = pinocchio::Data;
using SE3     = pinocchio::SE3;
using Inertia = pinocchio::Inertia;
using JointIndex = pinocchio::JointIndex;
using FrameIndex = pinocchio::FrameIndex;

using VectorXd  = Eigen::VectorXd;
using Vector3d   = Eigen::Vector3d;
using Matrix3d   = Eigen::Matrix3d;
using MatrixXd   = Eigen::MatrixXd;

// ─── Eigen ↔ JavaScript Helpers ─────────────────────────────────

/**
 * Convert a JS Float64Array (or regular Array) to Eigen::VectorXd.
 */
VectorXd jsToVectorXd(const val& arr) {
    const unsigned len = arr["length"].as<unsigned>();
    VectorXd v(len);
    for (unsigned i = 0; i < len; ++i)
        v[i] = arr[i].as<double>();
    return v;
}

/**
 * Convert Eigen::VectorXd to a JS Float64Array (copy).
 */
val vectorXdToJs(const VectorXd& v) {
    val result = val::global("Float64Array").new_(v.size());
    for (Eigen::Index i = 0; i < v.size(); ++i)
        result.set(i, val(v[i]));
    return result;
}

/**
 * Convert a 3-element JS array to Eigen::Vector3d.
 */
Vector3d jsToVector3d(const val& arr) {
    return Vector3d(
        arr[0].as<double>(),
        arr[1].as<double>(),
        arr[2].as<double>()
    );
}

/**
 * Convert Eigen::Vector3d to a JS array [x, y, z].
 */
val vector3dToJs(const Vector3d& v) {
    val result = val::global("Float64Array").new_(3);
    result.set(0, val(v[0]));
    result.set(1, val(v[1]));
    result.set(2, val(v[2]));
    return result;
}

/**
 * Convert a 6-element JS array to Eigen::Matrix<double,6,1>.
 */
Eigen::Matrix<double,6,1> jsToVector6d(const val& arr) {
    Eigen::Matrix<double,6,1> v;
    for (int i = 0; i < 6; ++i)
        v[i] = arr[i].as<double>();
    return v;
}

/**
 * Convert a flat 9-element JS array to Eigen::Matrix3d (row-major input).
 */
Matrix3d jsToMatrix3d(const val& arr) {
    Matrix3d m;
    m(0,0) = arr[0].as<double>(); m(0,1) = arr[1].as<double>(); m(0,2) = arr[2].as<double>();
    m(1,0) = arr[3].as<double>(); m(1,1) = arr[4].as<double>(); m(1,2) = arr[5].as<double>();
    m(2,0) = arr[6].as<double>(); m(2,1) = arr[7].as<double>(); m(2,2) = arr[8].as<double>();
    return m;
}

/**
 * Convert a flat MatrixXd to a JS Float64Array (column-major).
 */
val matrixXdToJs(const MatrixXd& m) {
    val result = val::global("Float64Array").new_(m.rows() * m.cols());
    int idx = 0;
    for (Eigen::Index j = 0; j < m.cols(); ++j)
        for (Eigen::Index i = 0; i < m.rows(); ++i)
            result.set(idx++, val(m(i,j)));
    return result;
}

/**
 * Convert a JS External Force Map ({ [linkIndex: number]: Float64Array(6) })
 * to a PINOCCHIO_ALIGNED_STD_VECTOR(pinocchio::Force) of size model.njoints.
 * Validates index bounds and wrench dimensions.
 * Duplicate keys are accumulated additively.
 */
PINOCCHIO_ALIGNED_STD_VECTOR(pinocchio::Force) jsToForceVector(const Model& model, const val& fext_js) {
    PINOCCHIO_ALIGNED_STD_VECTOR(pinocchio::Force) fext((size_t)model.njoints, pinocchio::Force::Zero());
    if (fext_js.isUndefined() || fext_js.isNull()) {
        return fext;
    }

    val keys = val::global("Object").call<val>("keys", fext_js);
    const unsigned n = keys["length"].as<unsigned>();
    for (unsigned k = 0; k < n; ++k) {
        std::string keyStr = keys[k].as<std::string>();
        int idx = 0;
        try {
            idx = std::stoi(keyStr);
        } catch (...) {
            std::string msg = "Link index " + keyStr + " is out of range (0.." + std::to_string(model.njoints - 1) + ")";
            EM_ASM({ throw new RangeError(UTF8ToString($0)); }, msg.c_str());
            throw std::out_of_range(msg);
        }

        if (idx < 0 || idx >= model.njoints) {
            std::string msg = "Link index " + std::to_string(idx) + " is out of range (0.." + std::to_string(model.njoints - 1) + ")";
            EM_ASM({ throw new RangeError(UTF8ToString($0)); }, msg.c_str());
            throw std::out_of_range(msg);
        }

        val w = fext_js[keys[k]];
        if (w.isUndefined() || w.isNull() || w["length"].isUndefined()) {
            std::string msg = "Wrench at link " + std::to_string(idx) + " must have 6 components, got 0";
            EM_ASM({ throw new RangeError(UTF8ToString($0)); }, msg.c_str());
            throw std::out_of_range(msg);
        }
        unsigned len = w["length"].as<unsigned>();
        if (len != 6) {
            std::string msg = "Wrench at link " + std::to_string(idx) + " must have 6 components, got " + std::to_string(len);
            EM_ASM({ throw new RangeError(UTF8ToString($0)); }, msg.c_str());
            throw std::out_of_range(msg);
        }
        Vector3d torque(w[0].as<double>(), w[1].as<double>(), w[2].as<double>());
        Vector3d linear(w[3].as<double>(), w[4].as<double>(), w[5].as<double>());
        fext[(size_t)idx] += pinocchio::Force(linear, torque);
    }
    return fext;
}

// ─── SE3 Factories ──────────────────────────────────────────────

/**
 * Create SE3 from rotation matrix (9 floats, row-major) + translation (3 floats).
 */
SE3 se3FromRotationTranslation(const val& rot, const val& trans) {
    return SE3(jsToMatrix3d(rot), jsToVector3d(trans));
}

/**
 * Create SE3 from xyz + rpy (URDF convention: fixed-axis XYZ = roll, pitch, yaw).
 */
SE3 se3FromXyzRpy(double x, double y, double z,
                  double roll, double pitch, double yaw) {
    Matrix3d R;
    // ZYX Euler convention (URDF standard: rpy = roll about X, pitch about Y, yaw about Z)
    double cr = cos(roll),  sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw),   sy = sin(yaw);

    R(0,0) = cy*cp;  R(0,1) = cy*sp*sr - sy*cr;  R(0,2) = cy*sp*cr + sy*sr;
    R(1,0) = sy*cp;  R(1,1) = sy*sp*sr + cy*cr;  R(1,2) = sy*sp*cr - cy*sr;
    R(2,0) = -sp;    R(2,1) = cp*sr;              R(2,2) = cp*cr;

    return SE3(R, Vector3d(x, y, z));
}

/**
 * SE3 identity.
 */
SE3 se3Identity() {
    return SE3::Identity();
}

// ─── Inertia Factories ──────────────────────────────────────────

/**
 * Create Inertia from mass, center of mass [3], and inertia matrix [6] (Ixx, Ixy, Ixz, Iyy, Iyz, Izz).
 */
Inertia inertiaFromMassComInertia(double mass, const val& com_js, const val& inertia_js) {
    Vector3d com = jsToVector3d(com_js);

    // Symmetric 3x3 inertia matrix from 6 unique elements
    double Ixx = inertia_js[0].as<double>();
    double Ixy = inertia_js[1].as<double>();
    double Ixz = inertia_js[2].as<double>();
    double Iyy = inertia_js[3].as<double>();
    double Iyz = inertia_js[4].as<double>();
    double Izz = inertia_js[5].as<double>();

    Matrix3d I;
    I << Ixx, Ixy, Ixz,
         Ixy, Iyy, Iyz,
         Ixz, Iyz, Izz;

    return Inertia(mass, com, I);
}

// ─── Joint Model Factories ──────────────────────────────────────

// Wrapper types to hold joint models for addJoint calls.
// We need this because Pinocchio's addJoint is templated on the joint model type,
// but Embind can't handle boost::variant directly.

struct JointModelWrapper {
    enum Type {
        RX, RY, RZ,
        PX, PY, PZ,
        REVOLUTE_UNALIGNED,
        PRISMATIC_UNALIGNED,
        FREE_FLYER,
        FIXED
    };

    Type type;
    Vector3d axis;  // Only used for UNALIGNED types

    JointModelWrapper(Type t) : type(t), axis(Vector3d::UnitX()) {}
    JointModelWrapper(Type t, const Vector3d& a) : type(t), axis(a.normalized()) {}
};

JointModelWrapper makeJointModelRX() { return JointModelWrapper(JointModelWrapper::RX); }
JointModelWrapper makeJointModelRY() { return JointModelWrapper(JointModelWrapper::RY); }
JointModelWrapper makeJointModelRZ() { return JointModelWrapper(JointModelWrapper::RZ); }
JointModelWrapper makeJointModelPX() { return JointModelWrapper(JointModelWrapper::PX); }
JointModelWrapper makeJointModelPY() { return JointModelWrapper(JointModelWrapper::PY); }
JointModelWrapper makeJointModelPZ() { return JointModelWrapper(JointModelWrapper::PZ); }

JointModelWrapper makeJointModelRevoluteUnaligned(double ax, double ay, double az) {
    return JointModelWrapper(JointModelWrapper::REVOLUTE_UNALIGNED, Vector3d(ax, ay, az));
}
JointModelWrapper makeJointModelPrismaticUnaligned(double ax, double ay, double az) {
    return JointModelWrapper(JointModelWrapper::PRISMATIC_UNALIGNED, Vector3d(ax, ay, az));
}
JointModelWrapper makeJointModelFreeFlyer() { return JointModelWrapper(JointModelWrapper::FREE_FLYER); }
JointModelWrapper makeJointModelFixed() { return JointModelWrapper(JointModelWrapper::FIXED); }

// ─── Model Wrapper Functions ────────────────────────────────────

/**
 * Add a joint to the model.
 * Returns the JointIndex of the newly added joint.
 */
JointIndex modelAddJoint(Model& model,
                         JointIndex parentId,
                         const JointModelWrapper& joint,
                         const SE3& placement,
                         const std::string& name) {
    JointIndex jointId = 0;
    switch (joint.type) {
        case JointModelWrapper::RX:
            jointId = model.addJoint(parentId, pinocchio::JointModelRX(), placement, name);
            break;
        case JointModelWrapper::RY:
            jointId = model.addJoint(parentId, pinocchio::JointModelRY(), placement, name);
            break;
        case JointModelWrapper::RZ:
            jointId = model.addJoint(parentId, pinocchio::JointModelRZ(), placement, name);
            break;
        case JointModelWrapper::PX:
            jointId = model.addJoint(parentId, pinocchio::JointModelPX(), placement, name);
            break;
        case JointModelWrapper::PY:
            jointId = model.addJoint(parentId, pinocchio::JointModelPY(), placement, name);
            break;
        case JointModelWrapper::PZ:
            jointId = model.addJoint(parentId, pinocchio::JointModelPZ(), placement, name);
            break;
        case JointModelWrapper::REVOLUTE_UNALIGNED:
            jointId = model.addJoint(parentId,
                pinocchio::JointModelRevoluteUnaligned(joint.axis),
                placement, name);
            break;
        case JointModelWrapper::PRISMATIC_UNALIGNED:
            jointId = model.addJoint(parentId,
                pinocchio::JointModelPrismaticUnaligned(joint.axis),
                placement, name);
            break;
        case JointModelWrapper::FREE_FLYER:
            jointId = model.addJoint(parentId, pinocchio::JointModelFreeFlyer(), placement, name);
            break;
        case JointModelWrapper::FIXED:
            jointId = model.addJoint(parentId,
                pinocchio::JointModelComposite(0),
                placement, name);
            break;
        default:
            return 0;
    }
    model.addJointFrame(jointId);
    return jointId;
}

/**
 * Add a joint with limits.
 */
JointIndex modelAddJointWithLimits(Model& model,
                                   JointIndex parentId,
                                   const JointModelWrapper& joint,
                                   const SE3& placement,
                                   const std::string& name,
                                   const val& maxEffort_js,
                                   const val& maxVelocity_js,
                                   const val& minConfig_js,
                                   const val& maxConfig_js) {
    VectorXd maxEffort = jsToVectorXd(maxEffort_js);
    VectorXd maxVelocity = jsToVectorXd(maxVelocity_js);
    VectorXd minConfig = jsToVectorXd(minConfig_js);
    VectorXd maxConfig = jsToVectorXd(maxConfig_js);

    JointIndex jointId = 0;
    switch (joint.type) {
        case JointModelWrapper::RX:
            jointId = model.addJoint(parentId, pinocchio::JointModelRX(), placement, name, maxEffort, maxVelocity, minConfig, maxConfig);
            break;
        case JointModelWrapper::RY:
            jointId = model.addJoint(parentId, pinocchio::JointModelRY(), placement, name, maxEffort, maxVelocity, minConfig, maxConfig);
            break;
        case JointModelWrapper::RZ:
            jointId = model.addJoint(parentId, pinocchio::JointModelRZ(), placement, name, maxEffort, maxVelocity, minConfig, maxConfig);
            break;
        case JointModelWrapper::PX:
            jointId = model.addJoint(parentId, pinocchio::JointModelPX(), placement, name, maxEffort, maxVelocity, minConfig, maxConfig);
            break;
        case JointModelWrapper::PY:
            jointId = model.addJoint(parentId, pinocchio::JointModelPY(), placement, name, maxEffort, maxVelocity, minConfig, maxConfig);
            break;
        case JointModelWrapper::PZ:
            jointId = model.addJoint(parentId, pinocchio::JointModelPZ(), placement, name, maxEffort, maxVelocity, minConfig, maxConfig);
            break;
        case JointModelWrapper::REVOLUTE_UNALIGNED:
            jointId = model.addJoint(parentId, pinocchio::JointModelRevoluteUnaligned(joint.axis), placement, name, maxEffort, maxVelocity, minConfig, maxConfig);
            break;
        case JointModelWrapper::PRISMATIC_UNALIGNED:
            jointId = model.addJoint(parentId, pinocchio::JointModelPrismaticUnaligned(joint.axis), placement, name, maxEffort, maxVelocity, minConfig, maxConfig);
            break;
        case JointModelWrapper::FREE_FLYER:
            jointId = model.addJoint(parentId, pinocchio::JointModelFreeFlyer(), placement, name, maxEffort, maxVelocity, minConfig, maxConfig);
            break;
        case JointModelWrapper::FIXED:
            jointId = model.addJoint(parentId, pinocchio::JointModelComposite(0), placement, name, maxEffort, maxVelocity, minConfig, maxConfig);
            break;
        default:
            return 0;
    }
    model.addJointFrame(jointId);
    return jointId;
}

/**
 * Append body inertia to an existing joint.
 */
void modelAppendBodyToJoint(Model& model,
                            JointIndex jointId,
                            const Inertia& inertia,
                            const SE3& bodyPlacement) {
    model.appendBodyToJoint(jointId, inertia, bodyPlacement);
}

// ─── Algorithm Wrappers ─────────────────────────────────────────

val rnea_js(Model& model, Data& data,
            const val& q_js, const val& v_js, const val& a_js) {
    VectorXd q = jsToVectorXd(q_js);
    VectorXd v = jsToVectorXd(v_js);
    VectorXd a = jsToVectorXd(a_js);
    pinocchio::rnea(model, data, q, v, a);
    return vectorXdToJs(data.tau);
}

val aba_js(Model& model, Data& data,
            const val& q_js, const val& v_js, const val& tau_js) {
    VectorXd q = jsToVectorXd(q_js);
    VectorXd v = jsToVectorXd(v_js);
    VectorXd tau = jsToVectorXd(tau_js);
    pinocchio::aba(model, data, q, v, tau);
    return vectorXdToJs(data.ddq);
}

val aba_fext_js(Model& model, Data& data,
                const val& q_js, const val& v_js, const val& tau_js,
                const val& fext_js) {
    VectorXd q = jsToVectorXd(q_js);
    VectorXd v = jsToVectorXd(v_js);
    VectorXd tau = jsToVectorXd(tau_js);

    if (fext_js.isUndefined() || fext_js.isNull()) {
        pinocchio::aba(model, data, q, v, tau);
        return vectorXdToJs(data.ddq);
    }

    val keys = val::global("Object").call<val>("keys", fext_js);
    if (keys["length"].as<unsigned>() == 0) {
        pinocchio::aba(model, data, q, v, tau);
        return vectorXdToJs(data.ddq);
    }

    PINOCCHIO_ALIGNED_STD_VECTOR(pinocchio::Force) fext = jsToForceVector(model, fext_js);
    pinocchio::aba(model, data, q, v, tau, fext);
    return vectorXdToJs(data.ddq);
}

val aba_with_forces_default_js(Model& model, Data& data,
                               const val& q_js, const val& v_js, const val& tau_js) {
    return aba_js(model, data, q_js, v_js, tau_js);
}

val crba_js(Model& model, Data& data, const val& q_js) {
    VectorXd q = jsToVectorXd(q_js);
    pinocchio::crba(model, data, q);
    // Pinocchio's crba only fills the upper triangle, we need to symmetrize it
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    return matrixXdToJs(data.M);
}

double computeKineticEnergy_js(Model& model, Data& data, const val& q_js, const val& v_js) {
    VectorXd q = jsToVectorXd(q_js);
    VectorXd v = jsToVectorXd(v_js);
    return pinocchio::computeKineticEnergy(model, data, q, v);
}

double computePotentialEnergy_js(Model& model, Data& data, const val& q_js) {
    VectorXd q = jsToVectorXd(q_js);
    return pinocchio::computePotentialEnergy(model, data, q);
}

val computeGeneralizedGravity_js(Model& model, Data& data, const val& q_js) {
    VectorXd q = jsToVectorXd(q_js);
    pinocchio::computeGeneralizedGravity(model, data, q);
    return vectorXdToJs(data.g);
}

val nonLinearEffects_js(Model& model, Data& data, const val& q_js, const val& v_js) {
    VectorXd q = jsToVectorXd(q_js);
    VectorXd v = jsToVectorXd(v_js);
    pinocchio::nonLinearEffects(model, data, q, v);
    return vectorXdToJs(data.nle);
}

void forwardKinematics_js(Model& model, Data& data, const val& q_js) {
    VectorXd q = jsToVectorXd(q_js);
    pinocchio::forwardKinematics(model, data, q);
}

void forwardKinematicsQVA_js(Model& model, Data& data, const val& q_js, const val& v_js, const val& a_js) {
    VectorXd q = jsToVectorXd(q_js);
    VectorXd v = jsToVectorXd(v_js);
    VectorXd a = jsToVectorXd(a_js);
    pinocchio::forwardKinematics(model, data, q, v, a);
}

void computeJointJacobians_js(Model& model, Data& data, const val& q_js) {
    VectorXd q = jsToVectorXd(q_js);
    pinocchio::computeJointJacobians(model, data, q);
}

val getJointJacobian_js(const Model& model, Data& data,
                        JointIndex jointId, int refFrame) {
    pinocchio::ReferenceFrame rf = static_cast<pinocchio::ReferenceFrame>(refFrame);
    MatrixXd J = MatrixXd::Zero(6, model.nv);
    pinocchio::getJointJacobian(model, data, jointId, rf, J);
    return matrixXdToJs(J);
}

val computeFrameJacobian_js(Model& model, Data& data, const val& q_js,
                            FrameIndex frameId, int refFrame) {
    if (frameId >= model.frames.size()) {
        std::string msg = "Invalid frame ID";
        EM_ASM({ throw new RangeError(UTF8ToString($0)); }, msg.c_str());
        throw std::out_of_range(msg);
    }
    VectorXd q = jsToVectorXd(q_js);
    pinocchio::ReferenceFrame rf = static_cast<pinocchio::ReferenceFrame>(refFrame);
    MatrixXd J = MatrixXd::Zero(6, model.nv);
    pinocchio::computeFrameJacobian(model, data, q, frameId, rf, J);
    return matrixXdToJs(J);
}

val getFrameJacobian_js(const Model& model, Data& data,
                        FrameIndex frameId, int refFrame) {
    if (frameId >= model.frames.size()) {
        std::string msg = "Invalid frame ID";
        EM_ASM({ throw new RangeError(UTF8ToString($0)); }, msg.c_str());
        throw std::out_of_range(msg);
    }
    pinocchio::ReferenceFrame rf = static_cast<pinocchio::ReferenceFrame>(refFrame);
    MatrixXd J = MatrixXd::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, frameId, rf, J);
    return matrixXdToJs(J);
}

val getFrameVelocity_js(const Model& model, const Data& data,
                        FrameIndex frameId, int refFrame) {
    if (frameId >= model.frames.size()) {
        std::string msg = "Invalid frame ID";
        EM_ASM({ throw new RangeError(UTF8ToString($0)); }, msg.c_str());
        throw std::out_of_range(msg);
    }
    pinocchio::ReferenceFrame rf = static_cast<pinocchio::ReferenceFrame>(refFrame);
    VectorXd v = pinocchio::getFrameVelocity(model, data, frameId, rf).toVector();
    return vectorXdToJs(v);
}

void getFrameAcceleration_js(
    const Model& model, 
    const Data& data, 
    const std::string& frameName, 
    int refFrame, 
    val outArray
) {
    if (!model.existFrame(frameName)) {
        std::string msg = "Frame '" + frameName + "' not found in model.";
        EM_ASM({ throw new Error(UTF8ToString($0)); }, msg.c_str());
        throw std::invalid_argument(msg);
    }

    // Heuristic to check if forwardKinematicsQVA was called:
    // If all joint velocities and accelerations are exactly zero, it is likely uninitialized.
    // This provides a safety check for the most common error case.
    bool is_initialized = false;
    for (size_t i = 1; i < data.v.size(); ++i) {
        if (!data.v[i].toVector().isZero() || !data.a[i].toVector().isZero()) {
            is_initialized = true;
            break;
        }
    }
    
    if (model.njoints > 1 && !is_initialized) {
        std::string msg = "forwardKinematicsQVA must be called before getFrameAcceleration";
        EM_ASM({ throw new Error(UTF8ToString($0)); }, msg.c_str());
        throw std::logic_error(msg);
    }

    FrameIndex frameId = model.getFrameId(frameName);
    pinocchio::ReferenceFrame rf = static_cast<pinocchio::ReferenceFrame>(refFrame);
    
    pinocchio::Motion a = pinocchio::getFrameAcceleration(model, data, frameId, rf);
    for(int i = 0; i < 3; ++i) {
        outArray.set(i, val(a.linear()[i]));
        outArray.set(i+3, val(a.angular()[i]));
    }
}

void updateFramePlacements_js(Model& model, Data& data) {
    pinocchio::updateFramePlacements(model, data);
}

val getJointPlacement_js(const Data& data, JointIndex jointId) {
    const pinocchio::SE3& placement = data.oMi[jointId];
    val result = val::object();
    result.set("translation", vector3dToJs(placement.translation()));
    result.set("rotation", matrixXdToJs(placement.rotation()));
    return result;
}

val centerOfMass_js(Model& model, Data& data, const val& q_js) {
    VectorXd q = jsToVectorXd(q_js);
    pinocchio::centerOfMass(model, data, q);
    return vector3dToJs(data.com[0]);
}

double computeTotalMass_js(const Model& model) {
    return pinocchio::computeTotalMass(model);
}

val randomConfiguration_js(const Model& model) {
    VectorXd q = pinocchio::randomConfiguration(model);
    return vectorXdToJs(q);
}

val neutralConfiguration_js(const Model& model) {
    VectorXd q = pinocchio::neutral(model);
    return vectorXdToJs(q);
}

// ─── Data accessors ─────────────────────────────────────────────

val dataTau(const Data& data) { return vectorXdToJs(data.tau); }
val dataNle(const Data& data) { return vectorXdToJs(data.nle); }

val dataComAt(const Data& data, unsigned idx) {
    return vector3dToJs(data.com[idx]);
}

val dataGetVelocity(const Data& data, JointIndex jointId) {
    if (jointId >= data.v.size()) {
        std::string msg = "Joint index out of bounds.";
        EM_ASM({ throw new RangeError(UTF8ToString($0)); }, msg.c_str());
        throw std::out_of_range(msg);
    }
    VectorXd v = data.v[jointId].toVector();
    return vectorXdToJs(v);
}

val dataGetAcceleration(const Data& data, JointIndex jointId) {
    if (jointId >= data.a.size()) {
        std::string msg = "Joint index out of bounds.";
        EM_ASM({ throw new RangeError(UTF8ToString($0)); }, msg.c_str());
        throw std::out_of_range(msg);
    }
    VectorXd a = data.a[jointId].toVector();
    return vectorXdToJs(a);
}

// ─── Embind Module ──────────────────────────────────────────────

EMSCRIPTEN_BINDINGS(pinocchio_wasm) {

    // ── Reference Frame enum ──
    enum_<pinocchio::ReferenceFrame>("ReferenceFrame")
        .value("WORLD", pinocchio::WORLD)
        .value("LOCAL", pinocchio::LOCAL)
        .value("LOCAL_WORLD_ALIGNED", pinocchio::LOCAL_WORLD_ALIGNED)
        ;

    // ── SE3 ──
    class_<SE3>("SE3")
        .class_function("identity", &se3Identity)
        .class_function("fromRotationTranslation", &se3FromRotationTranslation)
        .class_function("fromXyzRpy", &se3FromXyzRpy)
        ;

    // ── Inertia ──
    class_<Inertia>("Inertia")
        .class_function("fromMassComInertia", &inertiaFromMassComInertia)
        ;

    // ── JointModelWrapper ──
    class_<JointModelWrapper>("JointModel");

    // Joint factories
    function("JointModelRX", &makeJointModelRX);
    function("JointModelRY", &makeJointModelRY);
    function("JointModelRZ", &makeJointModelRZ);
    function("JointModelPX", &makeJointModelPX);
    function("JointModelPY", &makeJointModelPY);
    function("JointModelPZ", &makeJointModelPZ);
    function("JointModelRevoluteUnaligned", &makeJointModelRevoluteUnaligned);
    function("JointModelPrismaticUnaligned", &makeJointModelPrismaticUnaligned);
    function("JointModelFreeFlyer", &makeJointModelFreeFlyer);
    function("JointModelFixed", &makeJointModelFixed);

    // ── Model ──
    class_<Model>("Model")
        .constructor<>()
        .property("nq", &Model::nq)
        .property("nv", &Model::nv)
        .property("njoints", &Model::njoints)
        .property("nframes", &Model::nframes)
        .property("name", &Model::name)
        .function("existFrame", optional_override([](const Model& m, const std::string& name) -> bool {
            return m.existFrame(name);
        }))
        .function("getFrameId", optional_override([](const Model& m, const std::string& name) -> FrameIndex {
            return m.getFrameId(name);
        }))
        ;

    function("addJoint", &modelAddJoint);
    function("addJointWithLimits", &modelAddJointWithLimits);
    function("appendBodyToJoint", &modelAppendBodyToJoint);

    // ── Data ──
    class_<Data>("Data")
        .constructor<const Model&>()
        .function("getVelocity", &dataGetVelocity)
        .function("getAcceleration", &dataGetAcceleration)
        .function("oMf", optional_override([](const Data& d, size_t frame_id) -> pinocchio::SE3 {
            if (frame_id >= d.oMf.size()) {
                std::string msg = "Frame index out of bounds.";
                EM_ASM({ throw new RangeError(UTF8ToString($0)); }, msg.c_str());
                throw std::out_of_range(msg);
            }
            return d.oMf[frame_id];
        }))
        ;

    function("getTau", &dataTau);
    function("getNle", &dataNle);
    function("getComAt", &dataComAt);

    // ── Algorithms ──
    function("rnea", &rnea_js);
    function("aba", &aba_js);
    function("abaWithForces", select_overload<val(Model&, Data&, const val&, const val&, const val&, const val&)>(&aba_fext_js));
    function("abaWithForces", select_overload<val(Model&, Data&, const val&, const val&, const val&)>(&aba_with_forces_default_js));
    function("crba", &crba_js);
    function("computeKineticEnergy", &computeKineticEnergy_js);
    function("computePotentialEnergy", &computePotentialEnergy_js);
    function("computeGeneralizedGravity", &computeGeneralizedGravity_js);
    function("nonLinearEffects", &nonLinearEffects_js);
    function("forwardKinematics", &forwardKinematics_js);
    function("forwardKinematicsQVA", &forwardKinematicsQVA_js);
    function("updateFramePlacements", &updateFramePlacements_js);
    function("getJointPlacement", &getJointPlacement_js);
    function("computeJointJacobians", &computeJointJacobians_js);
    function("getJointJacobian", &getJointJacobian_js);
    function("computeFrameJacobian", &computeFrameJacobian_js);
    function("getFrameJacobian", &getFrameJacobian_js);
    function("getFrameVelocity", &getFrameVelocity_js);
    function("getFrameAcceleration", &getFrameAcceleration_js);
    function("centerOfMass", &centerOfMass_js);
    function("computeTotalMass", &computeTotalMass_js);
    function("randomConfiguration", &randomConfiguration_js);
    function("neutralConfiguration", &neutralConfiguration_js);
}
