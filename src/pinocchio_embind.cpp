// ──────────────────────────────────────────────────────────────────
// Pinocchio WASM — Embind Wrapper
// Exposes Pinocchio's core C++ API to JavaScript via Emscripten Embind.
// ──────────────────────────────────────────────────────────────────

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <cstdlib>
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
 * Convert a JS Float64Array (or regular Array) to Eigen::VectorXd using bulk memory operations.
 */
VectorXd jsToVectorXd(const val& arr) {
    if (arr.isUndefined() || arr.isNull()) {
        return VectorXd(0);
    }
    const unsigned len = arr["length"].as<unsigned>();
    VectorXd v(len);
    if (len > 0) {
        val wasmView = val(typed_memory_view(len, v.data()));
        wasmView.call<void>("set", arr);
    }
    return v;
}

/**
 * Convert Eigen::VectorXd to a JS Float64Array (bulk copy).
 * If outArray is provided with matching length, writes directly into outArray.
 */
val vectorXdToJs(const VectorXd& v, const val& outArray = val::undefined()) {
    const size_t len = static_cast<size_t>(v.size());
    if (len == 0) {
        if (!outArray.isUndefined() && !outArray.isNull()) return outArray;
        return val::global("Float64Array").new_(0);
    }
    val view = val(typed_memory_view(len, v.data()));
    if (!outArray.isUndefined() && !outArray.isNull() && outArray["length"].as<size_t>() == len) {
        outArray.call<void>("set", view);
        return outArray;
    }
    val result = val::global("Float64Array").new_(len);
    result.call<void>("set", view);
    return result;
}

/**
 * Convert a 3-element JS array to Eigen::Vector3d using bulk memory copy.
 */
Vector3d jsToVector3d(const val& arr) {
    Vector3d v;
    val wasmView = val(typed_memory_view(3, v.data()));
    wasmView.call<void>("set", arr);
    return v;
}

/**
 * Convert Eigen::Vector3d to a JS Float64Array [x, y, z].
 * If outArray is provided with length 3, writes directly into outArray.
 */
val vector3dToJs(const Vector3d& v, const val& outArray = val::undefined()) {
    val view = val(typed_memory_view(3, v.data()));
    if (!outArray.isUndefined() && !outArray.isNull() && outArray["length"].as<size_t>() == 3) {
        outArray.call<void>("set", view);
        return outArray;
    }
    val result = val::global("Float64Array").new_(3);
    result.call<void>("set", view);
    return result;
}

/**
 * Convert a 6-element JS array to Eigen::Matrix<double,6,1>.
 */
Eigen::Matrix<double,6,1> jsToVector6d(const val& arr) {
    Eigen::Matrix<double,6,1> v;
    val wasmView = val(typed_memory_view(6, v.data()));
    wasmView.call<void>("set", arr);
    return v;
}

/**
 * Convert a flat 9-element JS array to Eigen::Matrix3d (row-major input).
 */
Matrix3d jsToMatrix3d(const val& arr) {
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> m_row;
    val wasmView = val(typed_memory_view(9, m_row.data()));
    wasmView.call<void>("set", arr);
    return m_row;
}

/**
 * Convert a flat MatrixBase (MatrixXd, Matrix3d, etc.) to a JS Float64Array (column-major).
 * If outArray is provided with matching length, writes directly into outArray.
 */
template <typename Derived>
val matrixXdToJs(const Eigen::MatrixBase<Derived>& m, const val& outArray = val::undefined()) {
    const size_t len = static_cast<size_t>(m.rows() * m.cols());
    if (len == 0) {
        if (!outArray.isUndefined() && !outArray.isNull()) return outArray;
        return val::global("Float64Array").new_(0);
    }
    Derived evaluated = m.eval();
    val view = val(typed_memory_view(len, evaluated.data()));
    if (!outArray.isUndefined() && !outArray.isNull() && outArray["length"].as<size_t>() == len) {
        outArray.call<void>("set", view);
        return outArray;
    }
    val result = val::global("Float64Array").new_(len);
    result.call<void>("set", view);
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
    double buf[6];
    val wasmView = val(typed_memory_view(6, buf));

    for (unsigned k = 0; k < n; ++k) {
        val key = keys[k];
        std::string keyStr = key.as<std::string>();
        char* endptr = nullptr;
        long parsed = std::strtol(keyStr.c_str(), &endptr, 10);
        if (*endptr != '\0' || parsed < 0 || parsed >= model.njoints) {
            std::string msg = "Link index " + keyStr + " is out of range (0.." + std::to_string(model.njoints - 1) + ")";
            EM_ASM({ throw new (RangeError)(UTF8ToString($0)); }, msg.c_str());
            return fext;
        }
        int idx = static_cast<int>(parsed);

        val w = fext_js[key];
        if (w.isUndefined() || w.isNull() || w["length"].isUndefined()) {
            std::string msg = "Wrench at link " + std::to_string(idx) + " must have 6 components, got 0";
            EM_ASM({ throw new (RangeError)(UTF8ToString($0)); }, msg.c_str());
            return fext;
        }
        unsigned len = w["length"].as<unsigned>();
        if (len != 6) {
            std::string msg = "Wrench at link " + std::to_string(idx) + " must have 6 components, got " + std::to_string(len);
            EM_ASM({ throw new (RangeError)(UTF8ToString($0)); }, msg.c_str());
            return fext;
        }
        wasmView.call<void>("set", w);
        Vector3d torque(buf[0], buf[1], buf[2]);
        Vector3d linear(buf[3], buf[4], buf[5]);
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
            const val& q_js, const val& v_js, const val& a_js,
            const val& outArray) {
    VectorXd q = jsToVectorXd(q_js);
    VectorXd v = jsToVectorXd(v_js);
    VectorXd a = jsToVectorXd(a_js);
    pinocchio::rnea(model, data, q, v, a);
    return vectorXdToJs(data.tau, outArray);
}

val rnea_default_js(Model& model, Data& data,
                    const val& q_js, const val& v_js, const val& a_js) {
    return rnea_js(model, data, q_js, v_js, a_js, val::undefined());
}

val aba_js(Model& model, Data& data,
           const val& q_js, const val& v_js, const val& tau_js,
           const val& outArray) {
    VectorXd q = jsToVectorXd(q_js);
    VectorXd v = jsToVectorXd(v_js);
    VectorXd tau = jsToVectorXd(tau_js);
    pinocchio::aba(model, data, q, v, tau);
    return vectorXdToJs(data.ddq, outArray);
}

val aba_default_js(Model& model, Data& data,
                   const val& q_js, const val& v_js, const val& tau_js) {
    return aba_js(model, data, q_js, v_js, tau_js, val::undefined());
}

val aba_fext_js(Model& model, Data& data,
                const val& q_js, const val& v_js, const val& tau_js,
                const val& fext_js, const val& outArray) {
    VectorXd q = jsToVectorXd(q_js);
    VectorXd v = jsToVectorXd(v_js);
    VectorXd tau = jsToVectorXd(tau_js);

    if (fext_js.isUndefined() || fext_js.isNull()) {
        pinocchio::aba(model, data, q, v, tau);
        return vectorXdToJs(data.ddq, outArray);
    }

    val keys = val::global("Object").call<val>("keys", fext_js);
    if (keys["length"].as<unsigned>() == 0) {
        pinocchio::aba(model, data, q, v, tau);
        return vectorXdToJs(data.ddq, outArray);
    }

    PINOCCHIO_ALIGNED_STD_VECTOR(pinocchio::Force) fext = jsToForceVector(model, fext_js);
    pinocchio::aba(model, data, q, v, tau, fext);
    return vectorXdToJs(data.ddq, outArray);
}

val aba_fext_default_js(Model& model, Data& data,
                        const val& q_js, const val& v_js, const val& tau_js,
                        const val& fext_js) {
    return aba_fext_js(model, data, q_js, v_js, tau_js, fext_js, val::undefined());
}

val aba_with_forces_default_js(Model& model, Data& data,
                               const val& q_js, const val& v_js, const val& tau_js) {
    return aba_js(model, data, q_js, v_js, tau_js, val::undefined());
}

val crba_js(Model& model, Data& data, const val& q_js, const val& outArray) {
    VectorXd q = jsToVectorXd(q_js);
    pinocchio::crba(model, data, q);
    // Pinocchio's crba only fills the upper triangle, we need to symmetrize it
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    return matrixXdToJs(data.M, outArray);
}

val crba_default_js(Model& model, Data& data, const val& q_js) {
    return crba_js(model, data, q_js, val::undefined());
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

val computeGeneralizedGravity_js(Model& model, Data& data, const val& q_js, const val& outArray) {
    VectorXd q = jsToVectorXd(q_js);
    pinocchio::computeGeneralizedGravity(model, data, q);
    return vectorXdToJs(data.g, outArray);
}

val computeGeneralizedGravity_default_js(Model& model, Data& data, const val& q_js) {
    return computeGeneralizedGravity_js(model, data, q_js, val::undefined());
}

val nonLinearEffects_js(Model& model, Data& data, const val& q_js, const val& v_js, const val& outArray) {
    VectorXd q = jsToVectorXd(q_js);
    VectorXd v = jsToVectorXd(v_js);
    pinocchio::nonLinearEffects(model, data, q, v);
    return vectorXdToJs(data.nle, outArray);
}

val nonLinearEffects_default_js(Model& model, Data& data, const val& q_js, const val& v_js) {
    return nonLinearEffects_js(model, data, q_js, v_js, val::undefined());
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
                        JointIndex jointId, int refFrame, const val& outArray) {
    pinocchio::ReferenceFrame rf = static_cast<pinocchio::ReferenceFrame>(refFrame);
    MatrixXd J = MatrixXd::Zero(6, model.nv);
    pinocchio::getJointJacobian(model, data, jointId, rf, J);
    return matrixXdToJs(J, outArray);
}

val getJointJacobian_default_js(const Model& model, Data& data,
                                JointIndex jointId, int refFrame) {
    return getJointJacobian_js(model, data, jointId, refFrame, val::undefined());
}

val computeFrameJacobian_js(Model& model, Data& data, const val& q_js,
                            FrameIndex frameId, int refFrame, const val& outArray) {
    if (frameId >= model.frames.size()) {
        std::string msg = "Invalid frame ID";
        EM_ASM({ throw new (RangeError)(UTF8ToString($0)); }, msg.c_str());
        return val::null();
    }
    VectorXd q = jsToVectorXd(q_js);
    pinocchio::ReferenceFrame rf = static_cast<pinocchio::ReferenceFrame>(refFrame);
    MatrixXd J = MatrixXd::Zero(6, model.nv);
    pinocchio::computeFrameJacobian(model, data, q, frameId, rf, J);
    return matrixXdToJs(J, outArray);
}

val computeFrameJacobian_default_js(Model& model, Data& data, const val& q_js,
                                    FrameIndex frameId, int refFrame) {
    return computeFrameJacobian_js(model, data, q_js, frameId, refFrame, val::undefined());
}

val getFrameJacobian_js(const Model& model, Data& data,
                        FrameIndex frameId, int refFrame, const val& outArray) {
    if (frameId >= model.frames.size()) {
        std::string msg = "Invalid frame ID";
        EM_ASM({ throw new (RangeError)(UTF8ToString($0)); }, msg.c_str());
        return val::null();
    }
    pinocchio::ReferenceFrame rf = static_cast<pinocchio::ReferenceFrame>(refFrame);
    MatrixXd J = MatrixXd::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, frameId, rf, J);
    return matrixXdToJs(J, outArray);
}

val getFrameJacobian_default_js(const Model& model, Data& data,
                                FrameIndex frameId, int refFrame) {
    return getFrameJacobian_js(model, data, frameId, refFrame, val::undefined());
}

val getFrameVelocity_js(const Model& model, const Data& data,
                        FrameIndex frameId, int refFrame, const val& outArray) {
    if (frameId >= model.frames.size()) {
        std::string msg = "Invalid frame ID";
        EM_ASM({ throw new (RangeError)(UTF8ToString($0)); }, msg.c_str());
        return val::null();
    }
    pinocchio::ReferenceFrame rf = static_cast<pinocchio::ReferenceFrame>(refFrame);
    VectorXd v = pinocchio::getFrameVelocity(model, data, frameId, rf).toVector();
    return vectorXdToJs(v, outArray);
}

val getFrameVelocity_default_js(const Model& model, const Data& data,
                                FrameIndex frameId, int refFrame) {
    return getFrameVelocity_js(model, data, frameId, refFrame, val::undefined());
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
        EM_ASM({ throw new (Error)(UTF8ToString($0)); }, msg.c_str());
        return;
    }

    // Heuristic to check if forwardKinematicsQVA was called:
    bool is_initialized = false;
    for (size_t i = 1; i < data.v.size(); ++i) {
        if (!data.v[i].toVector().isZero() || !data.a[i].toVector().isZero()) {
            is_initialized = true;
            break;
        }
    }
    
    if (model.njoints > 1 && !is_initialized) {
        std::string msg = "forwardKinematicsQVA must be called before getFrameAcceleration";
        EM_ASM({ throw new (Error)(UTF8ToString($0)); }, msg.c_str());
        return;
    }

    FrameIndex frameId = model.getFrameId(frameName);
    pinocchio::ReferenceFrame rf = static_cast<pinocchio::ReferenceFrame>(refFrame);
    
    pinocchio::Motion a = pinocchio::getFrameAcceleration(model, data, frameId, rf);
    VectorXd a_vec = a.toVector();
    val view = val(typed_memory_view(6, a_vec.data()));
    outArray.call<void>("set", view);
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

val centerOfMass_js(Model& model, Data& data, const val& q_js, const val& outArray) {
    VectorXd q = jsToVectorXd(q_js);
    pinocchio::centerOfMass(model, data, q);
    return vector3dToJs(data.com[0], outArray);
}

val centerOfMass_default_js(Model& model, Data& data, const val& q_js) {
    return centerOfMass_js(model, data, q_js, val::undefined());
}

val centerOfMassJacobian_js(Model& model, Data& data, const val& q_js, const val& outArray) {
    VectorXd q = jsToVectorXd(q_js);
    const pinocchio::Data::Matrix3x& J = pinocchio::jacobianCenterOfMass(model, data, q);
    return matrixXdToJs(J, outArray);
}

val centerOfMassJacobian_default_js(Model& model, Data& data, const val& q_js) {
    return centerOfMassJacobian_js(model, data, q_js, val::undefined());
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

val integrate_js(const Model& model, const val& q_js, const val& v_js, const val& outArray) {
    if (q_js.isUndefined() || q_js.isNull() || q_js["length"].as<int>() != model.nq) {
        int len = (q_js.isUndefined() || q_js.isNull()) ? 0 : q_js["length"].as<int>();
        std::string msg = "Configuration vector q has invalid dimension: expected " + std::to_string(model.nq) + ", got " + std::to_string(len);
        EM_ASM({ throw new (RangeError)(UTF8ToString($0)); }, msg.c_str());
        return val::undefined();
    }
    if (v_js.isUndefined() || v_js.isNull() || v_js["length"].as<int>() != model.nv) {
        int len = (v_js.isUndefined() || v_js.isNull()) ? 0 : v_js["length"].as<int>();
        std::string msg = "Velocity vector v has invalid dimension: expected " + std::to_string(model.nv) + ", got " + std::to_string(len);
        EM_ASM({ throw new (RangeError)(UTF8ToString($0)); }, msg.c_str());
        return val::undefined();
    }
    if (!outArray.isUndefined() && !outArray.isNull() && outArray["length"].as<int>() != model.nq) {
        int len = outArray["length"].as<int>();
        std::string msg = "Output vector outArray has invalid dimension: expected " + std::to_string(model.nq) + ", got " + std::to_string(len);
        EM_ASM({ throw new (RangeError)(UTF8ToString($0)); }, msg.c_str());
        return val::undefined();
    }

    VectorXd q = jsToVectorXd(q_js);
    VectorXd v = jsToVectorXd(v_js);
    VectorXd q_next = pinocchio::integrate(model, q, v);
    return vectorXdToJs(q_next, outArray);
}

val integrate_default_js(const Model& model, const val& q_js, const val& v_js) {
    return integrate_js(model, q_js, v_js, val::undefined());
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
        EM_ASM({ throw new (RangeError)(UTF8ToString($0)); }, msg.c_str());
        return val::null();
    }
    VectorXd v = data.v[jointId].toVector();
    return vectorXdToJs(v);
}

val dataGetAcceleration(const Data& data, JointIndex jointId) {
    if (jointId >= data.a.size()) {
        std::string msg = "Joint index out of bounds.";
        EM_ASM({ throw new (RangeError)(UTF8ToString($0)); }, msg.c_str());
        return val::null();
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
                EM_ASM({ throw new (RangeError)(UTF8ToString($0)); }, msg.c_str());
                return pinocchio::SE3::Identity();
            }
            return d.oMf[frame_id];
        }))
        ;

    function("getTau", &dataTau);
    function("getNle", &dataNle);
    function("getComAt", &dataComAt);

    // ── Algorithms ──
    function("rnea", select_overload<val(Model&, Data&, const val&, const val&, const val&)>(&rnea_default_js));
    function("rnea", select_overload<val(Model&, Data&, const val&, const val&, const val&, const val&)>(&rnea_js));

    function("aba", select_overload<val(Model&, Data&, const val&, const val&, const val&)>(&aba_default_js));
    function("aba", select_overload<val(Model&, Data&, const val&, const val&, const val&, const val&)>(&aba_js));

    function("abaWithForces", select_overload<val(Model&, Data&, const val&, const val&, const val&)>(&aba_with_forces_default_js));
    function("abaWithForces", select_overload<val(Model&, Data&, const val&, const val&, const val&, const val&)>(&aba_fext_default_js));
    function("abaWithForces", select_overload<val(Model&, Data&, const val&, const val&, const val&, const val&, const val&)>(&aba_fext_js));

    function("crba", select_overload<val(Model&, Data&, const val&)>(&crba_default_js));
    function("crba", select_overload<val(Model&, Data&, const val&, const val&)>(&crba_js));

    function("computeKineticEnergy", &computeKineticEnergy_js);
    function("computePotentialEnergy", &computePotentialEnergy_js);

    function("computeGeneralizedGravity", select_overload<val(Model&, Data&, const val&)>(&computeGeneralizedGravity_default_js));
    function("computeGeneralizedGravity", select_overload<val(Model&, Data&, const val&, const val&)>(&computeGeneralizedGravity_js));

    function("nonLinearEffects", select_overload<val(Model&, Data&, const val&, const val&)>(&nonLinearEffects_default_js));
    function("nonLinearEffects", select_overload<val(Model&, Data&, const val&, const val&, const val&)>(&nonLinearEffects_js));

    function("forwardKinematics", &forwardKinematics_js);
    function("forwardKinematicsQVA", &forwardKinematicsQVA_js);
    function("updateFramePlacements", &updateFramePlacements_js);
    function("getJointPlacement", &getJointPlacement_js);
    function("computeJointJacobians", &computeJointJacobians_js);

    function("getJointJacobian", select_overload<val(const Model&, Data&, JointIndex, int)>(&getJointJacobian_default_js));
    function("getJointJacobian", select_overload<val(const Model&, Data&, JointIndex, int, const val&)>(&getJointJacobian_js));

    function("computeFrameJacobian", select_overload<val(Model&, Data&, const val&, FrameIndex, int)>(&computeFrameJacobian_default_js));
    function("computeFrameJacobian", select_overload<val(Model&, Data&, const val&, FrameIndex, int, const val&)>(&computeFrameJacobian_js));

    function("getFrameJacobian", select_overload<val(const Model&, Data&, FrameIndex, int)>(&getFrameJacobian_default_js));
    function("getFrameJacobian", select_overload<val(const Model&, Data&, FrameIndex, int, const val&)>(&getFrameJacobian_js));

    function("getFrameVelocity", select_overload<val(const Model&, const Data&, FrameIndex, int)>(&getFrameVelocity_default_js));
    function("getFrameVelocity", select_overload<val(const Model&, const Data&, FrameIndex, int, const val&)>(&getFrameVelocity_js));

    function("getFrameAcceleration", &getFrameAcceleration_js);

    function("centerOfMass", select_overload<val(Model&, Data&, const val&)>(&centerOfMass_default_js));
    function("centerOfMass", select_overload<val(Model&, Data&, const val&, const val&)>(&centerOfMass_js));

    function("centerOfMassJacobian", select_overload<val(Model&, Data&, const val&)>(&centerOfMassJacobian_default_js));
    function("centerOfMassJacobian", select_overload<val(Model&, Data&, const val&, const val&)>(&centerOfMassJacobian_js));

    function("computeTotalMass", &computeTotalMass_js);
    function("randomConfiguration", &randomConfiguration_js);
    function("neutralConfiguration", &neutralConfiguration_js);

    function("integrate", select_overload<val(const Model&, const val&, const val&)>(&integrate_default_js));
    function("integrate", select_overload<val(const Model&, const val&, const val&, const val&)>(&integrate_js));
}
