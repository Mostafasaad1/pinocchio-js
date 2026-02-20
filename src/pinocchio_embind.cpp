// ──────────────────────────────────────────────────────────────────
// Pinocchio WASM — Embind Wrapper
// Exposes Pinocchio's core C++ API to JavaScript via Emscripten Embind.
// ──────────────────────────────────────────────────────────────────

#include <emscripten/bind.h>
#include <emscripten/val.h>

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
    switch (joint.type) {
        case JointModelWrapper::RX:
            return model.addJoint(parentId, pinocchio::JointModelRX(), placement, name);
        case JointModelWrapper::RY:
            return model.addJoint(parentId, pinocchio::JointModelRY(), placement, name);
        case JointModelWrapper::RZ:
            return model.addJoint(parentId, pinocchio::JointModelRZ(), placement, name);
        case JointModelWrapper::PX:
            return model.addJoint(parentId, pinocchio::JointModelPX(), placement, name);
        case JointModelWrapper::PY:
            return model.addJoint(parentId, pinocchio::JointModelPY(), placement, name);
        case JointModelWrapper::PZ:
            return model.addJoint(parentId, pinocchio::JointModelPZ(), placement, name);
        case JointModelWrapper::REVOLUTE_UNALIGNED:
            return model.addJoint(parentId,
                pinocchio::JointModelRevoluteUnaligned(joint.axis),
                placement, name);
        case JointModelWrapper::PRISMATIC_UNALIGNED:
            return model.addJoint(parentId,
                pinocchio::JointModelPrismaticUnaligned(joint.axis),
                placement, name);
        case JointModelWrapper::FREE_FLYER:
            return model.addJoint(parentId, pinocchio::JointModelFreeFlyer(), placement, name);
        case JointModelWrapper::FIXED:
            // Use an empty Composite joint (0 DOFs) to represent a fixed joint in the kinematic tree
            return model.addJoint(parentId,
                pinocchio::JointModelComposite(0),
                placement, name);
        default:
            return 0;
    }
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

    switch (joint.type) {
        case JointModelWrapper::RX:
            return model.addJoint(parentId, pinocchio::JointModelRX(), placement, name,
                                  maxEffort, maxVelocity, minConfig, maxConfig);
        case JointModelWrapper::RY:
            return model.addJoint(parentId, pinocchio::JointModelRY(), placement, name,
                                  maxEffort, maxVelocity, minConfig, maxConfig);
        case JointModelWrapper::RZ:
            return model.addJoint(parentId, pinocchio::JointModelRZ(), placement, name,
                                  maxEffort, maxVelocity, minConfig, maxConfig);
        case JointModelWrapper::PX:
            return model.addJoint(parentId, pinocchio::JointModelPX(), placement, name,
                                  maxEffort, maxVelocity, minConfig, maxConfig);
        case JointModelWrapper::PY:
            return model.addJoint(parentId, pinocchio::JointModelPY(), placement, name,
                                  maxEffort, maxVelocity, minConfig, maxConfig);
        case JointModelWrapper::PZ:
            return model.addJoint(parentId, pinocchio::JointModelPZ(), placement, name,
                                  maxEffort, maxVelocity, minConfig, maxConfig);
        case JointModelWrapper::REVOLUTE_UNALIGNED:
            return model.addJoint(parentId,
                pinocchio::JointModelRevoluteUnaligned(joint.axis),
                placement, name, maxEffort, maxVelocity, minConfig, maxConfig);
        case JointModelWrapper::PRISMATIC_UNALIGNED:
            return model.addJoint(parentId,
                pinocchio::JointModelPrismaticUnaligned(joint.axis),
                placement, name, maxEffort, maxVelocity, minConfig, maxConfig);
        case JointModelWrapper::FREE_FLYER:
            return model.addJoint(parentId, pinocchio::JointModelFreeFlyer(), placement, name,
                                  maxEffort, maxVelocity, minConfig, maxConfig);
        default:
            return 0;
    }
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

void forwardKinematics_js(Model& model, Data& data, const val& q_js) {
    VectorXd q = jsToVectorXd(q_js);
    pinocchio::forwardKinematics(model, data, q);
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
        .property("name", &Model::name)
        ;

    function("addJoint", &modelAddJoint);
    function("addJointWithLimits", &modelAddJointWithLimits);
    function("appendBodyToJoint", &modelAppendBodyToJoint);

    // ── Data ──
    class_<Data>("Data")
        .constructor<const Model&>()
        ;

    function("getTau", &dataTau);
    function("getNle", &dataNle);
    function("getComAt", &dataComAt);

    // ── Algorithms ──
    function("rnea", &rnea_js);
    function("forwardKinematics", &forwardKinematics_js);
    function("computeJointJacobians", &computeJointJacobians_js);
    function("getJointJacobian", &getJointJacobian_js);
    function("centerOfMass", &centerOfMass_js);
    function("computeTotalMass", &computeTotalMass_js);
    function("randomConfiguration", &randomConfiguration_js);
    function("neutralConfiguration", &neutralConfiguration_js);
}
