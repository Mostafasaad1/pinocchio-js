#ifndef PINOCCHIO_CONFIG_HPP
#define PINOCCHIO_CONFIG_HPP

// Version 3.9.0
#define PINOCCHIO_MAJOR_VERSION 3
#define PINOCCHIO_MINOR_VERSION 9
#define PINOCCHIO_PATCH_VERSION 0
#define PINOCCHIO_VERSION "3.9.0"

// Features (UNDEFINE them to disable)
// Note: We comment them out to ensure #if defined(...) returns false
// #define PINOCCHIO_WITH_URDFDOM
// #define PINOCCHIO_WITH_HPP_FCL
// #define PINOCCHIO_WITH_OPENMP

// Deprecation macro (missing in our manual shim)
// Standard deprecation attribute used by GCC/Clang/EMSCRIPTEN
#define PINOCCHIO_DEPRECATED __attribute__((deprecated))

// Export macros
#define PINOCCHIO_EXPORTS
#define PINOCCHIO_DLLIMPORT

#endif // PINOCCHIO_CONFIG_HPP
