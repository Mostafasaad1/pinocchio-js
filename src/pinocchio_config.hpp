// Pinocchio WASM Configuration Header
// This header defines missing macros required by the Pinocchio library

#ifndef __pinocchio_wasm_config_hpp__
#define __pinocchio_wasm_config_hpp__

// Define PINOCCHIO_DEPRECATED_MESSAGE as a C++17 deprecated attribute
// This is required because the pinocchio headers use this macro but don't define it
#define PINOCCHIO_DEPRECATED_MESSAGE(msg) [[deprecated(msg)]]

#endif // __pinocchio_wasm_config_hpp__
