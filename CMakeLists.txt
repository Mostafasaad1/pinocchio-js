cmake_minimum_required(VERSION 3.14)
project(pinocchio-wasm CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─────────────────────────────────────────────
# Path to the Pinocchio source tree
# Override with: cmake .. -DPINOCCHIO_SOURCE_DIR=/path/to/pinocchio
# ─────────────────────────────────────────────
if(NOT DEFINED PINOCCHIO_SOURCE_DIR)
  get_filename_component(PINOCCHIO_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
endif()

message(STATUS "Pinocchio source dir: ${PINOCCHIO_SOURCE_DIR}")

# ─────────────────────────────────────────────
# Pinocchio build options — disable everything we don't need
# ─────────────────────────────────────────────
set(BUILD_PYTHON_INTERFACE OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_URDF_SUPPORT OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_SDF_SUPPORT OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_COLLISION_SUPPORT OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_AUTODIFF_SUPPORT OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_CASADI_SUPPORT OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_OPENMP_SUPPORT OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_EXTRA_SUPPORT OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARK OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(ENABLE_TEMPLATE_INSTANTIATION OFF CACHE BOOL "" FORCE)

# ─────────────────────────────────────────────
# Dependencies
# ─────────────────────────────────────────────

# Eigen3 — try config mode first, then fallback to headers
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  find_path(EIGEN3_INCLUDE_DIR Eigen/Core
    HINTS /usr/include/eigen3 /usr/local/include/eigen3 /opt/homebrew/include/eigen3
    NO_CMAKE_FIND_ROOT_PATH
  )
  if(EIGEN3_INCLUDE_DIR)
    message(STATUS "Found Eigen3 headers: ${EIGEN3_INCLUDE_DIR}")
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3::Eigen INTERFACE ${EIGEN3_INCLUDE_DIR})
  else()
    message(STATUS "Eigen3 not found locally. Downloading Eigen 3.4.0 via FetchContent...")
    include(FetchContent)
    FetchContent_Declare(
      Eigen3
      URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(Eigen3)
    
    # Define the alias if not defined by Eigen's CMake
    if(NOT TARGET Eigen3::Eigen)
      add_library(Eigen3::Eigen ALIAS eigen)
    endif()
  endif()
endif()
# Boost — use Emscripten Ports to avoid host /usr/include pollution
if(EMSCRIPTEN)
  message(STATUS "Using Emscripten Ports for Boost headers")
  # Use Ports globally for all targets
  add_compile_options(-s USE_BOOST_HEADERS=1)
else()
  # Non-WASM fallback
  find_package(Boost 1.55 REQUIRED)
  # Boost_INCLUDE_DIRS will be used later
endif()

# ─────────────────────────────────────────────
# Pinocchio include paths (header-only usage)
# ─────────────────────────────────────────────
set(PINOCCHIO_INCLUDE_DIR "${PINOCCHIO_SOURCE_DIR}/include")

# ─────────────────────────────────────────────
# WASM target
# ─────────────────────────────────────────────
add_executable(pinocchio_wasm src/pinocchio_embind.cpp)

target_include_directories(pinocchio_wasm PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${PINOCCHIO_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(pinocchio_wasm PRIVATE Eigen3::Eigen)

# Pinocchio compile definitions
target_compile_definitions(pinocchio_wasm PRIVATE
  PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  # PINOCCHIO_WITH_URDFDOM is now left undefined (via config.hpp) to disable strict checks
)

# ─────────────────────────────────────────────
# Emscripten-specific flags
# ─────────────────────────────────────────────
if(EMSCRIPTEN)
  set_target_properties(pinocchio_wasm PROPERTIES
    OUTPUT_NAME "pinocchio"
    SUFFIX ".js"
  )

  # Emscripten sysroot is usually: emsdk/upstream/emscripten/cache/sysroot
  target_compile_options(pinocchio_wasm PRIVATE
      -msimd128                  # WASM SIMD for Eigen vectorization
    )

  target_link_options(pinocchio_wasm PRIVATE
    --bind # Enable Embind
    -sWASM=1
    -sALLOW_MEMORY_GROWTH=1
    -sENVIRONMENT=web,worker,node
    -sMODULARIZE=1
    -sEXPORT_NAME=PinocchioModule
    -sMALLOC=emmalloc # Smaller allocator
    -sNO_FILESYSTEM=1 # No virtual filesystem needed
  )

  # Size optimization (override for debug builds)
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(pinocchio_wasm PRIVATE -O3)
    target_link_options(pinocchio_wasm PRIVATE -O3 -flto --closure 1)
  elseif(CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
    target_compile_options(pinocchio_wasm PRIVATE -Oz)
    target_link_options(pinocchio_wasm PRIVATE -Oz -flto --closure 1)
  endif()

else()
  message(WARNING
    "This project is intended for Emscripten. "
    "Use: emcmake cmake .. -DCMAKE_BUILD_TYPE=Release"
  )
endif()
