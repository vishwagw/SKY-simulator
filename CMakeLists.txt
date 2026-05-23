cmake_minimum_required(VERSION 3.22)
project(drone_sim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --------------------------------------------------------------------------
# Build type defaults
# --------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(DRONE_SIM_ASAN       "Enable AddressSanitizer"    OFF)
option(DRONE_SIM_PROFILE    "Enable profiling flags"      OFF)
option(DRONE_SIM_TESTS      "Build unit test suite"       OFF)

# --------------------------------------------------------------------------
# godot-cpp (must be cloned into thirdparty/godot-cpp)
# --------------------------------------------------------------------------
set(GODOT_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/godot-cpp")
if(NOT EXISTS "${GODOT_CPP_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "godot-cpp not found at ${GODOT_CPP_DIR}.\n"
    "Run: git submodule update --init --recursive")
endif()

set(GODOT_CPP_SYSTEM_HEADERS ON CACHE BOOL "" FORCE)
add_subdirectory(${GODOT_CPP_DIR} godot-cpp)

# --------------------------------------------------------------------------
# Sources
# --------------------------------------------------------------------------
file(GLOB_RECURSE DRONE_SIM_SOURCES
  "src/*.cpp"
  "src/aero/*.cpp"
  "src/control/*.cpp"
  "src/sensors/*.cpp"
  "src/core/*.cpp"
)

add_library(drone_sim SHARED ${DRONE_SIM_SOURCES})

target_include_directories(drone_sim PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/aero"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/control"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/sensors"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/core"
)

target_link_libraries(drone_sim PRIVATE godot-cpp)

# --------------------------------------------------------------------------
# Compile flags
# --------------------------------------------------------------------------
target_compile_options(drone_sim PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:
    -Wall -Wextra -Wpedantic
    -ffast-math
    -march=native
    $<$<CONFIG:Release>:-O3 -DNDEBUG>
    $<$<CONFIG:Debug>:-O0 -g3>
  >
  $<$<CXX_COMPILER_ID:MSVC>:
    /W4 /fp:fast /arch:AVX2
    $<$<CONFIG:Release>:/O2 /DNDEBUG>
    $<$<CONFIG:Debug>:/Od /Zi>
  >
)

if(DRONE_SIM_ASAN)
  target_compile_options(drone_sim PRIVATE -fsanitize=address,undefined)
  target_link_options(drone_sim PRIVATE -fsanitize=address,undefined)
endif()

if(DRONE_SIM_PROFILE)
  target_compile_options(drone_sim PRIVATE -pg -fno-omit-frame-pointer)
endif()

# --------------------------------------------------------------------------
# Output naming: drone_sim.windows.release.x86_64.dll  etc.
# --------------------------------------------------------------------------
if(WIN32)
  set(PLATFORM_TAG "windows")
  set_target_properties(drone_sim PROPERTIES SUFFIX ".dll" PREFIX "")
elseif(APPLE)
  set(PLATFORM_TAG "macos")
  set_target_properties(drone_sim PROPERTIES SUFFIX ".dylib" PREFIX "lib")
else()
  set(PLATFORM_TAG "linux")
  set_target_properties(drone_sim PROPERTIES SUFFIX ".so" PREFIX "lib")
endif()

string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TAG)
set_target_properties(drone_sim PROPERTIES
  OUTPUT_NAME "drone_sim.${PLATFORM_TAG}.${BUILD_TAG}.x86_64"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/gdextension/bin"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/gdextension/bin"
)

# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------
if(DRONE_SIM_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
