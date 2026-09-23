# Python bindings for the GenZ-LIO core.
cmake_minimum_required(VERSION 3.16)
project(genz_lio_python LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The core is built from the repository rather than expected to be installed.
add_subdirectory(../cpp/genz_lio ${CMAKE_CURRENT_BINARY_DIR}/genz_lio_core)

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(genz_lio_pybind MODULE genz_lio/pybind/genz_lio_pybind.cpp)
target_link_libraries(genz_lio_pybind PRIVATE genz_lio::core)

# Identify the actual compiled source, including uncommitted core/header fixes.
file(GLOB_RECURSE GENZ_SOURCE_FILES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/../cpp/genz_lio/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../cpp/genz_lio/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../cpp/genz_lio/*.h")
list(APPEND GENZ_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/genz_lio/pybind/genz_lio_pybind.cpp")
list(SORT GENZ_SOURCE_FILES)
set(GENZ_SOURCE_DIGESTS "")
foreach(source IN LISTS GENZ_SOURCE_FILES)
  file(SHA256 "${source}" digest)
  string(APPEND GENZ_SOURCE_DIGESTS "${digest}")
endforeach()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${GENZ_SOURCE_FILES})
string(SHA256 GENZ_SOURCE_HASH "${GENZ_SOURCE_DIGESTS}")
target_compile_definitions(genz_lio_pybind PRIVATE GENZ_LIO_SOURCE_HASH="${GENZ_SOURCE_HASH}")
install(TARGETS genz_lio_pybind DESTINATION genz_lio)

# Ship calibrated benchmark settings and sensor templates without requiring a
# source checkout. Keep the ROS YAMLs as the single source of truth.
install(DIRECTORY ../ros/config/default ../ros/config/experiments
  DESTINATION genz_lio/configs
  FILES_MATCHING PATTERN "*.yaml")
