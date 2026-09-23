# GenZ-LIO core: the odometry pipeline with no ROS and no PCL dependency.
# Requires only Eigen, OpenMP and the vendored IKFoM / robin-map headers, so it
# can be built standalone, wrapped for ROS 1 / ROS 2, or bound to Python.
cmake_minimum_required(VERSION 3.16)
project(genz_lio_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# A wheel is built in a container with no system Eigen, so fall back to
# fetching it rather than failing to configure.
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  message(STATUS "Eigen not found; fetching it")
  include(FetchContent)
  FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE)
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(eigen)
endif()

find_package(OpenMP REQUIRED)

add_library(genz_lio_core STATIC
  core/Downsample.cpp
  core/GenZLIO.cpp
  core/HybridMetricUpdate.cpp
  core/ImuProcessor.cpp
  core/Preprocessor.cpp
  core/ScaleAwareVoxelizer.cpp
  core/ScanBuffer.cpp
  core/StateTransforms.cpp
  core/VoxelMap.cpp
  core/VoxelPrunedSearch.cpp)
add_library(genz_lio::core ALIAS genz_lio_core)

target_include_directories(genz_lio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(genz_lio_core PUBLIC Eigen3::Eigen OpenMP::OpenMP_CXX)

# Wrappers may also inherit an older Eigen from PCL. Export the selected
# headers so every translation unit uses the same Eigen version as the core.
get_target_property(_genz_eigen_includes Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
set_property(TARGET genz_lio_core PROPERTY GENZ_LIO_EIGEN_INCLUDE_DIRS "${_genz_eigen_includes}")

# IKFoM includes <boost/bind.hpp>, which otherwise emits a deprecation pragma on
# every translation unit that reaches it.
target_compile_definitions(genz_lio_core PUBLIC BOOST_BIND_GLOBAL_PLACEHOLDERS)
set_target_properties(genz_lio_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Optional YAML configuration loading, shared by the ROS wrappers. Kept separate
# so the core itself stays free of the dependency.
option(GENZ_LIO_WITH_YAML "Build the YAML configuration loader" ON)
if(GENZ_LIO_WITH_YAML)
  find_package(yaml-cpp REQUIRED)
  add_library(genz_lio_config STATIC config/ConfigIo.cpp)
  add_library(genz_lio::config ALIAS genz_lio_config)
  target_include_directories(genz_lio_config PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(genz_lio_config PUBLIC genz_lio_core PRIVATE yaml-cpp)
  set_target_properties(genz_lio_config PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

option(GENZ_LIO_BUILD_TESTS "Build the core test executables" ON)
if(GENZ_LIO_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
