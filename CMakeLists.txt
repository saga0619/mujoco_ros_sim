# Copyright 2021 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)

# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# Default to GLVND if available.
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

# This line has to appear before 'PROJECT' in order to be able to disable incremental linking

set(MUJOCO_DEP_VERSION_lodepng
    b4ed2cd7ecf61d29076169b49199371456d4f90b
    CACHE STRING "Version of `lodepng` to be fetched."
)

project(
  mujoco_ros_sim
  VERSION 3.2.7
  DESCRIPTION "MuJoCo simulate binaries"
  HOMEPAGE_URL "https://mujoco.org"
)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)

# Check if we are building as standalone project.
set(SIMULATE_STANDALONE ON)

add_compile_options(-O)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

include(FetchContent)
include(FindOrFetch)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)
find_package (Eigen3 3.3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Fetch lodepng dependency.
if(NOT TARGET lodepng)
  FetchContent_Declare(
    lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG ${MUJOCO_DEP_VERSION_lodepng}
  )

  FetchContent_GetProperties(lodepng)
  if(NOT lodepng_POPULATED)
    FetchContent_Populate(lodepng)
    # This is not a CMake project.
    set(LODEPNG_SRCS ${lodepng_SOURCE_DIR}/lodepng.cpp)
    set(LODEPNG_HEADERS ${lodepng_SOURCE_DIR}/lodepng.h)
    add_library(lodepng STATIC ${LODEPNG_HEADERS} ${LODEPNG_SRCS})
    target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
  endif()
endif()

include_directories(
    include
    src/simulate
    src/ros)

if( ${ARCHITECTURE} STREQUAL "aarch64" )
  file(GLOB LIB_MUJOCO "lib/aarch64/libmujoco.so" "lib/aarch64/libmujoco.so.3.2.7")
else()
  file(GLOB LIB_MUJOCO "lib/x86_64/libmujoco.so" "lib/x86_64/libmujoco.so.3.2.7")
endif()


add_library(mujoco_data_header INTERFACE)
target_include_directories(mujoco_data_header 
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

# Simulate library
add_library(platform_ui_adapter OBJECT)
target_sources(
  platform_ui_adapter
  PUBLIC  src/simulate/glfw_adapter.h src/simulate/glfw_dispatch.h src/simulate/platform_ui_adapter.h
  PRIVATE src/simulate/glfw_adapter.cc src/simulate/glfw_dispatch.cc src/simulate/platform_ui_adapter.cc
)
target_compile_options(platform_ui_adapter PRIVATE ${MUJOCO_SIMULATE_COMPILE_OPTIONS})

target_include_directories(
  platform_ui_adapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
                             $<TARGET_PROPERTY:glfw,INTERFACE_INCLUDE_DIRECTORIES>
)
target_link_libraries(platform_ui_adapter PUBLIC ${LIB_MUJOCO})

add_library(mujoco::platform_ui_adapter ALIAS platform_ui_adapter)

add_library(libsimulate STATIC $<TARGET_OBJECTS:platform_ui_adapter>)
set_target_properties(libsimulate PROPERTIES OUTPUT_NAME simulate)
add_library(mujoco::libsimulate ALIAS libsimulate)

target_sources(
  libsimulate
  PUBLIC src/simulate/simulate.h
  PRIVATE src/simulate/simulate.cc src/simulate/array_safety.h
)
target_include_directories(libsimulate PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(libsimulate PRIVATE ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
target_link_libraries(libsimulate PUBLIC lodepng mujoco::platform_ui_adapter ${LIB_MUJOCO})
target_link_options(libsimulate PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})

add_library(ros_controller src/ros/ros_controller.cpp)
target_link_libraries(ros_controller
  ${LIB_MUJOCO}
  Eigen3::Eigen
)
ament_target_dependencies(ros_controller rclcpp std_msgs sensor_msgs)


add_executable(mujoco_ros_sim src/simulate/main.cpp src/simulate/array_safety.h ${SIMULATE_RESOURCE_FILES})
target_compile_options(mujoco_ros_sim PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})

target_link_libraries(
  mujoco_ros_sim
  libsimulate
  ros_controller
  ${LIB_MUJOCO}
  glfw
  Threads::Threads
  lodepng
)

target_link_options(mujoco_ros_sim PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})
set_target_properties(mujoco_ros_sim PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN" 
)

install(TARGETS mujoco_ros_sim
  DESTINATION lib/${PROJECT_NAME})

install(FILES ${LIB_MUJOCO} DESTINATION lib/${PROJECT_NAME})
install(FILES ${PLUGIN_MUJOCO} DESTINATION lib/${PROJECT_NAME})

ament_package()

