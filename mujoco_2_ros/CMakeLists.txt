cmake_minimum_required(VERSION 2.8.3)
project(mujoco_2_ros)

add_compile_options(-std=c++11 -ofast)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mujoco_ros_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
DEPENDS   mujoco_ros_msgs
)

set(MUJOCO_PATH ${CMAKE_SOURCE_DIR}/mujoco_ros_sim/mjpro200)

include_directories(
  include

  ${MUJOCO_PATH}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${MUJOCO_PATH}/bin
)

file(GLOB LIB_MUJOCO ${MUJOCO_PATH}/bin/libmujoco200.so)

file(GLOB UI_SRC "${MUJOCO_PATH}/include/uitools.c")

add_library(mujoco_ros_lib
    ${UI_SRC}
    src/mjros.cpp
    )

add_executable(mujoco_ros src/main.cpp)

target_link_libraries(mujoco_ros
   mujoco_ros_lib

   ${catkin_LIBRARIES}
   ${LIB_MUJOCO}
   GL
   #GLU 
   #glut
   
   ${MUJOCO_PATH}/bin/libglew.so
   ${MUJOCO_PATH}/bin/libglfw.so.3
 )


message(STATUS "MuJoCo path: " ${MUJOCO_PATH})
