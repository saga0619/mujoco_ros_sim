# mujoco_ros_sim

Simple connect between mujoco with ros. 

Mujoco sends joint and sensor data via rostopic, and receives torque or position command data via rostopic.   

mujoco is based on 2.1 // Mujoco is free now!

position/torque command available

※ubuntu 18.04 is recommended since graphical issue at 16.04

## Installation

clone this git to your_catkin_ws/src, then compile with catkin_make

with tocabi_ecat package, mujoco can send status , receive commands via shared memory

## how to start  

you can test mujoco_ros_sim with dyros_red or dyros_jet

* [DYROS_RED](https://github.com/saga0619/dyros_red)
* [DYROS_JET](https://github.com/psh117/dyros_jet)
* [DYROS_TOCABI](https://github.com/saga0619/dyros_tocabi)
* [DYROS_TOCABI_V2](https://github.com/saga0619/dyros_tocabi_v2)

