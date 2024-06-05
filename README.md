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


## ROS TOPIC LIST
### Publisher 
* /mujoco_ros_interface/sim_command_sim2con <std_msgs::String> : simulator <-> controller connector
* /mujoco_ros_interface/joint_states <sensor_msgs::JointState> : publish joint states(d->qpos, d->qvel, d->qacc)
* /mujoco_ros_interface/sensor_states <mujoco_ros_msgs::SensorState> : publish sensor states
* /mujoco_ros_interface/sim_time <std_msgs::Float32> : publish simulation time

### Subscriber
* /mujoco_ros_interface/sim_command_con2sim <std_msgs::String> : simulator <-> controller connector
* /mujoco_ros_interface/joint_set <mujoco_ros_msgs::JointSet> : position/torque command from controller

### How to Apply External Force

