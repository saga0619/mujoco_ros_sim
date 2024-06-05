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
* /mujoco_ros_interface/applied_ext_force <mujoco_ros_msgs::applyforce> : simulator <-> controller connector.
* mujoco_ros_msgs::applyforce : link index, force(3), torque(3) command from controller.
Modify the controller to recognize applyforce.msg, and configure the publisher to publish as shown below.
```
#define DEG2RAD (0.01745329251994329576923690768489)

ros::Publisher mujoco_ext_force_apply_pub;
mujoco_ext_force_apply_pub = nh_.advertise<mujoco_ros_msgs::applyforce>("/mujoco_ros_interface/applied_ext_force", 10);
mujoco_ros_msgs::applyforce mujoco_applied_ext_force_;

double force_temp_ = 100, theta_temp_ = 0; //100 N, 0 degree

if (tick >= 0 && tick < 2000) // (Example) in 2000hz loop, apply force for 1 second.
{    
  mujoco_applied_ext_force_.wrench.force.x = force_temp_*sin(theta_temp_*DEG2RAD); //x-axis linear force
  mujoco_applied_ext_force_.wrench.force.y = -force_temp_*cos(theta_temp_*DEG2RAD); //y-axis linear force  
  mujoco_applied_ext_force_.wrench.force.z = 0.0; //z-axis linear force
  mujoco_applied_ext_force_.wrench.torque.x = 0.0; //x-axis angular moment
  mujoco_applied_ext_force_.wrench.torque.y = 0.0; //y-axis angular moment
  mujoco_applied_ext_force_.wrench.torque.z = 0.0; //z-axis angular moment
  
  mujoco_applied_ext_force_.link_idx = 1; //link idx; 1:pelvis
  
  mujoco_ext_force_apply_pub.publish(mujoco_applied_ext_force_);                    
}
  ```
  
