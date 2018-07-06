std_msgs/Header header

# end_effector = true -> use 
bool[4] end_effector
uint32 EE_LEFT_FOOT = 0
uint32 EE_RIGHT_FOOT = 1
uint32 EE_LEFT_HAND = 2
uint32 EE_RIGHT_HAND = 3

uint32[4] mode
uint32 RELATIVE=0
uint32 ABSOLUTE=1

geometry_msgs/Pose[4] pose

float64[4] duration
