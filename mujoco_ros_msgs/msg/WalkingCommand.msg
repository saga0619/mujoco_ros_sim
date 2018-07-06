std_msgs/Header header

uint32 walk_mode
uint32 NONE = 0
uint32 STATIC_WALKING = 1

bool[2] compensator_mode
uint32 HIP_COMPENSTOR = 0
uint32 EXTERNAL_ENCODER = 1

uint32 ik_mode
uint32 IK=0
uint32 JACOBIAN = 1

bool heel_toe

bool first_foot_step

float32 x
float32 y
float32 z
float32 height
float32 theta
float32 step_length_x
float32 step_length_y
