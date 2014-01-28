# which arm to control
int32 arm

# constants for arm
int32 LEFT=0
int32 RIGHT=1

# control poses, i.e start pose and end pose
geometry_msgs/Pose start_pose
geometry_msgs/Pose end_pose

# action to perform
int32 action

# action constants
int32 TUCK=0
int32 STRETCH=1
# need to specify place location for GRASP action
int32 GRASP=2
int32 PUSH=3

# Push direction
int32 direction

# Push direction constants
int32 FRONT=0
int32 FROM_BELOW=1
int32 FROM_ABOVE=2
int32 FROM_RIGHT_SIDEWAYS=3
int32 FROM_RIGHT_UPRIGHT=4
int32 FROM_LEFT_UPRIGHT=5
int32 FROM_LEFT_SIDEWAYS=6

# FRAME ID
string frame_id
