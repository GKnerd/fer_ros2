# MotionTarget.msg exactly one of the three target forms is set

geometry_msgs/PoseStamped pose      # TCP (fer_hand_tcp) pose
string[] joint_names                # joint target; names required, order free
float64[] joint_positions
uint8 PATH_FREE=0                   # any collision-free path
uint8 PATH_STRAIGHT=1               # TCP moves in a straight line, orientation interpolated
uint8 path
string[] may_touch                  # world-model object ids the hand may contact on this motion
