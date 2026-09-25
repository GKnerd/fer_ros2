# PoseTarget.msg 

geometry_msgs/PoseStamped pose      # TCP pose (fer_hand tcp)
uint8 PATH_FREE=0                   # any collision-free path
uint8 PATH_STRAIGHT=1               # TCP moves in a straight line, orientation interpolated
uint8 path
string[] may_touch                  # world-model object ids the hand may contact on this motion
