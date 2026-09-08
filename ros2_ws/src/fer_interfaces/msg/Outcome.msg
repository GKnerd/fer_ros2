# Outcome.msg

uint8 OK=0
uint8 INVALID_GOAL=1      # malformed request (e.g. two targets set, speed out of range)
uint8 UNSUPPORTED=2       # valid, but this backend can't do it (e.g. MPC without straight lines)
uint8 NOT_FOUND=3         # unknown object id, unknown named target, object not seen
uint8 INVALID_STATE=4     # object exists but is in the wrong status (e.g. Grasp on a GRASPED object)
uint8 UNREACHABLE=5       # target itself impossible: no joint solution, or target in collision
uint8 NO_PATH=6           # target fine, no collision-free path found
uint8 EXECUTION_FAILED=7  # started moving, controller or robot reported an error
uint8 CANCELLED=8
uint8 TIMEOUT=9
uint8 GRASP_FAILED=10     # gripper closed but no object held
uint8 RELEASE_FAILED=11   # gripper did not open
uint8 GRIPPER_FAILED=12   # gripper error unrelated to the object
uint8 NO_DATA=99          # grasp node has no recent depth image
uint8 code
string message            # for humans and logs only, trees never parse it
