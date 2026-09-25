# WorldObject.msg

string id                           # world-model-owned, e.g. "cylinder_3"
string class_id
float32 score                       # detector confidence; 1.0 for fixtures and manual entries
bool fixed                          # table, walls: never picked, never updated by detection
uint8 FREE=0
uint8 GRASPED=1
uint8 LOST=2                        # was dropped or failed to place; location unknown until re-detected
uint8 status
string held_by                      # link name when GRASPED, else empty
geometry_msgs/PoseStamped pose      # center of shape; GRASPED: frame_id = held_by
shape_msgs/SolidPrimitive shape     # centered on pose
string source                       # "mock_perception", "d405", "release_estimate", "manual"
builtin_interfaces/Time last_observed   # last time a detector actually saw it
