# GraspCandidate.msg 

geometry_msgs/PoseStamped pregrasp_pose
geometry_msgs/PoseStamped grasp_pose
geometry_msgs/PoseStamped lift_pose
float64 width                       # goes straight into Grasp.width
float64 force                       # from the object catalog, per class
float32 score
