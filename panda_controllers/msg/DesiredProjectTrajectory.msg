std_msgs/Header header
# geometry_msgs/Pose pose
# geometry_msgs/Pose velocity
# geometry_msgs/Pose acceleration
int32 phase
float64[3] phase_array
float64[8] pose_d
float64[8] dpose_d
float64[8] ddpose_d
float64[3] position_d #also for absolute position
#For dual arm
float64[8] pose_r
float64[8] dpose_r
float64[8] ddpose_r
float64[3] position_r
float64[3] position_1
float64[3] position_2
float64[4] or_r
float64[3] vel_r
float64[3] acc_r
float64[3] vel_a
float64[3] acc_a
float64[4] rot_r
float64[4] rot_a
