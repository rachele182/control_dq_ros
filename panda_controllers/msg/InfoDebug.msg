std_msgs/Header header
float64[3] pos_error
float64[7] tau_task
float64[7] tau_d
float64[7] tau_null
float64[3] pos_curr
float64[3] position_c
float64[6] wrench_ext
# Info for dual-arm control
float64[7] tau_left #computed torque for left arm
float64[7] tau_right #computed torque for right arm
float64[6] wrench_ext_1 #ext wrench on left EE (WF)
float64[6] wrench_ext_2 #ext wrench on left EE (WF)
float64[3] rel_pos #relative position
float64[3] abs_pos #absolute position
float64[8] rel_pose #relative pose
float64[8] abs_pose #absolute pose
float64[8] x1 #arm1 pose
float64[8] x2 #arm2 pose
float64[3] pos_1 #relative position
float64[3] pos_2 #absolute position
float64[3] er #relative position error
float64[3] ea #absolute position error
float64 abs_norm #absolute error norm
float64 rel_norm
float64[3] vel #lin velocity
float64[3] w #ang velocity

