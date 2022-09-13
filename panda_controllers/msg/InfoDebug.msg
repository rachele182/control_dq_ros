std_msgs/Header header
float64 norm
float64[7] tau_task
float64[7] tau_measured
float64[7] tau_null
float64[7] tau_model
float64[7] r #moment observer output
float64[3] pos_curr
float64[3] position_c
float64[6] wrench_ext
float64[6] f_ext_hat
float64[7] q
float64[7] dq
float64 t
float64[6] f_veg
# Info for dual-arm control
float64[7] tau_left #computed torque for left arm
float64[7] tau_right #computed torque for right arm
float64[7] tau_l_mes #measured torque for left arm
float64[7] tau_r_mes #measured torque for right arm
float64[7] r_l #measured torque for left arm
float64[7] r_r #measured torque for right arm
float64[6] wrench_ext_1 #ext wrench on left EE (WF)
float64[6] wrench_ext_2 #ext wrench on left EE (WF)
float64[6] f_ext_hat_l #estimated ext wrench left arm 
float64[6] f_ext_hat_r #estimated ext wrench right arm 
float64[3] rel_pos #relative position
float64[3] abs_pos #absolute position
float64[8] rel_pose #relative pose
float64[8] abs_pose #absolute pose
float64[8] x1 #arm1 pose
float64[8] x2 #arm2 pose
float64[3] pos_1 # left arm position
float64[3] pos_2 # right arm position
float64[3] er #relative position error
float64[3] ea #absolute position error
float64 abs_norm #absolute error norm
float64 rel_norm #relative position error norm
float64[4] rot_r # rotation_axis_relative
float64[4] rot_a # rotation_axis_absolute
