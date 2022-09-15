std_msgs/Header header
float64[8] pose_c
float64[8] dpose_c
float64[8] ddpose_c
float64[6] flog
float64[6] wrench_n
float64[3] position_c
float64[3] vel_c
float64[3] acc_c
float64[3] f_filtered
#dual-arm sytem
float64[8] pose_rel_c
float64[8] dpose_rel_c
float64[8] ddpose_rel_c
##################
float64[8] pose_abs_c
float64[8] dpose_abs_c
float64[8] ddpose_abs_c
###########
float64[3] pos_rel_c #computed position for relative
float64[3] vel_rel_c #computed linear velocity for relative
float64[3] acc_rel_c #computed linear acceleration for relative 
float64[3] pos_abs_c #computed position for absolute
float64[3] vel_abs_c #computed linear velocity for absolute
float64[3] acc_abs_c #computed linear acceleration for absolute
float64[3] fa_f #ext force abs filtered
float64[3] fr_f #ext force rel filtered
float64[3] fa #absolute force
float64[3] fr #relative force
float64[3] fr_rel_frame #ext force rel (relative frame)
float64[3] stiff #translational stiffness
float64[3] n_r
float64[3] n_a
float64[4] rot_r
float64[4] rot_a
float64 phi_r
float64 phi_a