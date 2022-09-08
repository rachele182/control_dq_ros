#ifndef DUAL_IMPEDANCE_H
#define DUAL_IMPEDANCE_H

#include <memory>
#include <string>
#include <vector>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <ros/node_handle.h>

//Franka
#include <franka/robot_state.h>
#include <franka_msgs/FrankaState.h>

//Messages
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <panda_controllers/DesiredImpedance.h>
#include <panda_controllers/EEpose.h>
#include <panda_controllers/DesiredProjectTrajectory.h>
#include <panda_controllers/CompliantTraj.h>
#include <panda_controllers/InfoDebug.h>

// DQ toolbox
#include "dqrobotics/DQ.h"
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>

typedef Matrix<double, 8, 1> Vector8d;
typedef Matrix<double, 6, 1> Vector6d;
using namespace std; 

//namespace panda_controllers{

class dual_impedance_loop{
    public:
        bool init(ros::NodeHandle& node_handle);
        void update();
        std::string name_space;
    private: 
        //Functions
        MatrixXd getQ4 (DQ_robotics::DQ &rot);
        MatrixXd getQ8 (DQ_robotics::DQ &rot);
        MatrixXd getQ4_dot(DQ_robotics::DQ &rot,DQ_robotics::DQ &drot);
        MatrixXd getQ8_dot(DQ_robotics::DQ &x,DQ_robotics::DQ &dx);
        Vector6d wrench_mapping(Vector6d wrench_ext,DQ_robotics::DQ x_hat);
        Vector6d dead_zone(Vector6d wrench_ext, double dz_value,double dz_value_torques);
        void admittance_eq(Vector6d flog_r,Vector6d flog_abs,Vector6d yr_hat,Vector6d dyr_hat,Vector6d ya_hat,Vector6d dya_hat,
        MatrixXd Kd_r,MatrixXd Bd_r,MatrixXd Md_r, MatrixXd Kd_a,MatrixXd Bd_a,MatrixXd Md_a,double time_prec, const double t) ; 
        void compute_pose_disp(Vector6d yr_hat,Vector6d dyr_hat,Vector6d ddyr_hat,Vector6d ya_hat,Vector6d dya_hat,Vector6d ddya_hat); 
        void compute_traj(Vector8d xr_d,Vector8d dxr_d,Vector8d ddxr_d,Vector8d xa_d,Vector8d dxa_d,Vector8d ddxa_d,
                            Vector8d xr_hat,Vector8d dxr_hat,Vector8d ddxr_hat,Vector8d xa_hat,Vector8d dxa_hat,Vector8d ddxa_hat);
        void wrench_adaptor(Vector6d wrench_1,Vector6d wrench_2,DQ_robotics::DQ x1, DQ_robotics::DQ x2,DQ_robotics::DQ xa); 
        double Filter(double y, double y_last); //  exponential moving average filter

        //TRAJ STRUCTURES 
        struct adm_struct{ //log of relative/absolute pose displacements
            Vector6d yr_hat;
            Vector6d dyr_hat;
            Vector6d ddyr_hat;
            Vector6d ya_hat;
            Vector6d dya_hat;
            Vector6d ddya_hat;
        } adm_eq;
        struct disp_struct{
            Vector8d xr_hat;
            Vector8d dxr_hat;
            Vector8d ddxr_hat;
            Vector8d xa_hat;
            Vector8d dxa_hat;
            Vector8d ddxa_hat;
        } disp;
        struct comp_struct{
            Vector8d xr_c;
            Vector8d dxr_c;
            Vector8d ddxr_c;
            Vector8d xa_c;
            Vector8d dxa_c;
            Vector8d ddxa_c;
        } comp;
        struct coop_wrenches{
            Vector6d wr;
            Vector6d wa;
        } coop_wrench; 
        //VARIABLES
        ros::Time t_init; 
        int count;
        double t; 
        double time_prec;
        Matrix<double, 8, 6> Q8;  
        Matrix<double, 8, 8> I8;                           
		Matrix<double, 6, 6> I6;
        Matrix<double, 6, 6> KD_r;            // virtual stiffness relative
        Matrix<double, 6, 6> BD_r;            // virtual damping relative
        Matrix<double, 6, 6> MD;              // virtual mass 
        Matrix<double, 6, 6> KD_a;            // virtual stiffness absolute
        Matrix<double, 6, 6> BD_a;            // virtual damping absolute
        Vector6d wrench_ext_l_;               // External Wrench left arm 6x1
        Vector6d wrench_ext_r_;               // External Wrench right arm 6x1 
        Vector6d wrench_ext_n_l_;             // Filtered External Wrench left arm 6x1
        Vector6d wrench_ext_n_r_;             // Filtered External Wrench right arm 6x1 
        Vector8d pose_d_;                     // desired nominal absolute pose 8x1
        Vector8d dpose_d_;                    // desired nominal absolute dpose 8x1
        Vector8d ddpose_d_;                   // desired nominal absolute ddpose 8x1
        Vector8d pose_r_d_;                   // desired nominal relative pose 8x1
        Vector8d dpose_r_d_;                  // desired nominal relative dpose 8x1
        Vector8d ddpose_r_d_;                 // desired nominal relative ddpose 8x1
        Vector8d xr_;                         // current relative pose
        Vector8d xa_;                         // current absolute pose
        Vector8d x1_;                         // current left EE pose
        Vector8d x2_;                         // current right EE pose
        //utils for filter
        double fx_r;
        double fy_r;
        double fz_r;
        double fx_l;
        double fy_l;
        double fz_l;
        double fx_r_prec;
        double fy_r_prec;
        double fz_r_prec;
        double fx_l_prec;
        double fy_l_prec;
        double fz_l_prec;
    
        
    //------------SUBSCRIBERS-----------//
        
        //nominal desired trajectories
        ros::Subscriber sub_des_traj_proj_;
        void desiredProjectTrajectoryCallback(const panda_controllers::DesiredProjectTrajectoryConstPtr& msg);

        // external forces
        ros::Subscriber sub_cdts_var;
        void cdts_var_Callback(const panda_controllers::InfoDebugConstPtr& msg); 



       // desired impedance
        ros::Subscriber sub_des_imp_proj_;
		void desiredImpedanceProjectCallback(const panda_controllers::DesiredImpedanceConstPtr& msg);

    //------------PUBLISHERS-----//
        ros::Publisher pub_compliant_traj;

    //------------MESSAGES-------//  
        panda_controllers::CompliantTraj compliant_traj_msg;
}; 

//} //end namespace


#endif 