
#ifndef IMPEDANCE_H
#define IMPEDANCE_H

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
// #include "utils/lowpass_filter.h"

// DQ toolbox
#include "dqrobotics/DQ.h"
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>


typedef Matrix<double, 8, 1> Vector8d;
typedef Matrix<double, 6, 1> Vector6d;

// namespace panda_controllers{

class impedance_loop{
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
        void admittance_eq(Vector6d flog,Vector6d y_hat,Vector6d dy_hat,
        MatrixXd Kd,MatrixXd Bd,MatrixXd Md,double time_prec, const double t) ; 
        void compute_pose_disp(Vector6d y_hat,Vector6d dy_hat,Vector6d ddy_hat); 
        void compute_traj(Vector8d x_d,Vector8d dx_d,Vector8d ddx_d,
        Vector8d x_hat,Vector8d dx_hat,Vector8d ddx_hat);
        double Filter(double y, double y_last); //  exponential moving average filter
        //TRAJ STRUCT
        struct adm_struct{
            Vector6d y_hat;
            Vector6d dy_hat;
            Vector6d ddy_hat;
        } adm_eq;
        struct disp_struct{
            Vector8d x_hat;
            Vector8d dx_hat;
            Vector8d ddx_hat;
        } disp;
        struct comp_struct{
            Vector8d x_c;
            Vector8d dx_c;
            Vector8d ddx_c;
        }comp;
        //VARIABLES
        int count;
        Vector3d pos_in_;
        Vector4d or_in_; 
        double t; 
        double time_prec;
        Matrix<double, 8, 6> Q8;  
        Matrix<double, 8, 8> I8;                           
		Matrix<double, 6, 6> I6;
        Matrix<double, 6, 6> KD;              // virtual stiffness 
        Matrix<double, 6, 6> BD;              // virtual damping
        Matrix<double, 6, 6> MD;              // virtual mass
        Vector6d wrench_ext;                 // Estimated External Wrench 6x1
		Vector6d wrench_n;                   // new external wrench
        Vector6d wrench_f;                   // external wrench after EMA filter
        double fx;
        double fy;
        double fz; 
        double fx_prec;
        double fy_prec;
        double fz_prec;
        Vector8d pose_d_;                    // desired nominal pose 8x1
        Vector8d dpose_d_;                   // desired nominal dpose 8x1
        Vector8d ddpose_d_;                  // desired nominal ddpose 8x1
        Vector8d x_hat;                      // pose displacement 8x1
        Vector8d dx_hat;
        Vector8d ddx_hat;
        Vector6d y_hat;                     // log mapping of pose displacement 6x1
        Vector6d dy_hat;
        Vector6d ddy_hat;
        Vector8d x_c;                       // compliant traj 8x1
        Vector8d dx_c;
        Vector8d ddx_c;

    //------------SUBSCRIBERS-----------//

        ros::Subscriber sub_ee_pose;
        void ee_pose_Callback(const geometry_msgs::PoseStampedConstPtr& msg);
        
        ros::Subscriber sub_des_traj_proj_;
        void desiredProjectTrajectoryCallback(const panda_controllers::DesiredProjectTrajectoryConstPtr& msg);

        // external forces
        ros::Subscriber sub_ext_forces;
        void f_ext_Callback(const panda_controllers::InfoDebugConstPtr& msg); 

        // desired impedance
        ros::Subscriber sub_des_imp_proj_;
		void desiredImpedanceProjectCallback(const panda_controllers::DesiredImpedanceConstPtr& msg);

    //------------PUBLISHERS-----//
        ros::Publisher pub_compliant_traj;

    //------------MESSAGES-------//  
        panda_controllers::CompliantTraj compliant_traj_msg;
}; 

// } //end namespace


#endif 