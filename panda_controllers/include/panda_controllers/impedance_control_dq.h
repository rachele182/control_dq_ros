#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include "dqrobotics/DQ.h"
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>


#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>

#include <franka_msgs/SetFullCollisionBehavior.h>
#include <franka_msgs/SetJointImpedance.h>

#include <panda_controllers/DesiredProjectTrajectory.h>
#include <panda_controllers/DesiredImpedance.h>
#include <panda_controllers/EEpose.h>
#include <panda_controllers/InfoDebug.h>
#include <panda_controllers/RobotState.h>

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 4, 1> Vector4d;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 8, 1> Vector8d;

using namespace DQ_robotics;
using DQ_robotics::DQ;
using DQ_robotics::DQ_Kinematics;
using DQ_robotics::DQ_SerialManipulator;

namespace panda_controllers {

class ImpedanceControlDq : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 	public:
		bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
		void starting(const ros::Time&) override;
		void update(const ros::Time&, const ros::Duration& period) override;
		franka_msgs::SetFullCollisionBehavior collBehaviourSrvMsg;
		ros::ServiceClient collBehaviourClient;
		franka_msgs::SetJointImpedance jointImpedanceSrvMsg;
		ros::ServiceClient jointImpedanceClient;		
 	private:

		//----------FRANKA ---------// 
		Matrix<double, 7, 1> saturateTorqueRate(
		const Matrix<double, 7, 1>& tau_d_calculated,
		const Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)
		std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
		std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
		std::vector<hardware_interface::JointHandle> joint_handles_;
        DQ_SerialManipulator init_dq_robot();
        bool var_damp; 
		std::string name_space; 

        // ---------FUNCTIONS-----------//        
        MatrixXd getQ4(DQ_robotics::DQ &rot);
        MatrixXd getQ8(DQ_robotics::DQ &rot);
        MatrixXd getQ4_dot(DQ_robotics::DQ &rot,DQ_robotics::DQ &drot);
        MatrixXd getQ8_dot(DQ_robotics::DQ &x,DQ_robotics::DQ &dx);
		Vector6d wrench_mapping(Vector6d wrench_ext,DQ_robotics::DQ x_hat);
		Vector6d dead_zone(Vector6d wrench_ext, double dz_value);
		void admittance_eq(Vector6d flog,Vector6d y_hat,Vector6d dy_hat,
        MatrixXd Kd,MatrixXd Bd,MatrixXd Md) ; 
        
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
        
		//----------VARIABLES----------//
		Matrix<double, 7, 1> tau_limit;                    // joint torque limits vector [Nm], from datasheet https://frankaemika.github.io/docs/control_parameters.html
		const double delta_tau_max_{1.0};                  // torque rate limit [Nm/ms], from datasheet https://frankaemika.github.io/docs/control_parameters.html
		Vector6d wrench_ext;                               // external wrench w.r.t. world frame
		Vector6d wrench_n;                                 // external wrench after dead zone 
		double t; 
        double time_prec;
		Matrix<double, 7, 1> q;                            // joint positions
		Matrix<double, 7, 1> dq;                           // joint velocities
		Matrix<double, 3, 1> position_d_;                  // initial position
		Quaterniond orientation_d_;                        // initial orientation; 
		Vector4d or_d_; 
        Matrix<double, 6, 6> K;                            // virtual stiffness 
        Matrix<double, 6, 6> D;                            // virtual damping
        Matrix<double, 6, 6> M;                            // virtual mass
		Matrix<double, 8, 1> pose_d_;                      // desired pose  
		Matrix<double, 8, 1> dpose_d_;                     // desired velocity 
		Matrix<double, 8, 1> ddpose_d_;                    // desired acceleration      
		Matrix<double, 8, 1> pose_;                        // current pose  
		Matrix<double, 8, 1> dpose_;                       // current velocity         
		Matrix<double, 7, 1> q_d_nullspace_;               // qdes null-space controller                         
		Matrix<double, 7, 7> I7;
        Matrix<double, 6, 6> I6;
		Matrix<double, 8, 8> I8;
		Matrix <double,8,6> Q8;           // partial derivative log mapping dx = Q8*dy
		Matrix <double,8,6> Q8_dot;   
		
	    
		//----------SUBSCRIBERS----------//
		ros::Subscriber sub_des_traj_proj_;
		void desiredProjectTrajectoryCallback(const panda_controllers::DesiredProjectTrajectoryConstPtr& msg);

		ros::Subscriber sub_des_imp_proj_;
		void desiredImpedanceProjectCallback(const panda_controllers::DesiredImpedanceConstPtr& msg);

		//----------PUBLISHERS----------//
		ros::Publisher pub_pos_error;
		ros::Publisher pub_endeffector_pose_;
		ros::Publisher pub_ee_pose_;
		ros::Publisher pub_robot_state_;
		ros::Publisher pub_info_debug;
		
        //----------MESSAGES----------/7
		panda_controllers::EEpose ee_pos_msg;
		panda_controllers::InfoDebug info_debug_msg;	
		panda_controllers::RobotState robot_state_msg;
};

}  // namespace franka_softbots
