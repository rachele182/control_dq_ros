#pragma once

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <Eigen/Dense>
// ROS
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
// DQ TOOLBOX
#include "dqrobotics/DQ.h"
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h>
// FRANKA model
#include <panda_controllers/Dynamics.h>
// FRANKA ROS
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
// FRANKA MSGS
#include <eigen_conversions/eigen_msg.h>
#include <franka_msgs/SetFullCollisionBehavior.h>
#include <franka_msgs/SetJointImpedance.h>
// ROS MSGS
#include <panda_controllers/DesiredProjectTrajectory.h>
#include <panda_controllers/CompliantTraj.h>
#include <panda_controllers/EEpose.h>
#include <panda_controllers/InfoDebug.h>
#include <panda_controllers/RobotState.h>
#include <panda_controllers/InfoMsg.h>

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 4, 1> Vector4d;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 8, 1> Vector8d;
typedef Matrix<double, 16, 1> Vector16d;
typedef Matrix<double, 14, 1> Vector14d;

using namespace DQ_robotics;
using namespace std; 
using DQ_robotics::DQ; using DQ_robotics::DQ_Kinematics; using DQ_robotics::DQ_SerialManipulator; using DQ_robotics::DQ_CooperativeDualTaskSpace;

namespace panda_controllers {

struct FrankaDataContainer {
	std::unique_ptr<franka_hw::FrankaStateHandle>
      	state_handle_;  ///< To read to complete robot state.
  	std::unique_ptr<franka_hw::FrankaModelHandle>
        model_handle_;  ///< To have access to e.g. jacobians.
  	std::vector<hardware_interface::JointHandle> 
		joint_handles_;  ///< To command joint torques.
	Vector3d position_d_;
	Quaterniond orientation_d_; 
	const double delta_tau_max_{1.0}; 
};

class DualArmControl : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 	public:
	    /**
		* Initializes the controller class to be ready to run.
   		*
   		* @param[in] robot_hw Pointer to a RobotHW class to get interfaces and resource handles.
   		* @param[in] node_handle Nodehandle that allows getting parameterizations from the server and
   		* starting subscribers.
   		* @return True if the controller was initialized successfully, false otherwise.
   		*/
		bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
		void starting(const ros::Time&) override;
			/**
   		* Computes the control-law and commands the resulting joint torques to the robot.
   		*
   		* @param[in] period The control period (here 0.001s).
   		*/
		void update(const ros::Time&, const ros::Duration& period) override;
		franka_msgs::SetFullCollisionBehavior collBehaviourSrvMsg;
		ros::ServiceClient collBehaviourClient;
		franka_msgs::SetJointImpedance jointImpedanceSrvMsg;
		ros::ServiceClient jointImpedanceClient;		

		static Matrix<double, 59, 1> Xb_l;
		static Matrix<double, 59, 1> Xb_r;

 	private:
		std::map<std::string, FrankaDataContainer>
     		 arms_data_;             ///< Holds all relevant data for both arms.
  		std::string left_arm_id_;   ///< Name of the left arm, retrieved from the parameter server.
  		std::string right_arm_id_;  ///< Name of the right arm, retrieved from the parameter server.
		
		// -----------------//
  		///< Publisher for the centering tracking frame of the coordinated motion.
  		realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> center_frame_pub_;
  		///< Rate to trigger publishing the current pose of the centering frame.
  		franka_hw::TriggerRate publish_rate_;

		//----------FRANKA ---------// 
		 /**
   		* Saturates torque commands to ensure feasibility.
   		*
   		* @param[in] arm_data The data container of the arm.
   		* @param[in] tau_d_calculated The raw command according to the control law.
   		* @param[in] tau_J_d The current desired torque, read from the robot state.
   		* @return The saturated torque command for the 7 joints of one arm.
   		*/
  		Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      			const FrankaDataContainer& arm_data,
      			const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      			const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

		// GET DQ Robots
        DQ_SerialManipulator init_dq_robot(Vector3d r_B_O,Vector4d B_Q_O,double EE_offset); // DQ panda representation
		// ------DQ dual Panda representation---//
  		DQ_CooperativeDualTaskSpace init_dual_panda(DQ_Kinematics* robot1, DQ_Kinematics* robot2);

        //Utils
		bool var_damp; 
		std::string name_space;   
		double scale(double tf);  
		/**
   		* Initializes a single Panda robot arm.
   		* @param[in] robot_hw A pointer the RobotHW class for getting interfaces and resource handles.
   		* @param[in] arm_id The name of the panda arm.
   		* @param[in] joint_names The names of all joints of the panda.
   		* @return True if successful, false otherwise.
   		*/
  		bool initArm(hardware_interface::RobotHW* robot_hw,
   		            const std::string& arm_id,
   		            const std::vector<std::string>& joint_names);
	
   		/**
   		* Computes the decoupled controller update for a single arm.
   		*
   		* @param[in] arm_data The data container of the arm to control.
   		*/                                            
		//----------VARIABLES----------//
		Matrix<double, 7, 1> tau_limit;                    // joint torque limits vector [Nm], from datasheet https://frankaemika.github.io/docs/control_parameters.html
		const double delta_tau_max_{1.0};                  // torque rate limit [Nm/ms], from datasheet https://frankaemika.github.io/docs/control_parameters.html
		Vector6d wrench_ext_l;                             // external wrench left arm
		Vector6d wrench_ext_r;  						   // external wrench right arm
		Vector7d initial_tau_ext_l;  
		Vector7d initial_tau_ext_r;  
		DQ p_curr_dq; 								       // current position
		DQ or_curr_dq;                                     // current orientation     
		Vector14d q_in;                                    //Initial augmented joint configuration      
		Matrix<double, 7, 1> ql;                           // joint positions left arm
		Matrix<double, 7, 1> dql;                          // joint velocities left arm
		Matrix<double, 7, 1> qr;                           // joint positions right arm
		Matrix<double, 7, 1> dqr;                          // joint velocities right arm
		Matrix<double, 14, 1> q;                           // augmented joint angless vector
		Matrix<double, 14, 1> dq;                          // augmented joint velocities vector
		Matrix<double, 3, 1> pos_l_;                       // position left EE
		Vector4d rot_l_;                                   // orientation left EE 
		Matrix<double, 3, 1> pos_r_;                       // position right EE
		Vector4d rot_r_;                                   // orientation right EE 
		DQ x1_dq;                                          // left arm pose DQ
		DQ x2_dq;										   // right arm pose DQ
		Vector4d or_d_l;  
		Vector4d or_d_r;
		Vector8d rel_pose_in_;                             // initial relative pose
		Vector8d abs_pose_in_;                             // initial absolute pose
		Matrix<double, 8, 1> pose_r_d_;                    // desired relative pose
		Matrix<double, 8, 1> dpose_r_d_;                   // desired relative velocity
		Matrix<double, 8, 1> ddpose_r_d_;                  // desired relative acceleration 
		Matrix<double, 8, 1> pose_a_d_;                    // desired absolute pose
		Matrix<double, 8, 1> dpose_a_d_;                   // desired absolute velocity
		Matrix<double, 8, 1> ddpose_a_d_;                  // desired absolute acceleration 
		Matrix<double, 7, 1> q_nullspace_;                 // qdes null-space 
        Matrix<double, 8, 8> I8;                           
		Matrix<double, 7, 7> I7;
		Matrix<double, 16, 16> I16;
	    //Momentum observer Variables
		int l_counter; 
		int r_counter; 
		int reset_l; 
		int reset_r; 
		int st; 
		Vector7d rl_tmp;  
		Vector7d rr_tmp;  
		Vector7d r_l;  //observer output left arm
		Vector7d pl_dot_hat;
		Vector7d pl_int_hat;  
		Vector7d p0_l;  
		Vector7d p_l;  
		Vector7d r_r;  //observer output right EM
		Vector7d p_r;  
		Vector7d pr_dot_hat;
		Vector7d pr_int_hat;  
		Vector7d p0_r;  
		int count; 
		ros::Time t_init; 
		double t; 

		//----------SUBSCRIBERS----------//
		ros::Subscriber sub_compl_traj_proj_;
		void CompliantTrajCallback(const panda_controllers::CompliantTraj::ConstPtr& msg);

		//----------SUBSCRIBERS----------//
		ros::Subscriber sub_nom_traj_proj_;
		void desiredProjectTrajectoryCallback(const panda_controllers::DesiredProjectTrajectoryConstPtr& msg);

		// //----------PUBLISHERS----------//
		ros::Publisher pub_robot_state_;
		ros::Publisher pub_info_debug;
		ros::Publisher pub_info_msg;

        // //----------MESSAGES----------//
		std_msgs::Float64 info_msg; 
		panda_controllers::InfoDebug info_debug_msg;	
		panda_controllers::RobotState robot_state_msg;
};

}  // namespace franka_softbots
