#include <cmath>
#include <math.h>
#include <memory>
#include <functional>
#include <panda_controllers/dual_arm_control.h>

//DQ ROBOTICS
#include "dqrobotics/DQ.h"
#include "dqrobotics/robot_modeling/DQ_SerialManipulator.h"
#include "dqrobotics/utils/DQ_LinearAlgebra.h"
#include <dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h>
// FRANKA 
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <franka_hw/trigger_rate.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <ros/transport_hints.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
// Dynamics model
#include <panda_controllers/Dynamics.h>

using DQ_robotics::DQ_Kinematics;
using DQ_robotics::DQ_SerialManipulator;
using DQ_robotics::DQ_CooperativeDualTaskSpace;
using DQ_robotics::E_;
using DQ_robotics::C8;

using namespace DQ_robotics;

#define 	KP			    170.0   // proportional gain motion controller
#define 	KD			    30.0    // derivative gain motion controller
#define 	KP_ABS			120.0    // proportional gain motion controller absolute pose
#define 	KD_ABS			20.0    // derivative gain motion controller absolute pose
#define 	KO			    4.0  	// gain momentum observer
#define     KI              0*30.0    // integrative term 
#define 	D_JOINTS	    2.0    // dissipative term joints
#define 	COLL_LIMIT		50.0   // 
#define 	NULL_STIFF		2.0
#define 	JOINT_STIFF		{3000, 3000, 3000, 3000, 3000, 2000, 100}
#define     RHO             M_PI
#define     PHI             0.0

namespace panda_controllers {

const std::vector<double> xb_l{0.002084,0,0,1.0173,-0.0042701,-0.013951,-0.027933,1.0293,0.044034,-3.1291,-0.01598,-0.0025827,-0.011287,-0.017216,0.10307,0.66468,0.028323,0.56777,0.1371,0.029611,0.015026,0.63635,-0.49639,1.7809,0.018033,0.014545,-0.0050086,0.0018855,0.029855,-0.0086085,0.07923,-0.010402,-0.0017637,0.00010625,-0.0040539,0.021507,0.16397,-0.070696,0.0011735,0.0020802,0.0020867,0.0014081,-0.0051387,0.0042805,-0.001217,-0.048654,-0.090942,0.0044737,-0.075902,0.033227,0.012306,0.043793,0.3814,0.21931,0.25969,0.47304,0.23618,0.26877,0.24234};
Matrix<double, 59, 1> DualArmControl::Xb_l(xb_l.data());
const std::vector<double> xb_r{0.013194,0,0,1.0236,0.016767,-0.019676,-0.033091,1.0461,-0.00095987,-3.1813,-0.027374,0.011822,0.0013866,-0.0088441,0.10316,0.70899,0.016316,0.57733,0.13877,0.018732,0.008907,0.65852,-0.48559,1.7908,0.0082117,0.0085054,-0.0094675,-0.0032702,0.024545,-0.011372,0.074909,0.005767,0.0014424,-0.00010052,-0.00097505,0.026613,0.18937,-0.083343,-0.0056562,0.0039173,0.0023967,0.0012023,-0.0010778,0.0011972,-0.0015276,-0.022549,-0.028092,0.033738,-0.01046,0.018754,-0.0067986,-0.025118,0.27519,0.27734,0.21488,0.21712,0.26261,0.17809,0.33907}; 
Matrix<double, 59, 1> DualArmControl::Xb_r(xb_r.data());

// ---------- DQ_SERIAL_MANIPULATOR PANDA ROBOT ------- //
DQ_SerialManipulator DualArmControl::init_dq_robot(Vector3d r_B_O,Vector4d B_Q_O,double EE_offset) {

	Matrix<double, 4, 7> franka_dh(4, 7);
  	franka_dh << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // theta (q)
    0.333, 0.0, 0.316, 0.0, 0.384, 0.0,
    EE_offset+0.107,  // 
    0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088,             // a
    0.0, -pi / 2, pi / 2, pi / 2, -pi / 2, pi / 2, pi / 2;  // alpha
	DQ_SerialManipulator robot(franka_dh, "modified");
  	DQ p = DQ(0.0, r_B_O(0), r_B_O(1), r_B_O(2)); //position of base frame
  	DQ r = DQ(B_Q_O(0), B_Q_O(1), B_Q_O(2), B_Q_O(3)); //rotation of base frame
  	r = r * r.inv().norm();
  	// std::cout << "Panda Reference Frame: " << r + 0.5 * E_ * p * r << std::endl;
  	robot.set_base_frame(r + 0.5 * E_ * p * r);
  	robot.set_reference_frame(r + 0.5 * E_ * p * r); 
  	return robot;
	}

DQ_CooperativeDualTaskSpace DualArmControl::init_dual_panda(DQ_Kinematics* robot1, DQ_Kinematics* robot2){
	DQ_CooperativeDualTaskSpace dual_arm(robot1,robot2); 
    return dual_arm;
}

//================Init single arms ==============//

bool DualArmControl::initArm(
    hardware_interface::RobotHW* robot_hw,
    const std::string& arm_id,
    const std::vector<std::string>& joint_names) {
 FrankaDataContainer arm_data;
 auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DualArmController: Error getting model interface from hardware");
    return false;
  }
  try {
    arm_data.model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DualArmController: Exception getting model handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DualArmController: Error getting state interface from hardware");
    return false;
  }
  try {
    arm_data.state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DualArmController: Exception getting state handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DualArmController: Error getting effort joint interface from "
        "hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      arm_data.joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "DualArmController: Exception getting joint handles: "
          << ex.what());
      return false;
    }
  }

  arms_data_.emplace(std::make_pair(arm_id, std::move(arm_data)));

  return true;
}
	
// //------------------------------------------------------------------------------//
// //                          		INIT										//
// //------------------------------------------------------------------------------//
bool DualArmControl::init( hardware_interface::RobotHW* robot_hw, 
                                       ros::NodeHandle& node_handle) {
	//Name space extraction to add a prefix to the topic name
	int n = 0;
	name_space = node_handle.getNamespace();
	n = name_space.find("/", 2);
	name_space = name_space.substr(0,n);
										

	//--------------- INITIALIZE SUBSCRIBERS AND PUBLISHERS -----------------//

	sub_compl_traj_proj_ =  node_handle.subscribe(  "/motion_control_dq/compliant_traj", 1, 
													&DualArmControl::CompliantTrajCallback, this,
													ros::TransportHints().reliable().tcpNoDelay());

    sub_nom_traj_proj_ =  node_handle.subscribe(  "/motion_control_dq/dq_trajectory", 1, 
													&DualArmControl::desiredProjectTrajectoryCallback, this,
													ros::TransportHints().reliable().tcpNoDelay());												

	pub_robot_state_ = node_handle.advertise<panda_controllers::RobotState>("/motion_control_dq/robot_state", 1);
	pub_info_debug =  node_handle.advertise<panda_controllers::InfoDebug>("/motion_control_dq/info_debug", 1);


	//---------------- INITIALIZE SERVICE CLIENTS ------------------//

	collBehaviourClient = node_handle.serviceClient<franka_msgs::SetFullCollisionBehavior>( name_space 
																							+ "/franka_control/set_full_collision_behavior");

	jointImpedanceClient = node_handle.serviceClient<franka_msgs::SetJointImpedance>( name_space 
																						+ "/franka_control/set_joint_impedance");
	//----- INITIALIZE NODE, ROBOTs HANDLER AND  INTERFACE -----//

	if (!node_handle.getParam("left/arm_id", left_arm_id_)) {
    	ROS_ERROR_STREAM(
        "DualArmController: Could not read parameter left_arm_id_");
   	    return false;
 		 }

  	std::vector<std::string> left_joint_names;
  	if (!node_handle.getParam("left/joint_names", left_joint_names) || left_joint_names.size() != 7) {
  	  ROS_ERROR(
  	      "DualArmController: Invalid or no left_joint_names parameters "
  	      "provided, "
  	      "aborting controller init!");
  	  return false;
  	}

  	if (!node_handle.getParam("right/arm_id", right_arm_id_)) {
  	  ROS_ERROR_STREAM(
  	      "DualArmController: Could not read parameter right_arm_id_");
  	  return false;
  	}

	std::vector<std::string> right_joint_names;
  	if (!node_handle.getParam("right/joint_names", right_joint_names) ||
  	    right_joint_names.size() != 7) {
  	  ROS_ERROR(
  	      "DualArmController: Invalid or no right_joint_names parameters "
  	      "provided, "
  	      "aborting controller init!");
  	  return false;
  	}

	if (!node_handle.getParam("var_damp", var_damp)) {
			ROS_ERROR_STREAM("DualArmControl: Could not read parameter var_damp");
			return false;
		}

	bool left_success = initArm(robot_hw, left_arm_id_, left_joint_names);
  	bool right_success = initArm(robot_hw, right_arm_id_, right_joint_names);
    
	//---------------- INITIALIZE VARIABLES ------------------//

	pose_r_d_.setZero();                  	      	// desired pose
	dpose_r_d_.setZero();                          	// desired velocity
	ddpose_r_d_.setZero();                        	// desired acceleration
	pose_a_d_.setZero();                  	      	// nominal des pose
	dpose_a_d_.setZero();                          	// nominal des velocity
	ddpose_a_d_.setZero();                        	// nominal des acceleration
	wrench_ext_l.setZero();                         // external wrench applied by left arm
	wrench_ext_r.setZero();                         // external wrench applied by right arm
	initial_tau_ext_l.setZero();
	initial_tau_ext_r.setZero(); 

	// left arm
	tau_limit << 87, 87, 87, 87, 12, 12, 12;  		// joint torques limit vector	
	
	// Collision behaviours limits
	collBehaviourSrvMsg.request.lower_torque_thresholds_acceleration 	= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};
	collBehaviourSrvMsg.request.upper_torque_thresholds_acceleration 	= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};
	collBehaviourSrvMsg.request.lower_torque_thresholds_nominal 		= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};
	collBehaviourSrvMsg.request.upper_torque_thresholds_nominal 		= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};
	collBehaviourSrvMsg.request.lower_force_thresholds_acceleration 	= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};
	collBehaviourSrvMsg.request.upper_force_thresholds_acceleration 	= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};
	collBehaviourSrvMsg.request.lower_force_thresholds_nominal 			= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};
	collBehaviourSrvMsg.request.upper_force_thresholds_nominal 			= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};

	// Joint impedance
	jointImpedanceSrvMsg.request.joint_stiffness = JOINT_STIFF;

	//====== Get the transformation from right_O_frame to left_O_frame ======== ///

	tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
      if (listener.waitForTransform(left_arm_id_ + "_link0", right_arm_id_ + "_link0", ros::Time(0),
                                    ros::Duration(4.0))) {
        listener.lookupTransform(left_arm_id_ + "_link0", right_arm_id_ + "_link0", ros::Time(0),
                                 transform);
      } else {
        ROS_ERROR(
            "DualArmCartesianImpedanceExampleController: Failed to read transform from %s to %s. "
            "Aborting init!",
            (right_arm_id_ + "_link0").c_str(), (left_arm_id_ + "_link0").c_str());
        return false;
      }
    } catch (tf::TransformException& ex) {
      ROS_ERROR("DualArmCartesianImpedanceExampleController: %s", ex.what());
      return false;
    }
   
	return left_success && right_success;

	}

//------------------------------------------------------------------------------//
//                          	  STARTING										//
//------------------------------------------------------------------------------//
 

void DualArmControl::starting(const ros::Time& /*time*/) {

	// Get robots state
	franka::RobotState robot_state_right = arms_data_.at(right_arm_id_).state_handle_->getRobotState();
  	franka::RobotState robot_state_left = arms_data_.at(left_arm_id_).state_handle_->getRobotState();
	Affine3d transform_l(Matrix4d::Map(  // NOLINT (readability-identifier-naming)
      robot_state_left.O_T_EE.data()));           // NOLINT (readability-identifier-naming)
  	Affine3d transform_r(Matrix4d::Map(  // NOLINT (readability-identifier-naming)
      robot_state_right.O_T_EE.data()));          // NOLINT (readability-identifier-naming)
	
	Map<Matrix<double, 7, 1>> initial_tau_measured_l(robot_state_left.tau_J.data());
	Map<Matrix<double, 7, 1>> initial_tau_measured_r(robot_state_right.tau_J.data());
	std::array<double, 7> gravity_array_r = arms_data_.at(right_arm_id_).model_handle_->getGravity();
	Map<Matrix<double, 7, 1>> initial_gravity_r(gravity_array_r.data());
	std::array<double, 7> gravity_array_l =  arms_data_.at(left_arm_id_).model_handle_->getGravity();
	Map<Matrix<double, 7, 1>> initial_gravity_l(gravity_array_l.data());

	//Eigen conversion joint angles and velocities
	Map<Matrix<double, 7, 1> > q_l_in(robot_state_left.q.data());                       // joint positions  [rad]
	Map<Matrix<double, 7, 1> > q_r_in(robot_state_right.q.data()); 
	Map<Matrix<double, 7, 1> > dq_l_in(robot_state_left.dq.data());                       // joint positions  [rad]
	Map<Matrix<double, 7, 1> > dq_r_in(robot_state_right.dq.data());                        // joint positions  [rad]
	//Augment joint vector
	q_in.head(7) << q_l_in; q_in.tail(7) << q_r_in;

	// ==========  BASE FRAMES (set for now base of right arm (ip: 3.100) as world frame) ======= ///
	Vector3d p_b_0_l; //position base frame left arm
	Vector4d r_b_0_l; //rotation base frame left arm (r,i,j,k)
	Vector3d p_b_0_r; //position base frame right arm
	Vector4d r_b_0_r; //rotation base frame right arm (r,i,j,k)
	p_b_0_l << -0.09,1.44,0;  r_b_0_l << 1,0,0,0;  p_b_0_r << 0,0,0; r_b_0_r << 1,0,0,0; 
	//////////////////////////////////////

	//Load Panda robot (left arm)
	DQ_SerialManipulator panda_left = init_dq_robot(p_b_0_l,r_b_0_l,0);

	// Load Panda robot (right arm)
	DQ_SerialManipulator panda_right = init_dq_robot(p_b_0_r,r_b_0_r,0);

	// Cooperative dual arm system
	DQ_CooperativeDualTaskSpace dual_panda = init_dual_panda(&panda_left, &panda_right);

	// Initial position and orientation Left arm
	x1_dq = dual_panda.pose1(q_in);
	pos_l_ = vec3(x1_dq.translation());
	rot_l_ = vec4(x1_dq.rotation()); 

	//  Initial position and orientation right arm
	x2_dq = dual_panda.pose2(q_in);
	pos_r_ = vec3(x2_dq.translation());
	rot_r_ = vec4(x2_dq.rotation()); 

	//Relative pose
	DQ pose_rel_dq;  //relative pose
	pose_rel_dq = dual_panda.relative_pose(q_in); 
	pose_rel_dq = pose_rel_dq.normalize();
	rel_pose_in_ = vec8(pose_rel_dq); 

	//Absolute pose
	DQ pose_abs_dq; //absolute pose
	pose_abs_dq = dual_panda.absolute_pose(q_in); 
	pose_abs_dq = pose_abs_dq.normalize();
	abs_pose_in_ = vec8(pose_abs_dq); 

	// // set equilibrium points to current state
	pose_r_d_ = rel_pose_in_; 
	pose_a_d_ = abs_pose_in_; 
	
	// // set nullspace equilibrium configuration to central angles of joints
	Matrix <double, 7, 1> q_min;
	Matrix <double, 7, 1> q_max;
	q_min << -2.8973, -1.7628, -2.8973, -3.0718+0.1745, -2.8973, -0.0175+0.0873, -2.8973;
	q_max <<  2.8973,  1.7628,  2.8973, -0.0698-0.1745,  2.8973,  3.7525-0.0873,  2.8973;
	q_nullspace_ << (q_max + q_min)/2;

	// set collision behaviour
	collBehaviourClient.call(collBehaviourSrvMsg);
	// set joint impedance
	jointImpedanceClient.call(jointImpedanceSrvMsg);

	I8 = MatrixXd::Identity(8, 8); I7 = MatrixXd::Identity(7, 7); I16 = MatrixXd::Identity(16, 16);

	//initialize momentum observe variables right arm
	MatrixXd mass_in_r,c_in_r, mass_in_l,c_in_l, g_in_l, g_in_r; 
	
	//right arm 
	Dynamics dyn_2(RHO,PHI,Xb_r);
	mass_in_r = dyn_2.get_M(q_r_in); g_in_r = dyn_2.get_tau_G(q_r_in); c_in_r = dyn_2.get_C(q_r_in,dq_r_in); 
	p0_r = (mass_in_r*dq_r_in); 
	p_r = p0_r; //initialize momentum

	//left arm	
	Dynamics dyn(RHO,PHI,Xb_l);
	mass_in_l = dyn.get_M(q_l_in); g_in_l = dyn.get_tau_G(q_l_in); c_in_l = dyn.get_C(q_l_in,dq_l_in); 
	p0_l = (mass_in_l*dq_l_in); 
	p_l = p0_l; //initialize momentum

	//initialize momentum variables
	r_r.setZero();
	// pr_dot_hat = initial_tau_measured_r + (c_in_r.transpose())*dq_r_in - g_in_r;  
	pr_dot_hat.setZero(); 
	pr_int_hat.setZero(); 
		
	r_l.setZero();
	// pl_dot_hat = initial_tau_measured_l + (c_in_l.transpose())*dq_l_in - g_in_l;
	pl_dot_hat.setZero(); 
	pl_int_hat.setZero(); 

	// initial bias torque sensors
	initial_tau_ext_r = initial_tau_measured_r - g_in_r;
	initial_tau_ext_l = initial_tau_measured_l - g_in_l;
	rr_tmp.setZero(); rl_tmp.setZero(); 
	l_counter = 0; r_counter = 0;  
	reset_l = 0;  reset_r = 0; 
	count = 0;
	t_init = ros::Time::now();
	t = (ros::Time::now() - t_init).toSec();
	
 }

// //------------------------------------------------------------------------------//
// //                          	    UPDATE										//
// //------------------------------------------------------------------------------//

void DualArmControl::update(const ros::Time& /*time*/,
                                          const ros::Duration &period /*period*/) {
										
	I8 = MatrixXd::Identity(8, 8); I7 = MatrixXd::Identity(7, 7); I16 = MatrixXd::Identity(16, 16);
//----------- VARIABLES DECLARATIONS and DEFINITIONS -------------//
    //Load Dual-Arm system
	// ==========  BASE FRAMES (set for now base of right arm (ip: 3.100) as world frame) ======= ///
	Vector3d p_b_0_l; //position base frame left arm 
	Vector4d r_b_0_l; //rotation base frame left arm (r,i,j,k)
	Vector3d p_b_0_r; //position base frame right arm
	Vector4d r_b_0_r; //rotation base frame right arm (r,i,j,k)
	p_b_0_l << -0.09,1.44,0;  r_b_0_l << 1,0,0,0;  p_b_0_r << 0,0,0; r_b_0_r << 1,0,0,0; 
	//////////////////////////////////////

//============= LOAD CDTS MODEL ======================= //

	DQ_SerialManipulator panda_left = init_dq_robot(p_b_0_l,r_b_0_l,0);
	DQ_SerialManipulator panda_right = init_dq_robot(p_b_0_r,r_b_0_r,0);
	DQ_CooperativeDualTaskSpace dual_panda = init_dual_panda(&panda_left, &panda_right);

 	// Control Variables
	Vector8d er;  		          // relative pose error  8x1
	Vector8d ea; 		          // absolute pose error  8x1
	Vector8d der;                 // relative vel error   8x1
	Vector8d dea;                 // absolute vel error   8x1
 	Vector16d e_aug;              // agumented error vector 16x1 [er;ea];
 	Vector16d de_aug;             // agumented error vector derivative 16x1 [der;dea];
	Vector16d e_aug_i;            // integrative augmented error vector 16x1 [er;ea];
	Vector16d ddx_des;            // augmented desired acceleration [ddxr_d;ddxa_d]
	Vector8d pose_rel;       	  // current relative pose 8x1
	Vector8d dpose_rel;			  // current relative velocity 8x1
	Vector8d pose_abs;       	  // current absolute pose 8x1
	Vector8d dpose_abs;			  // current absolute velocity 8x1
	Vector3d pos_rel;             // current relative position 3x1
	Vector3d pos_abs;             // current absolute position 3x1
	Vector3d e_pos_rel;           // rel position error (for debug)
	Vector3d e_pos_abs;           // abs position error

//------DQ Variables-----//
	DQ pos_d_dq; 
	                               
	// Jacobians
	Matrix<double, 7, 6> Jg_l_t;           // transpose of jacobian matrix from franka left arm
	Matrix<double, 7, 6> Jg_r_t;           // transpose jacobian matrix from franka left arm
	Matrix<double, 8, 7>   J1;             // pose jacobian left arm
	Matrix<double, 8, 7>   J2;             // pose jacobian right arm
 	Matrix<double, 8, 14>  Jr;             // relative pose jacobian
 	Matrix<double, 8, 14>  Jr_old;         // previous computed relative pose jacobian
	Matrix<double, 8, 14>  Ja;             // absolute pose jacobian 
	Matrix<double, 8, 14>  Ja_old;         // previous compute absolute pose jacobian
	Matrix<double, 8, 14>  Jr_dot;         // relative pose derivative jacobian 
	Matrix<double, 8, 14>  Ja_dot;         // absolute pose derivative jacobian
	Matrix<double, 16, 14> J_aug;          // augmented Jacobian [Jr,Ja]
	J_aug.setZero(); 
	Matrix<double, 16, 14> J_aug_dot;      // derivative augmented Jacobian [Jr_dot,Ja_dot]
	Matrix<double, 14, 16> J_aug_inv;      // augmented jacobian pseudo-inverse 
	//Try dynamically consistent pseudo-inverse (inertia weighted)

// // ============================== GET ARMS STATE ======================== //

	franka::RobotState robot_state_right = arms_data_.at(right_arm_id_).state_handle_->getRobotState();
 	franka::RobotState robot_state_left = arms_data_.at(left_arm_id_).state_handle_->getRobotState();
    //Mesured torques
	Map<Matrix<double, 7, 1> > tau_measured_l(robot_state_left.tau_J.data());
	Map<Matrix<double, 7, 1> > tau_measured_r(robot_state_right.tau_J.data());
	//Eigen conversion joint angles and velocities
	//Left arm
	Map<Matrix<double, 7, 1> > q_l(robot_state_left.q.data());                      // joint positions  [rad]
	Map<Matrix<double, 7, 1> > dq_l(robot_state_left.dq.data());                    // joint velocities
	//Right arm
	Map<Matrix<double, 7, 1> > q_r(robot_state_right.q.data());                      // joint positions  [rad]
	Map<Matrix<double, 7, 1> > dq_r(robot_state_right.dq.data());  					 // joint velocities
	//Augmented joint vectors
	q.head(7) << q_l; q.tail(7) << q_r;
	dq.head(7) << dq_l; dq.tail(7) << dq_r;  

// // -------- get current EEs pose ---------------------- //
	Vector8d x1, x2; 
	x1_dq = dual_panda.pose1(q);
	x1 = vec8(x1_dq); 
	pos_l_ = vec3(x1_dq.translation());
	rot_l_ = vec4(x1_dq.rotation()); 

	//  Initial position and orientation right arm
	x2_dq = dual_panda.pose2(q);
	x2 = vec8(x2_dq);
	pos_r_ = vec3(x2_dq.translation());
	rot_r_ = vec4(x2_dq.rotation()); 

// //================================ GET DYNAMICS MODEL ================  //

	Matrix<double, 14, 14> Ma; //stacked mass matrix
	Matrix<double, 14, 14> Ca; //stacked coriolis matrix
	Matrix<double, 14, 1> ga;      //stacked gravity vector from model
	Matrix<double, 14, 1> fa; 					 //stacked friction vector from model
	Matrix<double, 14, 1> ga_mario; 			 // stacked gravity vector from mario model
	Matrix <double, 6,7> Jr_inv;				 // dynamically consistenst pseudo-inverse of J^T
	Matrix <double, 6,7> Jl_inv;				 // dynamically consistenst pseudo-inverse of J^T
	Ma.setZero(); Ca.setZero(); 

	std::array<double, 7>  gravity_array_right = arms_data_.at(right_arm_id_).model_handle_->getGravity(); 
	std::array<double, 42> jacobian_array_right = arms_data_.at(right_arm_id_).model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
	std::array<double, 7>  coriolis_array_right = arms_data_.at(right_arm_id_).model_handle_->getCoriolis(); 
    std::array<double, 7>  gravity_array_left = arms_data_.at(left_arm_id_).model_handle_->getGravity(); 
	std::array<double, 42> jacobian_array_left = arms_data_.at(left_arm_id_).model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

	// Eigen conversion
    Map<Matrix<double, 7, 1> > g1(gravity_array_left.data());                  // gravity vector from franka 
	Map<Matrix<double, 7, 1> > g2(gravity_array_right.data());
	Map<Matrix<double, 7, 1> > c2(coriolis_array_right.data());
	Map<Matrix<double, 6, 7> > Jg_l(jacobian_array_left.data());
	Map<Matrix<double, 6, 7> > Jg_r(jacobian_array_right.data());

	Jg_l_t = Jg_l.transpose(); Jg_r_t = Jg_r.transpose();


	//----------DYNAMIC MODEL MARIO RIGHT ARM (NO EE)------------//
	Dynamics dyn_2(RHO,PHI,Xb_r);
    MatrixXd m2_mario; MatrixXd c2_mario; VectorXd g2_mario;  
	m2_mario = dyn_2.get_M(q_r); c2_mario = dyn_2.get_C(q_r,dq_r); g2_mario = dyn_2.get_tau_G(q_r); 

	// Left arm
	Dynamics dyn(RHO,PHI,Xb_l);
    MatrixXd m1_mario; MatrixXd c1_mario; VectorXd g1_mario; 
	m1_mario = dyn.get_M(q_l); c1_mario = dyn.get_C(q_l,dq_l); g1_mario = dyn.get_tau_G(q_l); 
	
	// Jr_inv = pinv(Jg_r_t);
	// Jl_inv = pinv(Jg_l_t); 
	
	// Dynamically consistent pseudo-inverse
	Jr_inv = (Jg_r*m2_mario.inverse()*Jg_r_t).inverse()*Jg_r*(m2_mario.inverse()); // right arm
	Jl_inv = (Jg_l*m1_mario.inverse()*Jg_l_t).inverse()*Jg_l*(m1_mario.inverse()); // left arm

	// Mario model
	Ma.block(0,0,7,7) << m1_mario; Ma.block(7,7,7,7) << m2_mario;
    Ca.block(0,0,7,7) << c1_mario; Ca.block(7,7,7,7) << c2_mario;
	ga_mario.head(7) << g1_mario; ga_mario.tail(7) = g2_mario;  
	ga.head(7) << g1; ga.tail(7) = g2;  
	
	//Retrieve measured torques and ext forces from franka
	Map<Matrix<double, 7, 1> > tau_J_d_left(robot_state_left.tau_J_d.data());                // previous cycle commanded torques [Nm]
	Map<Matrix<double, 6, 1> > wrench_ext_l(robot_state_left.O_F_ext_hat_K.data());          // external wrench [N] exerted by arm1 wrt base frame
	Map<Matrix<double, 7, 1> > tau_J_d_right(robot_state_right.tau_J_d.data());              // previous cycle commanded torques [Nm]
	Map<Matrix<double, 6, 1> > wrench_ext_r(robot_state_right.O_F_ext_hat_K.data());         // external wrench [N] exerted by arm2 wrt base frame

// 	Get current CDTS variables
    DQ pose_rel_dq;
    pose_rel_dq = dual_panda.relative_pose(q); 
	pose_rel = vec8(pose_rel_dq);
	
	DQ pose_abs_dq; 
	pose_abs_dq = dual_panda.absolute_pose(q);
	pose_abs = vec8(pose_abs_dq); 

// === Store absolute and relative variables == //
	Vector4d rot_r, rot_a; //rotation 

    pose_rel_dq = pose_rel_dq.normalize(); pose_abs_dq = pose_abs_dq.normalize();
	pos_rel = vec3(pose_rel_dq.translation()); pos_abs = vec3(pose_abs_dq.translation()); 
	rot_r = vec4(pose_rel_dq.rotation()); rot_a = vec4(pose_abs_dq.rotation()); 
	// std::cout << "rot_r" << rot_r.transpose() << std::endl; 
	// std::cout << "rot_a" << rot_a.transpose() << std::endl; 

// // ================= GET JACOBIANS ===================== //

	J1 << dual_panda.pose_jacobian1(q); J2 << dual_panda.pose_jacobian2(q); 
	
	if(count==0){
		Jr_old = dual_panda.relative_pose_jacobian(q_in); 
		Jr_dot.setZero();
		Ja_old = dual_panda.absolute_pose_jacobian(q_in);
		Ja_dot.setZero(); 
	}

	Jr_old = Jr;
	Ja_old = Ja; 

	Jr << dual_panda.relative_pose_jacobian(q); 
	Ja << dual_panda.absolute_pose_jacobian(q); 
	
	Jr_dot = (Jr - Jr_old)/(period.toSec()); 
	Ja_dot = (Ja - Ja_old)/(period.toSec()); 
	
	dpose_rel << Jr*dq;  dpose_abs << Ja*dq; 

	J_aug.topRows(8) << Jr; J_aug.bottomRows(8) << Ja; 
	J_aug_dot.topRows(8) << Jr_dot; J_aug_dot.bottomRows(8) << Ja_dot; 
	
// //  -------------- PUBLISH MATRICES FOR PLANNING -------------//

	robot_state_msg.header.stamp = ros::Time::now();
	pub_robot_state_.publish(robot_state_msg);

// //================= COMPUTE ERRORS ================//

	//Relative pose error
	er << pose_r_d_ - pose_rel;
	der << dpose_r_d_ - dpose_rel;
    
	//Absolute pose error
	ea << pose_a_d_- pose_abs; 
	dea << dpose_a_d_ - dpose_abs; 
	
	//desired accelerations
	ddx_des.head(8) << ddpose_r_d_; ddx_des.tail(8) << ddpose_a_d_; 

	//augmented error defintion
	e_aug.head(8) << er; e_aug.tail(8) << ea; 
	de_aug.head(8) << der; de_aug.tail(8) << dea; 
 
 	// ==== Store variables for analysis === //
	
	DQ des_pos_a_dq; Vector3d des_pos_abs;  DQ des_pos_r_dq; Vector3d des_pos_rel; 

	//des absolute pos 
    des_pos_a_dq = (DQ(pose_a_d_)).normalize(); 
	des_pos_abs = vec3(des_pos_a_dq.translation()); 

	//des relative pos 
	des_pos_r_dq = (DQ(pose_r_d_)).normalize();
	des_pos_rel = vec3(des_pos_r_dq.translation()); 

	//position errors
    e_pos_rel = des_pos_rel - pos_rel; 
    e_pos_abs = des_pos_abs - pos_abs; 

	// std::cout << "pos_rel" << pos_rel.transpose() << std::endl; 
	// std::cout << "pos_abs" << pos_abs.transpose() << std::endl;

	//rotation errors 
	DQ rot_r_d,e_rot_rel_dq; 
	Vector8d e_rot_rel; 
	rot_r_d = des_pos_r_dq.rotation(); 

//  //======================| CONTROL VARIABLES |======================//
    Vector6d wrench_ext_r_hat,wrench_ext_l_hat;  // estimated ext wrench (w. momentum observer)
	Matrix <double, 16, 1> ax;                	 // input task-space
	Matrix <double, 14, 1> aq;         	      	 // controller new input joint space
	Matrix <double, 14, 1> tau_task;		  	 // tau for primary task
	Matrix <double, 7, 1> tau_null_left;	  	 // tau for nullspace task on robot with ip 192.168.3.101
	Matrix <double, 7, 1> tau_null_right;	  	 // tau for nullspace task
	Matrix <double, 7, 1> tau_d_left;		  	 // final desired torque left arm	
	Matrix <double, 7, 1> tau_d_right;		  	 // final desired torque right arm	
	Matrix <double, 7, 7> N1;				  	 // null projector for left arm
	Matrix <double, 7, 7> N2;				  	 // null projector for left arm
    Matrix <double,16,16 > gain_p;            	 // proportional gain matrix
	Matrix <double,16,16 > gain_d;    	      	 // derivative gain matrix
	Matrix <double,16,16 > gain_i;    	      	 // derivative gain matrix
	gain_p.setZero(); gain_d.setZero(); gain_i.setZero(); 
	
// Set gains
	gain_p.block(0,0,8,8) << KP*I8; gain_p.block(8,8,8,8) << KP_ABS*I8;
	gain_d.block(0,0,8,8) << KD*I8; gain_d.block(8,8,8,8) << KD_ABS*I8;

// Control input task space
   ax << ddx_des + gain_d*de_aug + gain_p*e_aug  - J_aug_dot*dq;	

   J_aug_inv << pinv(J_aug); 

// Control input joint space

   aq << J_aug_inv*ax;
   
//---------------- CONTROL COMPUTATION -----------------//
	
	tau_task << Ca*dq + Ma*aq + ga_mario - ga;  

	
// //---------------- NULLSPACE CONTROL COMPUTATION -----------------//  

   N1 =  I7 -  pinv(J1)*J1; // null-space projector left arm
   N2 =  I7 -  pinv(J2)*J2; // null-space projector right arm

// 	//Dissipative term on joints + joints limits avoidance 

   tau_null_left << N1 *(2 * (q_nullspace_ - q_l)) - N1 *(0.5 * dq_l); 
   tau_null_right << N2 *(2 * (q_nullspace_ - q_r)) - N2 *(0.5 * dq_r); 
	
//----------------- FINAL CONTROL --------------------------------//
	
   tau_d_left << tau_task.head(7) + 0*tau_null_left; 
   tau_d_right << tau_task.tail(7) + 0*tau_null_right; 

// // 	//=================================| END CONTROL |==================================//
	
	// //----------- TORQUE SATURATION and COMMAND-------------//

	// Saturate torque rate to avoid discontinuities
	tau_d_left << saturateTorqueRate(arms_data_.at(left_arm_id_),tau_d_left, tau_J_d_left);
	tau_d_right << saturateTorqueRate(arms_data_.at(right_arm_id_),tau_d_right, tau_J_d_right);

	// Saturate torque to avoid torque limit
	double ith_torque_rate_l;
	for (int i = 0; i < 7; ++i){
		ith_torque_rate_l = std::abs(tau_d_left(i)/tau_limit(i));
		if( ith_torque_rate_l > 1)
			tau_d_left = tau_d_left / ith_torque_rate_l;
	}

	double ith_torque_rate_r;
	for (int i = 0; i < 7; ++i){
		ith_torque_rate_r = std::abs(tau_d_right(i)/tau_limit(i));
		if( ith_torque_rate_r > 1)
			tau_d_right = tau_d_right / ith_torque_rate_r;
	}

 	//// Set arm command torques
	for (size_t i = 0; i < 7; ++i) {
		arms_data_.at(left_arm_id_).joint_handles_[i].setCommand(tau_d_left(i));
		arms_data_.at(right_arm_id_).joint_handles_[i].setCommand(tau_d_right(i));
    }

	//  ============ ESTIMATED EXT WRENCHES VIA MOMENTUM OBSERVER=================  //

	int delta = 1000; 	//deltaT = 1s
	double gain_l,gain_r; 
	gain_l = KO;
	gain_r = KO; 

	Vector7d beta_r,beta_l,pr_int_hat, pl_int_hat;  
	Matrix<double,7,7> Cr_t, Cl_t; 
	Cr_t = c2_mario.transpose(); Cl_t = c1_mario.transpose(); 
	//Momentum
	p_l = m1_mario*dq_l;  
	p_r = m2_mario*dq_r;
	

	tau_measured_l = tau_measured_l - initial_tau_ext_l;
	
	tau_measured_r = tau_measured_r - initial_tau_ext_r; 

	
	if(count==0){
		r_l.setZero();
		r_r.setZero(); 
	}else{
		beta_l = g1_mario - Cl_t*dq_l; 
		beta_r = g2_mario - Cr_t*dq_r;
		pl_dot_hat = tau_measured_l - beta_l + r_l;
		pr_dot_hat = tau_measured_r - beta_r + r_r;

		pl_int_hat  = pl_dot_hat*(period.toSec()) + pl_int_hat;
		pr_int_hat  = pr_dot_hat*(period.toSec()) + pr_int_hat;

	for(int i = 0; i<7; i++){
		r_l(i) = gain_l*(p_l(i) - pl_int_hat(i) - p0_l(i));
		r_r(i) = gain_r*(p_r(i) - pr_int_hat(i) - p0_r(i));
		}

	}

	wrench_ext_l_hat = Jl_inv*r_l;
	wrench_ext_r_hat = Jr_inv*r_r;

	//Ext forces under disturbance treshold 
	if(abs(wrench_ext_l_hat (0))<=1.5 && abs(wrench_ext_l_hat (1))<=1.5 && abs(wrench_ext_l_hat(2))<=1.5){
		l_counter = l_counter + 1; 		
	}

	if(abs(wrench_ext_r_hat (0))<=1.5 && abs(wrench_ext_r_hat (1))<=1.5 && abs(wrench_ext_r_hat(2))<=1.5){
		r_counter = r_counter + 1; 		
	}

// // 	//======================| PUBLISHER|======================//

// // 	// ----------- COMMANDED TORQUES and EXTERNAL FORCES -------------//

	for(int i=0; i<7;i++){
		info_debug_msg.tau_left[i] = (tau_task.head(7))(i);
		info_debug_msg.tau_right[i] = (tau_task.tail(7))(i);
		info_debug_msg.tau_r_mes[i] = tau_measured_r(i)-g2_mario(i); 
		info_debug_msg.tau_l_mes[i] = tau_measured_l(i)-g1_mario(i); 
		info_debug_msg.r_r[i] = r_r(i); 
		info_debug_msg.r_l[i] = r_l(i);  
	}

	for(int i=0; i<6;i++){
		info_debug_msg.wrench_ext_1[i] = - wrench_ext_l(i); 
		info_debug_msg.wrench_ext_2[i] = - wrench_ext_r(i);
		info_debug_msg.f_ext_hat_r[i] = wrench_ext_r_hat(i);  
		info_debug_msg.f_ext_hat_l[i] = wrench_ext_l_hat(i); 
	}

// //  // ------------- ABSOLUTE AND RELATIVE POSES --------- // // 

	for(int i=0; i<3;i++){
		info_debug_msg.rel_pos[i] = pos_rel(i); 
		info_debug_msg.abs_pos[i] = pos_abs(i); 
		info_debug_msg.pos_1[i] = pos_l_(i); 
		info_debug_msg.pos_2[i] = pos_r_(i); 
		info_debug_msg.er[i] = e_pos_rel(i); 
		info_debug_msg.ea[i] = e_pos_abs(i); 
	}


	for(int i=0; i<4;i++){
		info_debug_msg.rot_r[i] = rot_r(i); 
		info_debug_msg.rot_a[i] = rot_a(i); 
	}

	info_debug_msg.abs_norm = e_pos_abs.norm();
	info_debug_msg.rel_norm = e_pos_rel.norm();

	for(int i=0; i<8;i++){
		info_debug_msg.rel_pose[i] = pose_rel(i); 
		info_debug_msg.abs_pose[i] = pose_abs(i); 
		info_debug_msg.x1[i] = x1(i);
		info_debug_msg.x2[i] = x2(i); 
	}

	// std::cout << "pose_abs" << pose_abs.transpose() << std::endl; 
    // std::cout << "pose_rel" << pose_rel.transpose() << std::endl; 

    pub_info_debug.publish(info_debug_msg);
	
	count = count+1;
	t = (ros::Time::now() - t_init).toSec();

	
}
// // //---------------------------------------------------------------//
// // //                    	TORQUE SATURATION                    	 //
// // //---------------------------------------------------------------//
Eigen::Matrix<double, 7, 1> DualArmControl::saturateTorqueRate(
    const FrankaDataContainer& arm_data,
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, arm_data.delta_tau_max_),
                                               -arm_data.delta_tau_max_);
  }
  return tau_d_saturated;
}


 
// // // //                          CALLBACKS		                     //
// // // //---------------------------------------------------------------//

// // //----------- DESIRED NOMINAL TRAJECTORY -------------//

void DualArmControl::desiredProjectTrajectoryCallback(
    	const panda_controllers::DesiredProjectTrajectoryConstPtr& msg) {
			for (int i=0; i<8; i++){
          		pose_a_d_(i) = msg->pose_d[i];
          		dpose_a_d_(i) = msg->dpose_d[i];
          		ddpose_a_d_(i) = msg->ddpose_d[i];
          		pose_r_d_(i) = msg->pose_r[i];
		  		dpose_r_d_(i) = msg->dpose_r[i];
		  		ddpose_r_d_(i) = msg->ddpose_r[i];
          }
 } 

// //----------- DESIRED COMPLIANT TRAJECTORY -------------//
void DualArmControl::CompliantTrajCallback(
    	const panda_controllers::CompliantTraj::ConstPtr& msg) {
			for (int i=0; i<8; i++){
          		// pose_a_d_(i) = msg->pose_abs_c[i];
          		// dpose_a_d_(i) = msg->dpose_abs_c[i];
          		// ddpose_a_d_(i) = msg->ddpose_abs_c[i];
          		// pose_r_d_(i) = msg->pose_rel_c[i];
		  		// dpose_r_d_(i) = msg->dpose_rel_c[i];
		  		// ddpose_r_d_(i) = msg->ddpose_rel_c[i];
          }
		}										
}  // end namespace panda_controllers

PLUGINLIB_EXPORT_CLASS(panda_controllers::DualArmControl,
                      controller_interface::ControllerBase);


