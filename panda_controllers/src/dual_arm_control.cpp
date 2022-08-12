#include <cmath>
#include <math.h>
#include <memory>
#include <functional>
#include <panda_controllers/dual_arm_control.h>

#include "dqrobotics/DQ.h"
#include "dqrobotics/robot_modeling/DQ_SerialManipulator.h"
#include "dqrobotics/utils/DQ_LinearAlgebra.h"
#include <dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <franka_hw/trigger_rate.h>
#include <pluginlib/class_list_macros.h>
#include <ros/transport_hints.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>


//DQ ROBOTICS
using DQ_robotics::DQ_Kinematics;
using DQ_robotics::DQ_SerialManipulator;
using DQ_robotics::DQ_CooperativeDualTaskSpace;
using DQ_robotics::E_;
using DQ_robotics::i_;
using DQ_robotics::j_;
using DQ_robotics::k_;
using DQ_robotics::C8;

using namespace DQ_robotics;

#define 	KP			    180   // proportional gain motion controller
#define 	KD			    30    // derivative gain motion controller
#define 	KP_ABS			80    // proportional gain motion controller
#define 	KD_ABS			20    // derivative gain motion controller
#define     KI              50   // integrative term 
#define 	D_JOINTS	    2    // dissipative term joints
#define 	COLL_LIMIT		25   // 
#define 	NULL_STIFF		2
#define 	JOINT_STIFF		{3000, 3000, 3000, 3000, 3000, 2000, 100}

//Set base frame two arms from calibration

namespace panda_controllers {


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

	pub_EE1_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("/motion_control_dq/franka_left_ee_pose", 1);
	pub_EE2_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("/motion_control_dq/franka_right_ee_pose", 1);
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
	//Eigen conversion joint angles and velocities
	
	Map<Matrix<double, 7, 1> > q_l_in(robot_state_left.q.data());                       // joint positions  [rad]
	Map<Matrix<double, 7, 1> > q_r_in(robot_state_right.q.data());                      // joint positions  [rad]
	
	//Augment joint vector
	q_in.head(7) << q_l_in;
	q_in.tail(7) << q_r_in;

	// ==========  BASE FRAMES (set for now base of right arm (ip: 3.100) as world frame) ======= ///
	Vector3d p_b_0_l; //position base frame left arm
	Vector4d r_b_0_l; //rotation base frame left arm (r,i,j,k)
	Vector3d p_b_0_r; //position base frame right arm
	Vector4d r_b_0_r; //rotation base frame right arm (r,i,j,k)
	p_b_0_l << -0.09,1.44,0; 
	r_b_0_l << 1,0,0,0;  
	p_b_0_r << 0,0,0;
	r_b_0_r << 1,0,0,0; 
	//////////////////////////////////////

	//Load Panda robot (left arm)
	DQ_SerialManipulator panda_left = init_dq_robot(p_b_0_l,r_b_0_l,0.1034);

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
 }

// //------------------------------------------------------------------------------//
// //                          	    UPDATE										//
// //------------------------------------------------------------------------------//

void DualArmControl::update(const ros::Time& /*time*/,
                                          const ros::Duration &period /*period*/) {

//----------- VARIABLES DECLARATIONS and DEFINITIONS -------------//
//Load Dual-Arm system
	// ==========  BASE FRAMES (set for now base of right arm (ip: 3.100) as world frame) ======= ///
	Vector3d p_b_0_l; //position base frame left arm
	Vector4d r_b_0_l; //rotation base frame left arm (r,i,j,k)
	Vector3d p_b_0_r; //position base frame right arm
	Vector4d r_b_0_r; //rotation base frame right arm (r,i,j,k)
	p_b_0_l << -0.09,1.44,0; 
	r_b_0_l << 1,0,0,0;  
	p_b_0_r << 0,0,0;
	r_b_0_r << 1,0,0,0; 
	//////////////////////////////////////

//============= LOAD CDTS MODEL ======================= //

	DQ_SerialManipulator panda_left = init_dq_robot(p_b_0_l,r_b_0_l,0.1034);
	DQ_SerialManipulator panda_right = init_dq_robot(p_b_0_r,r_b_0_r,0);
	DQ_CooperativeDualTaskSpace dual_panda = init_dual_panda(&panda_left, &panda_right);

 	// Control Variables
	I8 = MatrixXd::Identity(8, 8);
	I7 = MatrixXd::Identity(7, 7);
	I16 = MatrixXd::Identity(16, 16);
	Vector8d er;  		          // relative pose error  8x1
	Vector8d ea; 		          // absolute pose error  8x1
	Vector8d der;                 // relative vel error   8x1
	Vector8d dea;                 // absolute vel error   8x1
 	Vector16d e_aug;              // agumented error vector 16x1 [er;ea];
 	Vector16d de_aug;             // agumented error vector derivative 16x1 [der;dea];
	Vector16d ddx_des;            // augmented desired acceleration [ddxr_d;ddxa_d]
	Vector8d pose_rel;       	  // current relative pose 8x1
	Vector8d dpose_rel;			  // current relative velocity 8x1
	Vector8d pose_abs;       	  // current absolute pose 8x1
	Vector8d dpose_abs;			  // current absolute velocity 8x1
	Vector3d pos_rel;             // current relative position 3x1
	Vector3d pos_abs;             // current absolute position 3x1
	Vector3d e_pos_rel;           // rel position error (for debug)
	Vector3d e_pos_abs;           // abs position error
	int count = 0; 

////------DQ Variables-----//
	DQ pos_d_dq; 
	                               
	// Jacobians
	Matrix<double, 8, 7>   J1;             // pose jacobian left arm
	Matrix<double, 8, 7>   J2;             // pose jacobian right arm
 	Matrix<double, 8, 14>  Jr;             // relative pose jacobian
	Matrix<double, 6, 14>  Jg;             // geomtrice Jacobian
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
	Matrix<double, 14, 16> J_dyn_inv;      // augmented jacobian pseudo-inverse 
    Matrix<double, 16, 14> J_aus;     

// // ============================== GET ARMS STATE ======================== //

	franka::RobotState robot_state_right = arms_data_.at(right_arm_id_).state_handle_->getRobotState();
 	franka::RobotState robot_state_left = arms_data_.at(left_arm_id_).state_handle_->getRobotState();

	//Eigen conversion joint angles and velocities

	//Left arm
	Map<Matrix<double, 7, 1> > q_l(robot_state_left.q.data());                      // joint positions  [rad]
	Map<Matrix<double, 7, 1> > dq_l(robot_state_left.dq.data());  
	//Right arm
	Map<Matrix<double, 7, 1> > q_r(robot_state_right.q.data());                      // joint positions  [rad]
	Map<Matrix<double, 7, 1> > dq_r(robot_state_right.dq.data());  

	//Augmented joint vector
	q.head(7) << q_l;
	q.tail(7) << q_r;
	dq.head(7) << dq_l;
	dq.tail(7) << dq_r;  

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
	Matrix<double, 14, 1> Ca; //stacked coriolis vector
	Ma.setZero();
	Ca.setZero(); 

	std::array<double, 49> mass_array_right = arms_data_.at(right_arm_id_).model_handle_->getMass();
	std::array<double, 7>  coriolis_array_right = arms_data_.at(right_arm_id_).model_handle_->getCoriolis();
	
	std::array<double, 49> mass_array_left = arms_data_.at(left_arm_id_).model_handle_->getMass();
	std::array<double, 7>  coriolis_array_left = arms_data_.at(left_arm_id_).model_handle_->getCoriolis();
	
	// Eigen conversion
	Map<Matrix<double, 7, 7> > m1(mass_array_left.data());                      // mass matrix [kg]
	Map<Matrix<double, 7, 7> > m2(mass_array_right.data());      
	Map<Matrix<double, 7, 1> > c1(coriolis_array_left.data());                 // coriolis forces  [Nm]
	Map<Matrix<double, 7, 1> > c2(coriolis_array_right.data());    
    
	Ma.block(0,0,7,7) << m1;
	Ma.block(7,7,7,7) << m2;

	Ca.head(7) << c1;
	Ca.tail(7) << c2;

	Map<Matrix<double, 7, 1> > tau_J_d_left(robot_state_left.tau_J_d.data());          // previous cycle commanded torques [Nm]
	Map<Matrix<double, 6, 1> > wrench_ext_l(robot_state_left.O_F_ext_hat_K.data());    // external wrench [N] exerted by arm1 wrt base frame
	Map<Matrix<double, 7, 1> > tau_J_d_right(robot_state_right.tau_J_d.data());        // previous cycle commanded torques [Nm]
	Map<Matrix<double, 6, 1> > wrench_ext_r(robot_state_right.O_F_ext_hat_K.data());   // external wrench [N] exerted by arm2 wrt base frame
	
// 	Get current CDTS variables
    DQ pose_rel_dq;
    pose_rel_dq = dual_panda.relative_pose(q); 
	pose_rel = vec8(pose_rel_dq); 
	
	DQ pose_abs_dq; 
	pose_abs_dq = dual_panda.absolute_pose(q);
	pose_abs = vec8(pose_abs_dq); 

//  Store absolute and relative positions 
    pose_rel_dq = pose_rel_dq.normalize(); pose_abs_dq = pose_abs_dq.normalize();
	pos_rel = vec3(pose_rel_dq.translation()); pos_abs = vec3(pose_abs_dq.translation()); 

// // ================= GET JACOBIANS ===================== //

	J1 << dual_panda.pose_jacobian1(q);
	J2 << dual_panda.pose_jacobian2(q); 
	
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

	Jg << geomJ(Ja,pose_abs_dq); 
	
	Jr_dot = (Jr - Jr_old)/(period.toSec()); 
	Ja_dot = (Ja - Ja_old)/(period.toSec()); 
	
	dpose_rel << Jr*dq; 
	dpose_abs << Ja*dq; 

	Vector6d twist;
	twist = Jg*dq; 

	J_aug.topRows(8) << Jr;
	J_aug.bottomRows(8) << Ja; 

	J_aug_dot.topRows(8) << Jr_dot; 
	J_aug_dot.bottomRows(8) << Ja_dot; 
	
// //  -------------- PUBLISH MATRICES FOR PLANNING -------------//

	robot_state_msg.header.stamp = ros::Time::now();
	std::copy(mass_array_left.begin(), mass_array_left.end(), robot_state_msg.m1.begin());
	std::copy(mass_array_right.begin(), mass_array_right.end(), robot_state_msg.m2.begin());

	pub_robot_state_.publish(robot_state_msg);

// //================= COMPUTE ERRORS ================//

	//Relative pose error
	er << pose_r_d_ - pose_rel;
	der << dpose_r_d_ - dpose_rel;
    
	//Absolute pose error
	ea << pose_a_d_- pose_abs; 
	dea << dpose_a_d_ - dpose_abs; 

	ddx_des.head(8) << ddpose_r_d_;
	ddx_des.tail(8) << ddpose_a_d_; 

	e_aug.head(8) << er;
	e_aug.tail(8) << ea; 
	de_aug.head(8) << der;
	de_aug.tail(8) << dea; 
    
	// ==== Store variables for analysis === //
	
	DQ des_pos_a_dq; Vector3d des_pos_abs;  DQ des_pos_r_dq; Vector3d des_pos_rel; 
	Vector3d n_abs, des_n_abs, n_rel,des_n_rel; double phi_abs, des_phi_abs, phi_rel,des_phi_rel;  

	//des absolute pos and rot
    des_pos_a_dq = (DQ(pose_a_d_)).normalize(); 
	des_pos_abs = vec3(des_pos_a_dq.translation()); 
	des_n_abs = vec3(des_pos_a_dq.rotation_axis()); des_phi_abs = double(des_pos_a_dq.rotation_angle()); 
	//des relative pos and rot
	des_pos_r_dq = (DQ(pose_r_d_)).normalize();
	des_pos_rel = vec3(des_pos_r_dq.translation()); 
	des_n_rel = vec3(des_pos_r_dq.rotation_axis()); des_phi_rel = double(des_pos_r_dq.rotation_angle()); 

	//current abs and rel rot
	n_abs = vec3(pose_abs_dq.rotation_axis()); phi_abs = double(pose_abs_dq.rotation_angle());
	n_rel = vec3(pose_rel_dq.rotation_axis()); phi_rel = double(pose_rel_dq.rotation_angle());

	//position error
    e_pos_rel = des_pos_rel - pos_rel; 
    e_pos_abs = des_pos_abs - pos_abs; 

//  //======================| CONTROL VARIABLES |======================//
    
	Matrix <double, 16, 1> ax;                // input task-space
	Matrix <double, 14, 1> aq;         	      // controller new input joint space
	Matrix <double, 14, 1> tau_task;		  // tau for primary task
	Matrix <double, 7, 1> tau_null_left;	  // tau for nullspace task on robot with ip 192.168.3.101
	Matrix <double, 7, 1> tau_null_right;	  // tau for nullspace task
	Matrix <double, 7, 1> tau_d_left;		  // final desired torque left arm	
	Matrix <double, 7, 1> tau_d_right;		  // final desired torque right arm	
	Matrix <double, 7, 7> N1;				  // null projector for left arm
	Matrix <double, 7, 7> N2;				  // null projector for left arm
    Matrix <double,16,16 > gain_p;            // proportional gain matrix
	Matrix <double,16,16 > gain_d;    	      // derivative gain matrix
	gain_p.setZero();
	gain_d.setZero();
	
	//setgains
	gain_p.block(0,0,8,8) << KP*I8;
	gain_p.block(8,8,8,8) << KP_ABS*I8;
	gain_d.block(0,0,8,8) << KD*I8;
	gain_d.block(8,8,8,8) << KD_ABS*I8;


// Control input task space
   //put the gains of relative 10 times bigger
   ax << ddx_des + gain_d*de_aug + gain_p*e_aug  - J_aug_dot*dq;	
   J_aug_inv << pinv(J_aug); 

// Control input joint space

   aq << J_aug_inv*ax;
   
//---------------- CONTROL COMPUTATION -----------------//
	
   tau_task << Ca + Ma*aq;  // +g (already compensated)
	
// //---------------- NULLSPACE CONTROL COMPUTATION -----------------//  

   N1 =  I7 -  pinv(J1)*J1; // null-space projector left arm
   N2 =  I7 -  pinv(J2)*J2; // null-space projector right arm

// 	//Dissipative term on joints + joints limits avoidance 

   tau_null_left << N1 *(NULL_STIFF * (q_nullspace_ - q_l)) - N1 *(0.5 * dq_l); 
   tau_null_right << N2 *(NULL_STIFF * (q_nullspace_ - q_r)) - N2 *(0.5 * dq_r); 
	
//----------------- FINAL CONTROL --------------------------------//
	
   tau_d_left << tau_task.head(7) + tau_null_left; 
   tau_d_right << tau_task.tail(7) + tau_null_right; 
  
	
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


 	////Set arm command torques
	for (size_t i = 0; i < 7; ++i) {
		arms_data_.at(left_arm_id_).joint_handles_[i].setCommand(tau_d_left(i));
		arms_data_.at(right_arm_id_).joint_handles_[i].setCommand(tau_d_right(i));
    }

// // 	//======================| PUBLISHER|======================//

// // 	// ----------- COMMANDED TORQUES and EXTERNAL FORCES -------------//

	for(int i=0; i<7;i++){
		info_debug_msg.tau_left[i] = (tau_task.head(7))(i);
		info_debug_msg.tau_right[i] = (tau_task.tail(7))(i);
	}

	for(int i=0; i<6;i++){
		info_debug_msg.wrench_ext_1[i] = - wrench_ext_l(i); 
		info_debug_msg.wrench_ext_2[i] = - wrench_ext_r(i); 
	}

// //  // ------------- ABSOLUTE AND RELATIVE POSES --------- // // 

	for(int i=0; i<3;i++){
		info_debug_msg.rel_pos[i] = pos_rel(i); 
		info_debug_msg.abs_pos[i] = pos_abs(i); 
		info_debug_msg.pos_1[i] = pos_l_(i); 
		info_debug_msg.pos_2[i] = pos_r_(i); 
		info_debug_msg.er[i] = e_pos_rel(i); 
		info_debug_msg.ea[i] = e_pos_abs(i); 
		info_debug_msg.e_n_abs[i] = des_n_abs(i)-n_abs(i); 
		info_debug_msg.e_n_rel[i] = des_n_rel(i)-n_rel(i); 
		info_debug_msg.vel[i] = (twist.tail(3))(i); 
		info_debug_msg.omega[i] = (twist.head(3))(i); 
	}

    info_debug_msg.e_phi_abs = des_phi_abs - phi_abs;
    info_debug_msg.e_phi_rel = des_phi_rel - phi_rel;
	info_debug_msg.abs_norm = e_pos_abs.norm();
	info_debug_msg.rel_norm = e_pos_rel.norm();

	for(int i=0; i<8;i++){
		info_debug_msg.rel_pose[i] = pose_rel(i); 
		info_debug_msg.abs_pose[i] = pose_abs(i); 
		info_debug_msg.x1[i] = x1(i);
		info_debug_msg.x2[i] = x2(i); 
	}

    pub_info_debug.publish(info_debug_msg);
	
	count = count+1; 

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


MatrixXd DualArmControl::geomJ(const MatrixXd& absoluteposeJ, const DQ& absolutepose) {
  Matrix<double, 8, 1> v;
  Matrix<double, 3, 4> CJ4_2_J3;
  MatrixXd J;
  v << -1, 1, 1, 1, -1, 1, 1, 1;
  MatrixXd C8 = v.array().matrix().asDiagonal();
  MatrixXd C4m = -C8.block(0, 0, 4, 4);
  CJ4_2_J3 << 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  DQ xm = absolutepose;
  if (absoluteposeJ.cols() == 14) {
    J.resize(6, 14);
    J.block(0, 0, 3, 14) =
        CJ4_2_J3 * 2 * xm.P().conj().haminus4() * absoluteposeJ.topRows(4);
    J.block(3, 0, 3, 14) =
        CJ4_2_J3 * 2 *
        (xm.D().hamiplus4() * C4m * absoluteposeJ.topRows(4) +
         xm.P().conj().haminus4() * absoluteposeJ.middleRows(4, 4));
  } else {
    J.resize(6, 7);
    J.block(0, 0, 3, 7) =
        CJ4_2_J3 * 2 * xm.P().conj().haminus4() * absoluteposeJ.topRows(4);
    J.block(3, 0, 3, 7) =
        CJ4_2_J3 * 2 *
        (xm.D().hamiplus4() * C4m * absoluteposeJ.topRows(4) +
         xm.P().conj().haminus4() * absoluteposeJ.middleRows(4, 4));
  }
  return J;
}
 
// // // //                          CALLBACKS		                     //
// // // //---------------------------------------------------------------//

// // //----------- DESIRED NOMINAL TRAJECTORY -------------//

void DualArmControl::desiredProjectTrajectoryCallback(
    	const panda_controllers::DesiredProjectTrajectoryConstPtr& msg) {
			for (int i=0; i<8; i++){
          		// pose_a_d_(i) = msg->pose_d[i];
          		// dpose_a_d_(i) = msg->dpose_d[i];
          		// ddpose_a_d_(i) = msg->ddpose_d[i];
          		// pose_r_d_(i) = msg->pose_r[i];
		  		// dpose_r_d_(i) = msg->dpose_r[i];
		  		// ddpose_r_d_(i) = msg->ddpose_r[i];
          }
 } 

// //----------- DESIRED COMPLIANT TRAJECTORY -------------//
void DualArmControl::CompliantTrajCallback(
    	const panda_controllers::CompliantTraj::ConstPtr& msg) {
			for (int i=0; i<8; i++){
          		pose_a_d_(i) = msg->pose_abs_c[i];
          		dpose_a_d_(i) = msg->dpose_abs_c[i];
          		ddpose_a_d_(i) = msg->ddpose_abs_c[i];
          		pose_r_d_(i) = msg->pose_rel_c[i];
		  		dpose_r_d_(i) = msg->dpose_rel_c[i];
		  		ddpose_r_d_(i) = msg->ddpose_rel_c[i];
          }
		}										
}  // end namespace panda_controllers


PLUGINLIB_EXPORT_CLASS(panda_controllers::DualArmControl,
                      controller_interface::ControllerBase);


