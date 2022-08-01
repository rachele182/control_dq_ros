#include <cmath>
#include <math.h>
#include <memory>
#include <panda_controllers/dual_arm_control.h>

#include "dqrobotics/DQ.h"
#include "dqrobotics/robot_modeling/DQ_SerialManipulator.h"
#include "dqrobotics/utils/DQ_LinearAlgebra.h"
#include <dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <franka_hw/trigger_rate.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

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

#define 	KP			    100  // proportional gain motion controller
#define 	KD			    20   // derivative gain motion controller
#define     KI              50   // integrative term 
#define 	D_JOINTS	    2    // dissipative term joints
#define 	COLL_LIMIT		25   // 
#define 	NULL_STIFF		2
#define 	JOINT_STIFF		{3000, 3000, 3000, 3000, 3000, 2000, 100}
//Set base frame two arms
double p_b_0_l[3]; //position base frame left arm
double r_b_0_l[4]; //rotation base frame left arm (r,i,j,k)
double p_b_0_r[3]; //position base frame right arm
double r_b_0_r[4]; //rotation base frame right arm (r,i,j,k)


namespace panda_controllers {


// ---------- DQ_SERIAL_MANIPULATOR PANDA ROBOT ------- //
DQ_SerialManipulator DualArmControl::init_dq_robot(double r_B_O_[3],double B_Q_O_[4]) {

	Matrix<double, 4, 7> franka_dh(4, 7);
  	franka_dh << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // theta (q)
    0.333, 0.0, 0.316, 0.0, 0.384, 0.0,
    0.1034+0.107,  // 
    0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088,             // a
    0.0, -pi / 2, pi / 2, pi / 2, -pi / 2, pi / 2, pi / 2;  // alpha
	DQ_SerialManipulator robot(franka_dh, "modified");
  	DQ p = DQ(0.0, r_B_O_[0], r_B_O_[1], r_B_O_[2]);
  	DQ r = DQ(B_Q_O_[0], B_Q_O_[1], B_Q_O_[2], B_Q_O_[3]);
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

	// sub_des_traj_proj_ =  node_handle.subscribe(  "/motion_control_dq/compliant_traj", 1, 
	// 												&DualArmControl::CompliantTrajCallback, this,
	// 												ros::TransportHints().reliable().tcpNoDelay());

    sub_nom_traj_proj_ =  node_handle.subscribe(  "/motion_control_dq/dq_trajectory", 1, 
													&DualArmControl::desiredProjectTrajectoryCallback, this,
													ros::TransportHints().reliable().tcpNoDelay());												

	// pub_pos_error =         node_handle.advertise<geometry_msgs::TwistStamped>("/motion_control_dq/pos_error", 1);
	// pub_endeffector_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("/motion_control_dq/franka_ee_pose", 1);
	// pub_robot_state_ =      node_handle.advertise<panda_controllers::RobotState>("/motion_control_dq/robot_state", 1);
	// pub_info_debug =        node_handle.advertise<panda_controllers::InfoDebug>("/motion_control_dq/info_debug", 1);


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
	Map<Matrix<double, 7, 1> > q_l_in(robot_state_left.q.data());                      // joint positions  [rad]
	Map<Matrix<double, 7, 1> > q_r_in(robot_state_right.q.data());                      // joint positions  [rad]
	
	//Augment joint vector
	q_in.head(7) << q_l_in;
	q_in.tail(7) << q_r_in;
	
	// Initial position and orientation Left arm
	pos_in_l_ = transform_l.translation();
	orientation_l_ = Quaterniond(transform_l.linear());
	or_d_l << orientation_l_.w(), orientation_l_.x(),orientation_l_.y(),orientation_l_.z();

	// Initial position and orientation roght arm
	pos_in_r_ = transform_r.translation();
	orientation_r_ = Quaterniond(transform_r.linear());
	or_d_r << orientation_r_.w(), orientation_r_.x(),orientation_r_.y(),orientation_r_.z();


	// Load Panda robot (left arm)
	DQ_SerialManipulator panda_left = init_dq_robot(p_b_0_l,r_b_0_l);

	// Load Panda robot (right arm)
	DQ_SerialManipulator panda_right = init_dq_robot(p_b_0_r,r_b_0_r);

	// Cooperative dual arm system
	DQ_CooperativeDualTaskSpace dual_panda = init_dual_panda(&panda_left, &panda_right);

	//Relative pose
	DQ pose_rel_dq;  //relative pose
	pose_rel_dq = dual_panda.relative_pose(q); 
	pose_rel_dq = pose_rel_dq.normalize();
	rel_pose_in_ = vec8(pose_rel_dq); 

	//Absolute pose
	DQ pose_abs_dq; //absolute pose
	pose_abs_dq = dual_panda.absolute_pose(q); 
	pose_abs_dq = pose_abs_dq.normalize();
	abs_pose_in_ = vec8(pose_abs_dq); 

	// set equilibrium point to current state
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
	DQ_SerialManipulator panda_left = init_dq_robot(p_b_0_l,r_b_0_l);
	DQ_SerialManipulator panda_right = init_dq_robot(p_b_0_r,r_b_0_r);
	DQ_CooperativeDualTaskSpace dual_panda = init_dual_panda(&panda_left, &panda_right);

	// Control Variables
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

    //------DQ Variables-----//
	DQ pos_d_dq; 
	                               
	// Jacobians
	Matrix<double, 8, 7>   J1;             // pose jacobian left arm
	Matrix<double, 8, 7>   J2;             // pose jacobian right arm
    Matrix<double, 8, 14>  Jr;             // relative pose jacobian
    Matrix<double, 8, 14>  Jr_old;         // previous computed relative pose jacobian
	Matrix<double, 8, 14>  Ja;             // absolute pose jacobian 
	Matrix<double, 8, 14>  Ja_old;         // previous compute absolute pose jacobian
	Matrix<double, 8, 14>  Jr_dot;         // relative pose derivative jacobian 
	Matrix<double, 8, 14>  Ja_dot;          // absolute pose derivative jacobian
	Matrix<double, 16, 14> J_aug;         // augmented Jacobian [Jr,Ja]
	Matrix<double, 16, 14> J_aug_dot;     // derivative augmented Jacobian [Jr_dot,Ja_dot]
	Matrix<double, 14, 16> J_aug_inv;     // augmented jacobian pseudo-inverse 
	
    
// ============================== GET ARMS STATE ======================== //

	franka::RobotState robot_state_right = arms_data_.at(right_arm_id_).state_handle_->getRobotState();
  	franka::RobotState robot_state_left = arms_data_.at(left_arm_id_).state_handle_->getRobotState();
	Affine3d transform_l(Matrix4d::Map(  // NOLINT (readability-identifier-naming)
      robot_state_left.O_T_EE.data()));           // NOLINT (readability-identifier-naming)
  	Affine3d transform_r(Matrix4d::Map(  // NOLINT (readability-identifier-naming)
      robot_state_right.O_T_EE.data()));          // NOLINT (readability-identifier-naming)

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

//================================ GET DYNAMICS MODEL ================  //

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
    pose_rel_dq = pose_rel_dq.normalize();
	pose_abs_dq = pose_abs_dq.normalize();
	pos_rel = vec3(pose_rel_dq.translation());
	pos_abs = vec3(pose_abs_dq.translation()); 
	
// ================= GET JACOBIANS ===================== //

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

	Jr_dot = (Jr - Jr_dot)/(period.toSec()); 
	Ja_dot = (Ja - Ja_dot)/(period.toSec()); 
	
	dpose_rel << Jr*dq; 
	dpose_abs << Ja*dq; 

	J_aug.topRows(8) << Jr;
	J_aug.bottomRows(8) << Ja; 
	J_aug_dot.topRows(8) << Jr_dot; 
	J_aug_dot.bottomRows(8) << Ja_dot; 

//  -------------- PUBLISH MATRICES FOR PLANNING -------------//

	robot_state_msg.header.stamp = ros::Time::now();
	std::copy(mass_array_left.begin(), mass_array_left.end(), robot_state_msg.m1.begin());
	std::copy(mass_array_right.begin(), mass_array_right.end(), robot_state_msg.m2.begin());

	pub_robot_state_.publish(robot_state_msg);

//================= COMPUTE ERRORS ================//
	
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
    
	//Store position error for analysis
	DQ er_dq;
	DQ ea_dq; 
    er_dq = DQ(er).normalize(); 
	ea_dq = DQ(ea).normalize(); 
    e_pos_rel <<  vec3(er_dq.translation());
    e_pos_abs <<  vec3(ea_dq.translation());  


 //======================| CONTROL VARIABLES |======================//
    
	Matrix <double, 16, 1> ax;                // input task-space
	Matrix <double, 14, 1> aq;         	      // controller new input joint space
	Matrix <double, 14, 1> tau_task;		  // tau for primary task
	Matrix <double, 7, 1> tau_null_left;	  // tau for nullspace task
	Matrix <double, 7, 1> tau_null_right;	  // tau for nullspace task
	Matrix <double, 7, 1> tau_d_left;		  // final desired torque left arm	
	Matrix <double, 7, 1> tau_d_right;		  // final desired torque right arm	
	Matrix <double, 7, 7> N1;				  // null projector for left arm
	Matrix <double, 7, 7> N2;				  // null projector for left arm

	I8 = MatrixXd::Identity(8, 8);
	I7 = MatrixXd::Identity(7, 7);
	I16 = MatrixXd::Identity(16, 16);
    
// Control input task space
   ax << ddx_des + KD*I16*de_aug + KP*I16*e_aug  - J_aug_dot*dq;
	
   J_aug_inv << pinv(J_aug); 

// Control input joint space

   aq << J_aug_inv*ax;

//---------------- CONTROL COMPUTATION -----------------//
	
   tau_task << Ca + Ma*aq;  // +g (already compensated)
	
//---------------- NULLSPACE CONTROL COMPUTATION -----------------//  

   N1 =  I7 -  pinv(J1)*J1; // null-space projector left arm
   N2 =  I7 -  pinv(J2)*J2; // null-space projector right arm

// 	//Dissipative term on joints + joints limits avoidance 

 tau_null_left << N1 *(NULL_STIFF * (q_nullspace_ - q_l)) - N1 *(0.5 * dq_l); 
 tau_null_right << N2 *(NULL_STIFF * (q_nullspace_ - q_r)) - N2 *(0.5 * dq_r); 
	
//----------------- FINAL CONTROL --------------------------------//
	
  tau_d_left << tau_task.head(7) + tau_null_left; 
  tau_d_right << tau_task.tail(7) + tau_null_right; 

	
// 	//=================================| END CONTROL |==================================//
	
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


	//set arm command torques
	for (size_t i = 0; i < 7; ++i) {
		arms_data_.at(left_arm_id_).joint_handles_[i].setCommand(tau_d_left(i));
		arms_data_.at(right_arm_id_).joint_handles_[i].setCommand(tau_d_right(i));
    }

// 	//======================| PUBISH & SUBSCRIBE |======================//

// 	//----------- DEBUG MSG -------------//
    
// 	// ----------- COMMANDED TORQUES and EXTERNAL FORCES -------------//

// 	for(int i=0; i<7;i++){
// 		info_debug_msg.tau_d[i] = tau_d(i);
// 		info_debug_msg.tau_null[i] = tau_nullspace(i);
// 		info_debug_msg.tau_task[i] = tau_task(i);
// 	}

// 	for(int i=0; i<6;i++){
// 		info_debug_msg.wrench_ext[i] = - wrench_ext(i); 
// 	}
    

// // 	//----------- POSITION ERROR -------------//

// 	info_debug_msg.header.stamp = ros::Time::now();
// 	info_debug_msg.pos_curr[0] = pos(0); // position error x-axis
// 	info_debug_msg.pos_curr[1] = pos(1); // position error y-axis
// 	info_debug_msg.pos_curr[2] = pos(2); // position error z-axis
// 	info_debug_msg.pos_error[0] = pos_error(0); // position error x-axis
// 	info_debug_msg.pos_error[1] = pos_error(1); // position error y-axis
// 	info_debug_msg.pos_error[2] = pos_error(2); // position error z-axis
// 	info_debug_msg.norma[0] = norma(0); // position error norm x-axis
// 	info_debug_msg.norma[1] = norma(1); // position error norm y-axis
// 	info_debug_msg.norma[2] = norma(2); // position error norm z-axis

//  	pub_info_debug.publish(info_debug_msg);

// // // ------------Check des subscribed -------//
	

// // 	//----------- EE-POSE -------------//

// 	geometry_msgs::PoseStamped position_endeff;
// 	position_endeff.pose.position.x = position.x();
// 	position_endeff.pose.position.y = position.y();
// 	position_endeff.pose.position.z = position.z();
// 	position_endeff.pose.orientation.w = orientation.w();
//  position_endeff.pose.orientation.x = orientation.x();
//  position_endeff.pose.orientation.y = orientation.y();
//  position_endeff.pose.orientation.z = orientation.z();


// 	pub_endeffector_pose_.publish(position_endeff);

// }
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

// void DualArmControl::desiredProjectTrajectoryCallback(
//     	const panda_controllers::DesiredProjectTrajectoryConstPtr& msg) {
  
// 	pose_n_ << msg->pose_d[0], msg->pose_d[1], msg->pose_d[2],msg->pose_d[3],msg->pose_d[4],msg->pose_d[5],msg->pose_d[6],msg->pose_d[7];		
// 	dpose_n_ << msg->dpose_d[0], msg->dpose_d[1], msg->dpose_d[2],msg->dpose_d[3],msg->dpose_d[4],msg->dpose_d[5],msg->dpose_d[6],msg->dpose_d[7];	
// 	ddpose_n_ << msg->ddpose_d[0], msg->ddpose_d[1], msg->ddpose_d[2],msg->ddpose_d[3],msg->ddpose_d[4],msg->ddpose_d[5],msg->ddpose_d[6],msg->ddpose_d[7];	
//  } 

// // //----------- DESIRED COMPLIANT TRAJECTORY -------------//
// void DualArmControl::CompliantTrajCallback(
//     	const panda_controllers::CompliantTraj::ConstPtr& msg) {
  
// 	pose_d_ << msg->pose_c[0], msg->pose_c[1], msg->pose_c[2],msg->pose_c[3],msg->pose_c[4],msg->pose_c[5],msg->pose_c[6],msg->pose_c[7];		
// 	dpose_d_ << msg->dpose_c[0], msg->dpose_c[1], msg->dpose_c[2],msg->dpose_c[3],msg->dpose_c[4],msg->dpose_c[5],msg->dpose_c[6],msg->dpose_c[7];	
// 	ddpose_d_ << msg->ddpose_c[0], msg->ddpose_c[1], msg->ddpose_c[2],msg->ddpose_c[3],msg->ddpose_c[4],msg->ddpose_c[5],msg->ddpose_c[6],msg->ddpose_c[7];	
//  } 
										  

}  // end namespace franka_softbots


PLUGINLIB_EXPORT_CLASS(panda_controllers::DualArmControl,
                      controller_interface::ControllerBase);


