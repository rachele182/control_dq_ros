#include <cmath>
#include <math.h>
#include <memory>
#include <panda_controllers/motion_control_dq.h>

#include "dqrobotics/DQ.h"
#include "dqrobotics/robot_modeling/DQ_SerialManipulator.h"
#include "dqrobotics/utils/DQ_LinearAlgebra.h"

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <franka_hw/trigger_rate.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "utils/pseudo_inversion.h"
// #include "utils/lowpass_filter.h"

using DQ_robotics::DQ_SerialManipulator;
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

namespace panda_controllers {
	
// ---------- DQ ROBOT INIT ------- //
DQ_SerialManipulator MotionControlDq::init_dq_robot() {
	double r_B_O_[3] = {0.0, 0.0, 0.0};         // initial base translation
	double B_Q_O_[4] = {1.0, 0.0, 0.0,0.0};    // intial base rotation

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

// //------------------------------------------------------------------------------//
// //                          		INIT										//
// //------------------------------------------------------------------------------//
bool MotionControlDq::init( hardware_interface::RobotHW* robot_hw, 
                                       ros::NodeHandle& node_handle) {
	//Name space extraction to add a prefix to the topic name
	int n = 0;
	name_space = node_handle.getNamespace();
	n = name_space.find("/", 2);
	name_space = name_space.substr(0,n);
										

	//--------------- INITIALIZE SUBSCRIBERS AND PUBLISHERS -----------------//

	sub_des_traj_proj_ =  node_handle.subscribe(  "/motion_control_dq/compliant_traj", 1, 
													&MotionControlDq::CompliantTrajCallback, this,
													ros::TransportHints().reliable().tcpNoDelay());

    sub_nom_traj_proj_ =  node_handle.subscribe(  "/motion_control_dq/dq_trajectory", 1, 
													&MotionControlDq::desiredProjectTrajectoryCallback, this,
													ros::TransportHints().reliable().tcpNoDelay());												

	pub_pos_error =         node_handle.advertise<geometry_msgs::TwistStamped>("/motion_control_dq/pos_error", 1);
	pub_endeffector_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("/motion_control_dq/franka_ee_pose", 1);
	pub_robot_state_ =      node_handle.advertise<panda_controllers::RobotState>("/motion_control_dq/robot_state", 1);
	pub_info_debug =        node_handle.advertise<panda_controllers::InfoDebug>("/motion_control_dq/info_debug", 1);


	//---------------- INITIALIZE SERVICE CLIENTS ------------------//

	collBehaviourClient = node_handle.serviceClient<franka_msgs::SetFullCollisionBehavior>( name_space 
																							+ "/franka_control/set_full_collision_behavior");

	jointImpedanceClient = node_handle.serviceClient<franka_msgs::SetJointImpedance>( name_space 
																						+ "/franka_control/set_joint_impedance");


	//----- INITIALIZE NODE, ROBOT HANDLER AND ROBOT INTERFACE -----//

	std::string arm_id;
	if (!node_handle.getParam("arm_id", arm_id)) {
		ROS_ERROR_STREAM("MotionControlDq: Could not read parameter arm_id");
		return false;
	}
	std::vector<std::string> joint_names;
	if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
		ROS_ERROR(
				"MotionControlDq: Invalid or no joint_names parameters provided, "
				"aborting controller init!");
		return false;
	}
	if (!node_handle.getParam("var_damp", var_damp)) {
		ROS_ERROR_STREAM("MotionControlDq: Could not read parameter var_damp");
		return false;
	}

	franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
	if (model_interface == nullptr) {
		ROS_ERROR_STREAM("MotionControlDq: Error getting model interface from hardware");
		return false;
	}
	try {
		model_handle_.reset(
				new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM(
				"MotionControlDq: Exception getting model handle from interface: "
				<< ex.what());
		return false;
	}

	franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
	if (state_interface == nullptr) {
		ROS_ERROR_STREAM("MotionControlDq: Error getting state interface from hardware");
		return false;
	}
	try {
		state_handle_.reset(
				new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM(
				"MotionControlDq: Exception getting state handle from interface: "
				<< ex.what());
		return false;
	}

	hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
	if (effort_joint_interface == nullptr) {
		ROS_ERROR_STREAM("MotionControlDq: Error getting effort joint interface from hardware");
		return false;
	}
	for (size_t i = 0; i < 7; ++i) {
		try {
			joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
		} catch (const hardware_interface::HardwareInterfaceException& ex) {
			ROS_ERROR_STREAM(
					"MotionControlDq: Exception getting joint handles: " << ex.what());
		return false;
		}
	}
    
    
	//---------------- INITIALIZE VARIABLES ------------------//

	pose_d_.setZero();                  	      	// desired pose
	dpose_d_.setZero();                          	// desired velocity
	ddpose_d_.setZero();                        	// desired acceleration
	pose_n_.setZero();                  	      	// nominal des pose
	dpose_n_.setZero();                          	// nominal des velocity
	ddpose_n_.setZero();                        	// nominal des acceleration
	wrench_ext.setZero();                           // external wrench EE frame
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

	return true;
}

//------------------------------------------------------------------------------//
//                          	  STARTING										//
//------------------------------------------------------------------------------//

 

void MotionControlDq::starting(const ros::Time& /*time*/) {
  
	franka::RobotState initial_state = state_handle_->getRobotState();
	Map<Matrix<double, 7, 1> > q_initial(initial_state.q.data()); // intiial joint positions  [rad]
	Map<Matrix<double, 7, 1> > dq_initial(initial_state.dq.data()); // initial joint velocities [rad/s]
    Affine3d initial_transform(Matrix4d::Map(initial_state.O_T_EE.data()));


	// Initial position and orientation
	position_d_ = initial_transform.translation();
	orientation_d_ = Quaterniond(initial_transform.linear());
	or_d_ << orientation_d_.w(), orientation_d_.x(),orientation_d_.y(),orientation_d_.z();

	// Load Panda robot
	DQ_SerialManipulator robot = init_dq_robot();

	// set equilibrium point to current state
	DQ pose_d_dq;  
	pose_d_dq = DQ(or_d_) + 0.5*E_*(DQ(position_d_)*DQ(or_d_)); 
	pose_d_dq = pose_d_dq.normalize();
	pose_d_ = vec8(pose_d_dq); 
	pose_n_ = vec8(pose_d_dq); 

	// set nullspace equilibrium configuration to central angles of joints
	Matrix <double, 7, 1> q_min;
	Matrix <double, 7, 1> q_max;
	q_min << -2.8973, -1.7628, -2.8973, -3.0718+0.1745, -2.8973, -0.0175+0.0873, -2.8973;
	q_max <<  2.8973,  1.7628,  2.8973, -0.0698-0.1745,  2.8973,  3.7525-0.0873,  2.8973;
	q_d_nullspace_ << (q_max + q_min)/2;

	// set collision behaviour
	collBehaviourClient.call(collBehaviourSrvMsg);

	// set joint impedance
	jointImpedanceClient.call(jointImpedanceSrvMsg);
 }

// //------------------------------------------------------------------------------//
// //                          	    UPDATE										//
// //------------------------------------------------------------------------------//

void MotionControlDq::update(const ros::Time& /*time*/,
                                          const ros::Duration &period /*period*/) {

	//----------- VARIABLES DECLARATIONS and DEFINITIONS -------------//

DQ_SerialManipulator robot = init_dq_robot();

	// Control Variables
	Vector8d error;  		         // pose error  8x1
	Vector8d derror; 		         // pose vel. error  8x1
	Vector8d error_i;                // integrative term error 8x1
	Vector3d pos_error;              // position error 3x1
	Vector8d xe;                     // pose displacement                  
	Vector8d dxe;                    // derivative of pose displacement
	Vector8d pose;       	         // current pose 8x1
	Vector8d dpose;			         // current velocity 8x1
	Vector3d pos;                    // current computed EE position 3x1
	Vector8d pose_d_conj;   
	Vector8d dpose_d_conj;             
	Vector8d ddpose_d_conj; 
	DQ pos_d_dq; 
	DQ pose_util; 
	                               
	// for debug, clean later
	Vector4d rot_check;            
	DQ rot_check_dq; 

	// Panda utils
    Matrix<double, 8, 7> Jp;         // pose jacobian 8x7
	Matrix<double, 8, 7> Jp_d;       // pose jacobian 8x7
	Matrix<double, 7, 8> Jp_inv;     // pose jacobian pseudo-inverse 7x8
	Matrix<double, 8, 7> Jp_dot;     // pose derivative jacobian 8*7
    
	// Franka
	franka::RobotState robot_state = state_handle_->getRobotState();        // robot state
	std::array<double, 49> mass_array = model_handle_->getMass();			// mass matrix array
	std::array<double, 7> coriolis_array = model_handle_->getCoriolis();	// coriolis vector

	// Eigen conversion
	Map<Matrix<double, 7, 7> > mass(mass_array.data());                      // mass matrix [kg]
	Map<Matrix<double, 7, 1> > coriolis(coriolis_array.data());              // coriolis forces  [Nm]
	Map<Matrix<double, 7, 1> > q(robot_state.q.data());                      // joint positions  [rad]
	Map<Matrix<double, 7, 1> > dq(robot_state.dq.data());                    // joint velocities [rad/s]
	Map<Matrix<double, 7, 1> > tau_J_d(robot_state.tau_J_d.data());          // previous cycle commanded torques [Nm]
	Map<Matrix<double, 6, 1> > wrench_ext(robot_state.O_F_ext_hat_K.data()); // external wrench [N] wrt base frame

	Affine3d transform(Matrix4d::Map(robot_state.O_T_EE.data()));       // ee-base homog. transf. matrix
	Vector3d position(transform.translation());                         // ee-base position [m]
	Quaterniond orientation(transform.linear());                        // ee-base orientation
	
	// get current state
	rot_check << orientation.w(),orientation.x(),orientation.y(),orientation.z();
    rot_check_dq = DQ(rot_check);
	pose_util = rot_check_dq + 0.5*E_*(DQ(position)*rot_check_dq); 
	pose_util = pose_util.normalize(); 

	//current EE Pose
	pose = vec8(pose_util);
	pos = vec3(pose_util.translation()); 
	
// ----------------------------------------------------

// 	 GET JACOBIANS

	Jp << robot.pose_jacobian(q); 
	Jp_dot << robot.pose_jacobian_derivative(q,dq); 
	dpose << Jp*dq; 

// -------------- PUBLISH MATRICES FOR PLANNING -------------//

	robot_state_msg.header.stamp = ros::Time::now();
	std::copy(mass_array.begin(), mass_array.end(), robot_state_msg.mass_matrix.begin());

	pub_robot_state_.publish(robot_state_msg);

	//----------- COMPUTE ERRORS -------------//
	
	xe << pose_d_ - pose;
	dxe << dpose_d_ - dpose;

	// DQ current and desired pose
	DQ pose_d_dq = DQ(pose_d_); 
	pose_d_dq = pose_d_dq.normalize();
	DQ pose_dq = DQ(pose); 
	pose_dq = pose_dq.normalize(); 

	pose_d_conj = C8()*pose_d_; 
	dpose_d_conj = C8()*dpose_d_; 
	ddpose_d_conj =  C8()*ddpose_d_; 

	// Invariant error definition 

	error << haminus8(DQ(pose_d_conj))*xe; 
	derror << haminus8(DQ(dpose_d_conj))*xe + haminus8(DQ(pose_d_conj))*dxe;
	error_i << error + derror*period.toSec();

	// pos_error << pose_d_dq.translation().q.segment(1, 3) - pose_dq.translation().q.segment(1, 3);
	pos_error << vec3(pose_d_dq.translation()) - vec3(pose_dq.translation());



// 	//======================| CONTROL VARIABLES |======================//
    
	Matrix <double, 8, 1> yd;               // desired cl dyanmics
	Matrix <double, 8, 1> ax;               // input task-space
	Matrix <double, 7, 1> y;         	    // controller new input 
	Matrix <double, 7, 1> tau_task;		    // tau for primary task
	Matrix <double, 7, 1> tau_nullspace;	// tau for nullspace task
	Matrix <double, 7, 1> tau_d;			// final desired torque	
	Matrix <double, 7, 7> N;				// null projector

	I8 = MatrixXd::Identity(8, 8);
	I7 = MatrixXd::Identity(7, 7);
    
	// Desired closed-loop dynamics
	yd << KP*I8*error + KD*I8*derror + 0*KI*error_i; 

    // Control input task space
    ax << haminus8(DQ(ddpose_d_conj))*xe + 2*haminus8(DQ(dpose_d_conj))*dxe  + haminus8(DQ(pose_d_conj))*(ddpose_d_ - Jp_dot*dq) + yd; 
	
	Jp_d << haminus8(DQ(pose_d_conj))*Jp; 

	Jp_inv << pinv(Jp_d); 

    // Control input joint space

    y << Jp_inv*ax;

	//---------------- CONTROL COMPUTATION -----------------//
	
	tau_task << coriolis + mass*y;  // +g (already compensated)
	
	//---------------- NULLSPACE CONTROL COMPUTATION -----------------//
    
    N =  I7 -  pinv(Jp)*Jp; // null-space projector

	//Dissipative term on joints + joints limits avoidance 

	tau_nullspace << N *(NULL_STIFF * (q_d_nullspace_ - q)) - N *(0.5 * dq); //
	
// 	//----------------- FINAL CONTROL --------------------------------//
	
	tau_d << tau_task + tau_nullspace; 

	
// 	//=================================| END CONTROL |==================================//
	
	// //----------- TORQUE SATURATION and COMMAND-------------//

	// Saturate torque rate to avoid discontinuities
	tau_d << saturateTorqueRate(tau_d, tau_J_d);

	// Saturate torque to avoid torque limit
	double ith_torque_rate;
	for (int i = 0; i < 7; ++i){
		ith_torque_rate = std::abs(tau_d(i)/tau_limit(i));
		if( ith_torque_rate > 1)
			tau_d = tau_d / ith_torque_rate;
	}


	//set arm command torques
	for (size_t i = 0; i < 7; ++i) {
		joint_handles_[i].setCommand(tau_d(i));
	}

	//======================| PUBISH & SUBSCRIBE |======================//

	//----------- DEBUG MSG -------------//
    
	// ----------- COMMANDED TORQUES and EXTERNAL FORCES -------------//

	for(int i=0; i<7;i++){
		info_debug_msg.tau_d[i] = tau_d(i);
		info_debug_msg.tau_null[i] = tau_nullspace(i);
		info_debug_msg.tau_task[i] = tau_task(i);
	}

	for(int i=0; i<6;i++){
		info_debug_msg.wrench_ext[i] = - wrench_ext(i); 
	}
    

// 	//----------- POSITION ERROR -------------//

	info_debug_msg.header.stamp = ros::Time::now();
	info_debug_msg.pos_curr[0] = pos(0); // position error x-axis
	info_debug_msg.pos_curr[1] = pos(1); // position error y-axis
	info_debug_msg.pos_curr[2] = pos(2); // position error z-axis
	info_debug_msg.pos_error[0] = pos_error(0); // position error x-axis
	info_debug_msg.pos_error[1] = pos_error(1); // position error y-axis
	info_debug_msg.pos_error[2] = pos_error(2); // position error z-axis

 	pub_info_debug.publish(info_debug_msg);

// // ------------Check des subscribed -------//
	

// 	//----------- EE-POSE -------------//

	geometry_msgs::PoseStamped position_endeff;
	position_endeff.pose.position.x = position.x();
	position_endeff.pose.position.y = position.y();
	position_endeff.pose.position.z = position.z();
	position_endeff.pose.orientation.w = orientation.w();
  	position_endeff.pose.orientation.x = orientation.x();
 	position_endeff.pose.orientation.y = orientation.y();
  	position_endeff.pose.orientation.z = orientation.z();


	pub_endeffector_pose_.publish(position_endeff);

}

// //---------------------------------------------------------------//
// //                    	TORQUE SATURATION                    	 //
// //---------------------------------------------------------------//
 Matrix <double, 7, 1> MotionControlDq::saturateTorqueRate(
     	const Matrix <double, 7, 1>& tau_d_calculated,
     	const Matrix <double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
		const Matrix <double, 7, 1> M;
 		Matrix <double, 7, 1> tau_d_saturated;
 		for (size_t i = 0; i < 7; i++) {
 		double difference = tau_d_calculated[i] - tau_J_d[i];
 		tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
 	}
 	return tau_d_saturated;
 }

 
// // //                          CALLBACKS		                     //
// // //---------------------------------------------------------------//

// //----------- DESIRED NOMINAL TRAJECTORY -------------//

void MotionControlDq::desiredProjectTrajectoryCallback(
    	const panda_controllers::DesiredProjectTrajectoryConstPtr& msg) {
  
	pose_n_ << msg->pose_d[0], msg->pose_d[1], msg->pose_d[2],msg->pose_d[3],msg->pose_d[4],msg->pose_d[5],msg->pose_d[6],msg->pose_d[7];		
	dpose_n_ << msg->dpose_d[0], msg->dpose_d[1], msg->dpose_d[2],msg->dpose_d[3],msg->dpose_d[4],msg->dpose_d[5],msg->dpose_d[6],msg->dpose_d[7];	
	ddpose_n_ << msg->ddpose_d[0], msg->ddpose_d[1], msg->ddpose_d[2],msg->ddpose_d[3],msg->ddpose_d[4],msg->ddpose_d[5],msg->ddpose_d[6],msg->ddpose_d[7];	
 } 

// //----------- DESIRED COMPLIANT TRAJECTORY -------------//
void MotionControlDq::CompliantTrajCallback(
    	const panda_controllers::CompliantTraj::ConstPtr& msg) {
  
	pose_d_ << msg->pose_c[0], msg->pose_c[1], msg->pose_c[2],msg->pose_c[3],msg->pose_c[4],msg->pose_c[5],msg->pose_c[6],msg->pose_c[7];		
	dpose_d_ << msg->dpose_c[0], msg->dpose_c[1], msg->dpose_c[2],msg->dpose_c[3],msg->dpose_c[4],msg->dpose_c[5],msg->dpose_c[6],msg->dpose_c[7];	
	ddpose_d_ << msg->ddpose_c[0], msg->ddpose_c[1], msg->ddpose_c[2],msg->ddpose_c[3],msg->ddpose_c[4],msg->ddpose_c[5],msg->ddpose_c[6],msg->ddpose_c[7];	
 } 
 

}  // end namespace franka_softbots


PLUGINLIB_EXPORT_CLASS(panda_controllers::MotionControlDq,
                      controller_interface::ControllerBase);


