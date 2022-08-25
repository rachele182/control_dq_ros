// ========================== IMPEDANCE CONTROLLER USING DQ LOG ========================= //

//Include:
#include <cmath>
#include <math.h>
#include <memory>
#include <panda_controllers/impedance_control_dq.h>
// DQ robotics
#include "dqrobotics/DQ.h"
#include "dqrobotics/robot_modeling/DQ_SerialManipulator.h"
#include "dqrobotics/utils/DQ_LinearAlgebra.h"
// Franka ros
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <franka_hw/trigger_rate.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

using DQ_robotics::DQ_SerialManipulator;
using DQ_robotics::E_;
using DQ_robotics::i_;
using DQ_robotics::j_;
using DQ_robotics::k_;
using DQ_robotics::C8;

using namespace DQ_robotics;

#define     MASS            1.5                         // [kg]         apparent mass
#define     K_INIT          300                         // [Nm]         initial stiffness
#define     D_INIT          2*sqrt(K_DEFAULT*MASS);     // [Ns/m]       initial damping
#define     K_DEFAULT       300                         // [Nm]         default translational stiffness
#define     D_DEFAULT       2*sqrt(K_DEFAULT*MASS);     // [Ns/m]       default translational damping
#define     K_ROT           500                         // [N*rad]      defaul rotational stiffness
#define     D_ROT           2*sqrt(K_ROT*MASS);         // [Ns/rad]     defaul rotational damping
#define     DZ_VALUE        6    // dead zone values for external forces    
#define 	D_JOINTS	    2    // dissipative term joints
#define 	COLL_LIMIT		25  
#define 	NULL_STIFF		2
#define 	JOINT_STIFF		{3000, 3000, 3000, 3000, 3000, 2000, 100}

namespace panda_controllers {
	
// ---------- LOAD DQ ROBOT ------- //
DQ_SerialManipulator ImpedanceControlDq::init_dq_robot() {
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

// //------------------------------------------------------------------------------// //
// //                          		INIT										   // //
// //------------------------------------------------------------------------------// //

bool ImpedanceControlDq::init( hardware_interface::RobotHW* robot_hw, 
                                       ros::NodeHandle& node_handle) {
	//Name space extraction to add a prefix to the topic name
	int n = 0;
	name_space = node_handle.getNamespace();
	n = name_space.find("/", 2);
	name_space = name_space.substr(0,n);
										

	//--------------- INITIALIZE SUBSCRIBERS AND PUBLISHERS -----------------//

	sub_des_imp_proj_ =   node_handle.subscribe(  "/motion_control_dq/desired_impedance", 1, 
													&ImpedanceControlDq::desiredImpedanceProjectCallback, this,
													ros::TransportHints().reliable().tcpNoDelay());

    sub_des_traj_proj_ =  node_handle.subscribe(  "/motion_control_dq/dq_trajectory", 1, 
													&ImpedanceControlDq::desiredProjectTrajectoryCallback, this,
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
		ROS_ERROR_STREAM("ImpedanceControlDq: Could not read parameter arm_id");
		return false;
	}
	std::vector<std::string> joint_names;
	if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
		ROS_ERROR(
				"ImpedanceControlDq: Invalid or no joint_names parameters provided, "
				"aborting controller init!");
		return false;
	}
	if (!node_handle.getParam("var_damp", var_damp)) {
		ROS_ERROR_STREAM("ImpedanceControlDq: Could not read parameter var_damp");
		return false;
	}

	franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
	if (model_interface == nullptr) {
		ROS_ERROR_STREAM("ImpedanceControlDq: Error getting model interface from hardware");
		return false;
	}
	try {
		model_handle_.reset(
				new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM(
				"ImpedanceControlDq: Exception getting model handle from interface: "
				<< ex.what());
		return false;
	}

	franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
	if (state_interface == nullptr) {
		ROS_ERROR_STREAM("ImpedanceControlDq: Error getting state interface from hardware");
		return false;
	}
	try {
		state_handle_.reset(
				new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM(
				"ImpedanceControlDq: Exception getting state handle from interface: "
				<< ex.what());
		return false;
	}

	hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
	if (effort_joint_interface == nullptr) {
		ROS_ERROR_STREAM("ImpedanceControlDq: Error getting effort joint interface from hardware");
		return false;
	}
	for (size_t i = 0; i < 7; ++i) {
		try {
			joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
		} catch (const hardware_interface::HardwareInterfaceException& ex) {
			ROS_ERROR_STREAM(
					"ImpedanceControlDq: Exception getting joint handles: " << ex.what());
		return false;
		}
	}
    
    
	//---------------- INITIALIZE VARIABLES ------------------//

	I6 = MatrixXd::Identity(6, 6);
	I7 = MatrixXd::Identity(7, 7);
	I8 = MatrixXd::Identity(8, 8);
	Q8 = MatrixXd::Identity(8, 6); 
	Q8_dot = MatrixXd::Zero(8, 6); 
	pose_d_.setZero();                  	      	// desired pose
	dpose_d_.setZero();                          	// desired velocity
	ddpose_d_.setZero();                        	// desired acceleration
	disp.x_hat << 1,0,0,0,0,0,0,0;    				// initial pose displacement 8x1 (DQ(x).conj*DQ(xd))
	disp.dx_hat.setZero(); 
    adm_eq.y_hat.setZero();                         // log mapping of frames displacement
    adm_eq.dy_hat.setZero();             
	time_prec = ros::Time::now().toSec();          // previous cycle time 
    t = ros::Time::now().toSec();                  // current time
	wrench_ext.setZero();                          // external wrench EE frame
	wrench_n.setZero();

    // Initialize stiffness and damping matrices
    K.setIdentity(); 
    D.setIdentity(); 
    M.setIdentity();

	K << I6*K_INIT; 
	D << I6*D_INIT; 

	// Tau limit
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


void ImpedanceControlDq::starting(const ros::Time& /*time*/) {
  
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

	// set equilibrium point of desired variables to current state
	DQ pose_d_dq;  
	pose_d_dq = DQ(or_d_) + 0.5*E_*(DQ(position_d_)*DQ(or_d_)); 
	pose_d_dq = pose_d_dq.normalize();
	pose_d_ = vec8(pose_d_dq);
	pose_ = vec8(pose_d_dq); 
	
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

void ImpedanceControlDq::update(const ros::Time& /*time*/,
                                          const ros::Duration &period /*period*/) {

	//----------- VARIABLES DECLARATIONS and DEFINITIONS -------------//

DQ_SerialManipulator robot = init_dq_robot();

	Vector3d pos_error;               // position error 3x1
	Vector3d pos;                     // current EE position 3x1
	DQ pose_dq;                       // DQ current pose 
	DQ pose_d_dq;                     // DQ desired pose


   //===================== | IMPEDANCE VALUES | =================== //

    M = I6*MASS;
	K(0,0)= K_ROT;
	K(1,1)= K_ROT;
	K(2,2)= K_ROT;
	K(3,3)= K_DEFAULT;
	K(4,4)= K_DEFAULT;
	K(5,5)= K_DEFAULT;

	D(0,0)= D_ROT;
	D(1,1)= D_ROT;
	D(2,2)= D_ROT;
	D(3,3)= D_DEFAULT;
	D(4,4)= D_DEFAULT;
	D(5,5)= D_DEFAULT;

	//======================| CONTROL VARIABLES |======================//
	DQ x_hat_dq;
	DQ dx_hat_dq; 
	Matrix <double, 8, 1> ax;               // controller input task-space
	Matrix <double, 7, 1> ad;         	    // controller new input joint-space
	Matrix <double, 7, 1> tau_task;		    // tau for primary task
	Matrix <double, 7, 1> tau_nullspace;	// tau for nullspace task
	Matrix <double, 7, 1> tau_d;			// final desired torque	
	Matrix <double, 7, 7> N;				// null projector
                      
	// for debug, clean later
	Vector4d rot_check;            
	DQ rot_check_dq; 

	// Panda utils
    Matrix<double, 8, 7> Jp;          // pose jacobian 8x7
	Matrix<double, 8, 7> Jp_d;        // pose jacobian consistent with error definition 8x7
	Matrix<double, 7, 8> Jp_inv;      // pose jacobian pseudo-inverse 7x8
	Matrix<double, 8, 7> Jp_dot;      // pose derivative jacobian 8*7
    
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
	pose_dq = rot_check_dq + 0.5*E_*(DQ(position)*rot_check_dq); 
	pose_dq = pose_dq.normalize(); 

	//current EE Pose and position
	pose_ = vec8(pose_dq);
	pos = vec3(pose_dq.translation()); 

	// Desired pose
	pose_d_dq = DQ(pose_d_); 
	
// ----------------------------------------------------

// 	 GET JACOBIANS

	Jp << robot.pose_jacobian(q); 
	Jp_dot << robot.pose_jacobian_derivative(q,dq); 
	dpose_ << Jp*dq; 

// -------------- PUBLISH MATRICES FOR PLANNING -------------//

	robot_state_msg.header.stamp = ros::Time::now();
	std::copy(mass_array.begin(), mass_array.end(), robot_state_msg.mass_matrix.begin());

	pub_robot_state_.publish(robot_state_msg);
// ---------------------------------------------------------//


//   COMPUTE POSE ERROR

	disp.x_hat = vec8(pose_dq.conj()*pose_d_dq);
    disp.dx_hat = vec8(DQ(dpose_).conj()*pose_d_dq) + vec8(pose_dq.conj()*DQ(dpose_d_)); 
	x_hat_dq = DQ(disp.x_hat); 
	dx_hat_dq = DQ(disp.dx_hat);
    adm_eq.y_hat = vec6(log(x_hat_dq));
    Q8 = getQ8(x_hat_dq);
	Q8_dot = getQ8_dot(x_hat_dq,dx_hat_dq);
    adm_eq.dy_hat = pinv(Q8)*disp.dx_hat; 
	
	// External wrench world-frame acting on end-effector
	wrench_ext = -1*wrench_ext; 

	//Read external wrench
	wrench_n = dead_zone(wrench_ext,DZ_VALUE); 

	// External wrench wrt to EE frame
	wrench_n =  vec6(rot_check_dq.conj()*DQ(wrench_n)*rot_check_dq);

	// External wrench consistent with log displacement

    Vector6d f_log = wrench_mapping(wrench_n,x_hat_dq);

	t = ros::Time::now().toSec();

	// Enforce Impedance

	admittance_eq(f_log,adm_eq.y_hat,adm_eq.dy_hat,
         K, D, M);

	time_prec = t; 

	//Compute des cl dynamics

	disp.ddx_hat = Q8*adm_eq.ddy_hat + Q8_dot*adm_eq.dy_hat;
	
	// Store position error

	pos_error << vec3(pose_d_dq.translation()) - pos;

	// Task space control input
    ax = hamiplus8(DQ(ddpose_d_))*C8()*disp.x_hat+ 2*hamiplus8(DQ(dpose_d_))*C8()*disp.dx_hat + hamiplus8(DQ(pose_d_))*C8()*disp.ddx_hat - Jp_dot*dq; 
	Jp_inv = pinv(Jp);
    
    // control input joint space
    ad = Jp_inv*ax; 

	//---------------- CONTROL COMPUTATION (FB LIN)-----------------//
	
	tau_task << coriolis + mass*ad;  // +g (already compensated)
	std::cout << tau_task << std::endl;

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
		info_debug_msg.tau_null[i] = tau_nullspace(i);
		info_debug_msg.tau_task[i] = tau_task(i);
	}

	for(int i=0; i<6;i++){
		info_debug_msg.wrench_ext[i] = - wrench_ext(i); 
	}
    

	//----------- POSITION ERROR -------------//

	info_debug_msg.header.stamp = ros::Time::now();
	info_debug_msg.pos_curr[0] = pos(0); // position error x-axis
	info_debug_msg.pos_curr[1] = pos(1); // position error y-axis
	info_debug_msg.pos_curr[2] = pos(2); // position error z-axis

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

// //-----------------------WRENCH LOG MAPPING-----------------------//

Vector6d ImpedanceControlDq::wrench_mapping(Vector6d wrench_ext,DQ_robotics::DQ x_hat){
       Vector6d flog;  
       MatrixXd G = x_hat.generalized_jacobian();
       MatrixXd Q8 = getQ8(x_hat); 
       MatrixXd Ibar = MatrixXd::Zero(6, 8);
       Ibar.block<3,3>(0,1) =MatrixXd::Identity(3,3); 
       Ibar.block<3,3>(3,5) =MatrixXd::Identity(3,3);
       MatrixXd Glog = Ibar * G * Q8;
       flog = Glog.transpose()*wrench_ext;

    return flog;
    }


// -------------------------DEAD ZONE ESTIMATED FORCES --------------//

Vector6d ImpedanceControlDq::dead_zone(Vector6d wrench_ext, double dz_value){
   Vector6d wrench_n; 

   for (int i = 0; i < 6; i++) {
    if (abs(wrench_ext[i]) < dz_value) {
      wrench_n[i] = 0;
    } else if (wrench_ext[i] <= -dz_value) {
      wrench_n[i] = wrench_ext[i] + dz_value;

    } else if(wrench_ext[i] >= dz_value){
      wrench_n[i] = wrench_ext[i] - dz_value;
    }
  }
return wrench_n;
}

//===============================DEFINE FUNCTIONS ====================//

void ImpedanceControlDq::admittance_eq(Vector6d flog,Vector6d y_hat,Vector6d dy_hat,
        MatrixXd Kd,MatrixXd Bd,MatrixXd Md){
            adm_eq.ddy_hat = Md.inverse()*(-Bd*dy_hat-Kd*y_hat-flog);            
        }

// //---------------------------------------------------------------//
// //                    	TORQUE SATURATION                    	 //
// //---------------------------------------------------------------//
 Matrix <double, 7, 1> ImpedanceControlDq::saturateTorqueRate(
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


// ----------------------------------GET Q, partial derivatives of DQ log-----------------------  //


MatrixXd ImpedanceControlDq::getQ4 (DQ_robotics::DQ &rot)
{
    double tol;
    tol = 1e-4;
    VectorXd r = rot.vec4();
    double phi = rot.rotation_angle();
    double theta;
    double gamma;
    VectorXd n = rot.rotation_axis().vec3();
    float nx = n[0];
    float ny = n[1];
    float nz = n[2];

    if (phi < tol){
       phi = 0;
    }
   
    if (phi == 0)
    {
        theta = 1;
    }
     else{
       theta = sin(phi/2)/(phi/2);
    }
 
    gamma = r[0] - theta; 

    Matrix<double, 4, 3> Q4;

    Q4(0,0)  = -r[1];
    Q4(0,1)  = -r[2];
    Q4(0,2)  = -r[3];
    Q4(1,0)  = gamma*pow(nx,2)+theta;
    Q4(1,1)  = gamma*nx*ny;
    Q4(1,2)  = gamma*nx*nz;
    Q4(2,0)  = gamma*nx*ny;
    Q4(2,1)  = gamma*pow(ny,2)+theta;
    Q4(2,2)  = gamma*ny*nz;
    Q4(3,0)  = gamma*nz*nx;
    Q4(3,1)  = gamma*nz*ny;
    Q4(3,2)  = gamma*pow(nz,2)+theta;

return Q4;
}

// Q8(x): Returns the partial derivative of the unit quaternion x=r+0.5*DQ.E()*p*r with respect to log(x).

 MatrixXd ImpedanceControlDq::getQ8 (DQ_robotics::DQ &x)
{
    DQ_robotics::DQ r = x.rotation();
    DQ_robotics::DQ p = x.translation();
    //get Q4
    MatrixXd Q = getQ4(r);
    MatrixXd Qp(4,3);
    Qp.topRows<1>() <<  MatrixXd::Zero(1, 3);
    Qp.bottomRows<3>() <<  MatrixXd::Identity(3, 3);

    MatrixXd Q8(8,6);
    Q8.topRows(4) << Q, MatrixXd::Zero(4, 3);
    Q8.bottomRows(4) << 0.5*p.hamiplus4()*Q, r.haminus4()*Qp;

return Q8;
}

// Derivatives of Q4 and Q8 

MatrixXd ImpedanceControlDq::getQ4_dot(DQ_robotics::DQ &rot,DQ_robotics::DQ &drot)
{
    VectorXd r = rot.vec4();
    VectorXd dr = drot.vec4();
    double phi = double(rot.rotation_angle()); 
    VectorXd n = rot.rotation_axis().vec3();
    double dphi;
    VectorXd dn;
    if (phi == 0) {
       dphi = 0; 
       dn = MatrixXd::Zero(3, 1);
    }
    else  {
        DQ_robotics::DQ tmp;
        tmp = (2*drot.Re())*(1/sin(0.5*phi));
        dphi = -double(tmp);
        VectorXd tmp1;
        tmp1 = drot.Im().vec3();
        dn = (tmp1 - cos(0.5*phi)*dphi*n)/sin(0.5*phi);
    }

    double theta;
    double dtheta;
    double gamma;
    double dgamma;
    
    float nx = n[0];float ny = n[1];float nz = n[2];
    float dnx = dn[0];float dny = dn[1];float dnz = dn[2];
    
    if (phi == 0)
    {
        theta = 1;
        dtheta = 0;
    }
    else
    {
        theta = sin(phi/2)/(phi/2);
        dtheta = (cos(phi/2)*dphi)/phi - (2*sin(phi/2)*dphi)/pow(phi,2);
    }
 
    gamma = r[0] - theta; 
    dgamma = dr[0] - dtheta; 

    MatrixXd Q4_dot(4,3);
    Q4_dot(0,0)  = -dr[1];
    Q4_dot(0,1)  = -dr[2];
    Q4_dot(0,2)  = -dr[3];
    Q4_dot(1,0)  = dgamma*pow(nx,2)+dtheta + 2*gamma*nx*dnx;
    Q4_dot(1,1)  = gamma*nx*dny + gamma*ny*dnx + nx*ny*dgamma;
    Q4_dot(1,2)  = gamma*nx*dnz + gamma*nz*dnx + nx*nz*dgamma;
    Q4_dot(2,0)  = gamma*nx*dny + gamma*ny*dnx + nx*ny*dgamma;
    Q4_dot(2,1)  = dgamma*pow(ny,2)+dtheta + 2*gamma*ny*dny;;
    Q4_dot(2,2)  = gamma*ny*dnz + gamma*nz*dny + ny*nz*dgamma;
    Q4_dot(3,0)  = gamma*dnz*nx + gamma*nz*dnx + nx*nz*dgamma;
    Q4_dot(3,1)  = gamma*nx*dnz + gamma*nz*dny + ny*nz*dgamma;
    Q4_dot(3,2)  = gamma*pow(nz,2)+ dtheta + 2*gamma*nz*dnz;

return Q4_dot;
}

MatrixXd ImpedanceControlDq::getQ8_dot(DQ_robotics::DQ &x,DQ_robotics::DQ &dx)
{
    DQ_robotics::DQ r = x.P(); 
    DQ_robotics::DQ dr = dx.P();
    VectorXd dx_d = vec4(dx.D()); //mapping dx dual part to vec4
    VectorXd dx_p_conj = vec4(dx.P().conj()); //mapping dx primary part to vec4
    DQ_robotics::DQ p = 2*x.D()*(x.P()).conj();
    VectorXd dp_vec = 2*x.conj().P().haminus4()*dx_d + 2*x.D().hamiplus4()*dx_p_conj;
    DQ_robotics::DQ dp = DQ_robotics::DQ(dp_vec); 
    //get Q4 & Q4_dot
    MatrixXd Q = getQ4(r);
    MatrixXd Q_dot = getQ4_dot(r,dr);
   
    MatrixXd Qp(4,3);
    Qp.topRows<1>() <<  MatrixXd::Zero(1, 3);
    Qp.bottomRows<3>() <<  MatrixXd::Identity(3, 3);

    MatrixXd Q8_dot(8,6);
    Q8_dot.setZero();
    Q8_dot.topRows(4) << Q_dot;
    Q8_dot.bottomRows(4) << 0.5*dp.hamiplus4()*Q, dr.haminus4()*Qp;

return Q8_dot;

}

// // //---------------------------------------------------------------//
// // //                          CALLBACKS		                     //
// // //---------------------------------------------------------------//

//----------- DESIRED IMPEDANCE -------------//
void ImpedanceControlDq::desiredImpedanceProjectCallback(
  		const panda_controllers::DesiredImpedanceConstPtr& msg){

	for (int i=0; i<6; i++){
		for (int j=0; j<6; j++){
			K(i, j) = msg->stiffness_matrix[i*6 + j];
			D(i, j) = msg->damping_matrix[i*6 + j];
		}
	}
}

// //----------- DESIRED NOMINAL TRAJECTORY -------------//

void ImpedanceControlDq::desiredProjectTrajectoryCallback(
    	const panda_controllers::DesiredProjectTrajectoryConstPtr& msg) {
  
	pose_d_ << msg->pose_d[0], msg->pose_d[1], msg->pose_d[2],msg->pose_d[3],msg->pose_d[4],msg->pose_d[5],msg->pose_d[6],msg->pose_d[7];		
	dpose_d_ << msg->dpose_d[0], msg->dpose_d[1], msg->dpose_d[2],msg->dpose_d[3],msg->dpose_d[4],msg->dpose_d[5],msg->dpose_d[6],msg->dpose_d[7];	
	ddpose_d_ << msg->ddpose_d[0], msg->ddpose_d[1], msg->ddpose_d[2],msg->ddpose_d[3],msg->ddpose_d[4],msg->ddpose_d[5],msg->ddpose_d[6],msg->ddpose_d[7];	
 } 

}  // end namespace franka_softbots


PLUGINLIB_EXPORT_CLASS(panda_controllers::ImpedanceControlDq,
                      controller_interface::ControllerBase);

