#include "panda_controllers/impedance_loop.h"
#include <geometry_msgs/PoseStamped.h>


#define     MASS          1.5                         // [kg]         apparent mass
#define     K_DEFAULT     300                         // [Nm]         default stiffness
#define     D_DEFAULT     8*sqrt(K_DEFAULT*MASS);     // [Ns/m]       default damping
// #define     D_DEFAULT     150;                     // [Ns/m]       default damping
#define     K_ROT         500               
#define     D_ROT         2*sqrt(K_ROT*MASS);      
#define     K_INIT        200                         // [Nm]         default translational stiffness
#define     D_INIT        2*sqrt(K_DEFAULT*MASS);     // [Ns/m]        default translational damping
#define     DZ_VALUE      5                           // dead zone value ext forces (?)       
#define     SAMPLE_TIME   0.005                       // s                 
#define     CUT_OFF_FREQ  1                           // Hz

using namespace DQ_robotics;  
using namespace panda_controllers; 
using DQ_robotics::E_;
using DQ_robotics::i_;
using DQ_robotics::j_;
using DQ_robotics::k_;
using DQ_robotics::C8;

// --------------------------DEFINE FUNCTIONS---------------------------------------------------//


double impedance_loop::lowpassFilter(double sample_time, double y, double y_last, double cutoff_frequency){
  if (sample_time < 0 || !std::isfinite(sample_time)) {
    throw std::invalid_argument("lowpass-filter: sample_time is negative, infinite or NaN.");
  }
  if (cutoff_frequency <= 0 || !std::isfinite(cutoff_frequency)) {
    throw std::invalid_argument(
        "lowpass-filter: cutoff_frequency is zero, negative, infinite or NaN.");
  }
  if (!std::isfinite(y) || !std::isfinite(y_last)) {
    throw std::invalid_argument(
        "lowpass-filter: current or past input value of the signal to be filtered is infinite or "
        "NaN.");
  }
  double gain = sample_time / (sample_time + (1.0 / (2.0 * M_PI * cutoff_frequency)));
  return gain * y + (1 - gain) * y_last;
}


Vector6d impedance_loop::wrench_mapping(Vector6d wrench_ext,DQ_robotics::DQ x_hat){
       Vector6d flog;  
       MatrixXd G = x_hat.generalized_jacobian();
       MatrixXd Q8 = getQ8(x_hat); 
       MatrixXd Ibar = MatrixXd::Zero(6, 8);
       Ibar.block<3,3>(0,1) = MatrixXd::Identity(3,3); 
       Ibar.block<3,3>(3,5) = MatrixXd::Identity(3,3);
       MatrixXd Glog = Ibar * G * Q8;
       flog = Glog.transpose()*wrench_ext;

    return flog;
    }

// DEAD ZONE FOR EXTERNAL FORCES // 

Vector6d impedance_loop::dead_zone(Vector6d wrench_ext, double dz_value){
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
   
void impedance_loop::admittance_eq(Vector6d flog,Vector6d y_hat,Vector6d dy_hat,
        MatrixXd Kd,MatrixXd Bd,MatrixXd Md,double time_prec, double t){
            adm_eq.ddy_hat = Md.inverse()*(-Bd*dy_hat-Kd*y_hat-flog);            
            double cdt; 
            cdt = t-time_prec;
            adm_eq.dy_hat  = adm_eq.ddy_hat*cdt + adm_eq.dy_hat;
            adm_eq.y_hat = adm_eq.dy_hat*cdt + adm_eq.y_hat;
         
        }

void impedance_loop::compute_pose_disp(Vector6d y_hat,Vector6d dy_hat,Vector6d ddy_hat){
        DQ y_hat_dq; 
        DQ x_hat_dq;
        DQ dx_hat_dq; 
        y_hat_dq = DQ(y_hat); 
        disp.x_hat = vec8(exp(y_hat_dq));
        x_hat_dq = DQ(disp.x_hat);
        MatrixXd Q8 = getQ8(x_hat_dq);
        disp.dx_hat = Q8*dy_hat;
        dx_hat_dq = DQ(disp.dx_hat); 
        MatrixXd Q8_dot = getQ8_dot(x_hat_dq,dx_hat_dq);
        disp.ddx_hat = Q8*ddy_hat + Q8_dot*dy_hat;

}

void impedance_loop::compute_traj(Vector8d x_d,Vector8d dx_d,Vector8d ddx_d,
        Vector8d x_hat,Vector8d dx_hat,Vector8d ddx_hat){
        DQ x_hat_dq; 
        DQ dx_hat_dq;
        DQ ddx_hat_dq;
        x_hat_dq = DQ(x_hat);
        dx_hat_dq = DQ(dx_hat);
        ddx_hat_dq = DQ(ddx_hat);
        DQ x_d_dq; 
        DQ dx_d_dq;
        DQ ddx_d_dq;
        x_d_dq = DQ(x_d);
        dx_d_dq = DQ(dx_d);
        ddx_d_dq = DQ(ddx_d);
        comp.x_c = hamiplus8(x_d_dq)*C8()*(x_hat);
        comp.dx_c = hamiplus8(dx_d_dq)*C8()*(x_hat) + hamiplus8(x_d_dq)*C8()*(dx_hat);
        comp.ddx_c = hamiplus8(ddx_d_dq)*C8()*(x_hat) + 2*hamiplus8(dx_d_dq)*C8()*(dx_hat) + hamiplus8(x_d_dq)*C8()*ddx_hat;
        }

//--------------------------------INIT----------------------------//

bool impedance_loop::init(ros::NodeHandle& node_handle){

    int n = 0;
	  name_space = node_handle.getNamespace();
	  n = name_space.find("/", 2);
	  name_space = name_space.substr(0,n);

    sub_des_traj_proj_ = node_handle.subscribe(
      "/motion_control_dq/dq_trajectory", 1, &impedance_loop::desiredProjectTrajectoryCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  
    sub_ext_forces = node_handle.subscribe(
      "/motion_control_dq/info_debug", 1, &impedance_loop::f_ext_Callback, this,
      ros::TransportHints().reliable().tcpNoDelay());

    sub_ee_pose = node_handle.subscribe(
      "/motion_control_dq/franka_ee_pose", 1, &impedance_loop::ee_pose_Callback, this,
      ros::TransportHints().reliable().tcpNoDelay());

    sub_des_imp_proj_ =  node_handle.subscribe(  "/motion_control_dq/desired_impedance", 1, 
													&impedance_loop::desiredImpedanceProjectCallback, this,
													ros::TransportHints().reliable().tcpNoDelay());

    pub_compliant_traj = node_handle.advertise<panda_controllers::CompliantTraj>("/motion_control_dq/compliant_traj", 1);

 
  //---------------INITIALIZE VARIABLES---------------//
    // VARIABLES
    time_prec = ros::Time::now().toSec();     // previous cyle time 
    t = ros::Time::now().toSec();             // current ros time
    wrench_ext.setZero();                     // external wrench on EE
    wrench_n = wrench_ext;  
    fx = 0;
    fy = 0;
    fz = 0; 
    fx_prec = 0;
    fy_prec = 0;
    fz_prec = 0; 
    pose_d_ << 1,0,0,0,0,0,0,0;               // nominal des trajectory                                         
    dpose_d_.setZero();
    ddpose_d_.setZero();
    pos_in_.setZero();                        // initial EE position
    or_in_ << 1,0,0,0;                        // initial EE orientation
    I8 = MatrixXd::Identity(8, 8);
	  I6 = MatrixXd::Identity(6, 6);
    adm_eq.y_hat.setZero();                   // log mapping of displacement
    adm_eq.dy_hat.setZero();
    adm_eq.ddy_hat.setZero();
    disp.x_hat << 1,0,0,0,0,0,0,0;             // pose displacement 8x1
    comp.x_c << 1,0,0,0,0,0,0,0;               // desired compliant pose 8x1 
    // Initialize stiffness and damping matrices
    KD.setIdentity(); 
    BD.setIdentity(); 
    MD.setIdentity();
    KD << I6*K_INIT; 
	  BD << I6*D_INIT; 
    count = 0;
   return true;

}

// ------------------------------UPDATE------------------------//

void impedance_loop::update(){

   // VARIABLES
   DQ x_dq;      //current pose DQ
   DQ x_hat_dq;  // displacement between nominal and computed DQ
   DQ pos_in_dq; // DQ initial EE position
   DQ rot_in_dq; // DQ initial EE orientation
   DQ pose_d_dq; 
   Vector3d position_c;
   Vector8d pose_nom;
   Vector8d dpose_nom;
   double fx_fil,fy_fil,fz_fil; //current external forces
   pose_nom << 1,0,0,0,0,0,0,0;
   dpose_nom << 0,0,0,0,0,0,0; 
  
   //Retrieve current EE orientation
   pos_in_dq = DQ(pos_in_); //position
   rot_in_dq = DQ(or_in_);  //initial rotation

   // DQ nominal desired pose
   pose_d_dq = DQ(pose_d_); //desired pose DQ 
   pose_d_dq = pose_d_dq.normalize(); 

   //Current pose
   x_dq = rot_in_dq + 0.5*E_*(pos_in_dq*rot_in_dq); 
   x_dq = x_dq.normalize();

   //Impedance matrices 
  //  KD(0,0)= K_ROT;
	//  KD(1,1)= K_ROT;
	//  KD(2,2)= K_ROT;
	//  KD(3,3)= K_DEFAULT;
	//  KD(4,4)= K_DEFAULT;
	//  KD(5,5)= K_DEFAULT;
 
	//  BD(0,0)= D_ROT;
	//  BD(1,1)= D_ROT;
	//  BD(2,2)= D_ROT;
	//  BD(3,3)= D_DEFAULT;
	//  BD(4,4)= D_DEFAULT;
	//  BD(5,5)= D_DEFAULT;

   MD =I6*MASS;
  
  //lowpass filter for ext forces
  fx_prec = fx; 
  fy_prec = fy;
  fz_prec = fz; 

  fx = wrench_ext(0);
  fy = wrench_ext(1);
  fz = wrench_ext(2); 

  fx_fil = lowpassFilter(SAMPLE_TIME,fx,fx_prec,CUT_OFF_FREQ); 
  fy_fil = lowpassFilter(SAMPLE_TIME,fy,fy_prec,CUT_OFF_FREQ); 
  fz_fil = lowpassFilter(SAMPLE_TIME,fz,fz_prec,CUT_OFF_FREQ); 

  wrench_n << fx_fil,fy_fil,fz_fil,wrench_ext(3),wrench_ext(4),wrench_ext(5); 

  //Dead zone for estimated external force
  wrench_n = dead_zone(wrench_n,DZ_VALUE);

  // External wrench w.r.t compliant frame

  wrench_n =  vec6(rot_in_dq.conj()*DQ(wrench_n)*rot_in_dq);

  x_hat_dq = DQ(disp.x_hat); 

  // External wrench consistent with log displacement

  Vector6d f_log = wrench_mapping(wrench_n,x_hat_dq);

  t = ros::Time::now().toSec();

   admittance_eq(f_log,adm_eq.y_hat, adm_eq.dy_hat,
         KD, BD, MD, time_prec, t);

  time_prec = t;

  //Compute frames displacement
  compute_pose_disp(adm_eq.y_hat,adm_eq.dy_hat,adm_eq.ddy_hat);

  //Compute compliant trajectory

  compute_traj(pose_d_,dpose_d_,ddpose_d_,disp.x_hat,disp.dx_hat,disp.ddx_hat);

  position_c = vec3(DQ(comp.x_c).translation()); 

  
 
  // //---------------PUBLISHING----------------//

    compliant_traj_msg.header.stamp = ros::Time::now();

    for (int i=0; i<8; i++){
         compliant_traj_msg.pose_c[i] = comp.x_c(i);
         compliant_traj_msg.dpose_c[i] = comp.dx_c(i);
         compliant_traj_msg.ddpose_c[i] = comp.ddx_c(i);
       }
    for (int i=0; i<6; i++){
         compliant_traj_msg.flog[i] = f_log(i);
         compliant_traj_msg.wrench_n[i] = wrench_n(i);
       }   

    compliant_traj_msg.position_c[0] = position_c(0); 
    compliant_traj_msg.position_c[1] = position_c(1); 
    compliant_traj_msg.position_c[2] = position_c(2); 

    compliant_traj_msg.f_filtered[0] = fx_fil;
    compliant_traj_msg.f_filtered[1] = fy_fil;
    compliant_traj_msg.f_filtered[2] = fz_fil;

    if(pose_d_!= pose_nom){
       pub_compliant_traj.publish(compliant_traj_msg);
    }

   count = count++;
}

//---------------------CALLBACKS------------------//

//----------- DESIRED TRAJECTORY -------------//
void impedance_loop::desiredProjectTrajectoryCallback(
    	const panda_controllers::DesiredProjectTrajectoryConstPtr& msg) {
  
	pose_d_ << msg->pose_d[0], msg->pose_d[1], msg->pose_d[2],msg->pose_d[3],msg->pose_d[4],msg->pose_d[5],msg->pose_d[6],msg->pose_d[7];		
  dpose_d_ << msg->dpose_d[0], msg->dpose_d[1], msg->dpose_d[2],msg->dpose_d[3],msg->dpose_d[4],msg->dpose_d[5],msg->dpose_d[6],msg->dpose_d[7];	
	ddpose_d_ << msg->ddpose_d[0], msg->ddpose_d[1], msg->ddpose_d[2],msg->ddpose_d[3],msg->ddpose_d[4],msg->ddpose_d[5],msg->ddpose_d[6],msg->ddpose_d[7];	
 } 

//Callback for robot pose
void impedance_loop::ee_pose_Callback(const geometry_msgs::PoseStampedConstPtr& msg) {
  pos_in_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  or_in_ << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z; 
}

void impedance_loop::f_ext_Callback(const panda_controllers::InfoDebugConstPtr& msg){
  wrench_ext << msg->wrench_ext[0], msg->wrench_ext[1],msg->wrench_ext[2], msg->wrench_ext[3],msg->wrench_ext[4],msg->wrench_ext[5];
}

//----------- DESIRED IMPEDANCE -------------//
void impedance_loop::desiredImpedanceProjectCallback(
     		const panda_controllers::DesiredImpedanceConstPtr& msg){

	for (int i=0; i<6; i++){
		for (int j=0; j<6; j++){
			KD(i, j) = msg->stiffness_matrix[i*6 + j];
			BD(i, j) = msg->damping_matrix[i*6 + j];
		}
	}
}


//--------------------------------------------------------------------------------------------------
// ----------------------------------GET Q, partial derivatives of DQ log-----------------------  //


MatrixXd impedance_loop::getQ4 (DQ_robotics::DQ &rot)
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

 MatrixXd impedance_loop::getQ8 (DQ_robotics::DQ &x)
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

MatrixXd impedance_loop::getQ4_dot(DQ_robotics::DQ &rot,DQ_robotics::DQ &drot)
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
    Q4_dot(3,2)  = gamma*pow(nz,2)+dtheta + 2*gamma*nz*dnz;

return Q4_dot;
}

MatrixXd impedance_loop::getQ8_dot(DQ_robotics::DQ &x,DQ_robotics::DQ &dx)
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



//==========================================================================================//
//                                         MAIN                                             //
//==========================================================================================//
int main(int argc, char **argv){

    using namespace panda_controllers; 

    ros::init(argc, argv, "impedance_loop");

    ros::NodeHandle node_impedance;

    ros::Rate loop_rate(200); // outer loop frequency Hz
    
    impedance_loop impedance;

    impedance.init(node_impedance);

   //  ros::spinOnce();
  
    while (ros::ok()){

        ros::spinOnce();

        impedance.update();

        loop_rate.sleep();
    }
    return 0;
}

