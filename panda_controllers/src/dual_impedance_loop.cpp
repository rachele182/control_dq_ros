#include "panda_controllers/dual_impedance_loop.h"
#include <geometry_msgs/PoseStamped.h>


#define     MASS          1.5                         // [kg]         apparent mass
#define     Kr_DEFAULT    500                         // [Nm]         default relative stiffness
#define     Dr_DEFAULT    2*sqrt(Kr_DEFAULT*MASS);    // [Ns/m]      default relative damping
#define     Ka_DEFAULT    200                         // [Nm]         default absolutestiffness
#define     Da_DEFAULT    2*sqrt(Ka_DEFAULT*MASS);    // [Ns/m]      default absolute damping          
#define     K_ROT         500               
#define     D_ROT         2*sqrt(K_ROT*MASS);      
#define     DZ_VALUE      5                           // dead zone value ext forces (?)       

using namespace DQ_robotics;   using namespace panda_controllers; 
using DQ_robotics::E_;
using DQ_robotics::C8;

// =========================DEFINE FUNCTIONS=====================//

// ----DEAD ZONE FOR EXTERNAL FORCES-- // 

Vector6d dual_impedance_loop::dead_zone(Vector6d wrench_ext, double dz_value){
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

Vector6d dual_impedance_loop::wrench_mapping(Vector6d wrench_ext,DQ_robotics::DQ x_hat){
       // compute external wrench consistent with DQ log mapping
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

void dual_impedance_loop::wrench_adaptor(Vector6d wrench_1,Vector6d wrench_2,DQ_robotics::DQ x1, DQ_robotics::DQ x2,DQ_robotics::DQ xa){
    //linear mapping from the external wrenches measured at the EEs to the abs and rel task space
    DQ rot_dq; Vector3d p1,p2,pa,p1_a,p2_a; Vector4d r1; Vector3d f1,f2,fa,Ma,fr,Mr; Vector6d wr1_1,wr2_1,wa,wr;
    p1 = vec3(x1.translation()); p2 = vec3(x2.translation()); pa = vec3(xa.translation());
    r1 = vec4(x1.rotation()); rot_dq = DQ(r1); //rotation of left arm
    //virtual stick displacement
    f1 = wrench_1.head(3); f2 = wrench_2.head(3); //measured forces
    p1_a << pa - p1; p2_a << pa - p2; 
    //ext wrenches wrt to left arm frame (arbitrary) to compute relative force and toque
    wr1_1 = vec6(rot_dq.conj()*DQ(wrench_1)*rot_dq); 
    wr2_1 = vec6(rot_dq.conj()*DQ(wrench_2)*rot_dq); 
    //absolute and realtive mapped wrenches computation
    fa = wrench_1.head(3) + wrench_2.head(3); // abs force
    fr = 0.5*(wr2_1.head(3) - wr1_1.head(3)); // rel force
    //Ma = wrench_1.tail(3) + (p1_a).cross(f1) + wrench_2.tail(3) + (p2_a).cross(f2); // abs torque
    Mr = 0.5*(wr2_1.tail(3)-wr1_1.tail(3)); // rel torque
    // wa.head(3) << fa; wa.tail(3) << Ma; 
    wa << fa(0),fa(1),fa(2),0,0,0; //torques a 0 to begin 
    wr.head(3) << fr; wr.tail(3) << Mr; 
    coop_wrench.wa = wa; coop_wrench.wr = wr; 
}
   
void dual_impedance_loop::admittance_eq(Vector6d flog_r,Vector6d flog_abs,Vector6d yr_hat,Vector6d dyr_hat,
                                            Vector6d ya_hat,Vector6d dya_hat,MatrixXd Kd_r,MatrixXd Bd_r,MatrixXd Md_r,
                                            MatrixXd Kd_a,MatrixXd Bd_a,MatrixXd Md_a,double time_prec, const double t){
            //Admittance equation for relative and absolute pose
            adm_eq.ddyr_hat = Md_r.inverse()*(-Bd_r*dyr_hat-Kd_r*yr_hat-flog_r); 
            adm_eq.ddya_hat = Md_a.inverse()*(-Bd_a*dya_hat-Kd_a*ya_hat-flog_abs);           
            double cdt; 
            cdt = t-time_prec;
            adm_eq.dyr_hat  = adm_eq.ddyr_hat*cdt + adm_eq.dyr_hat;
            adm_eq.dya_hat  = adm_eq.ddya_hat*cdt + adm_eq.dya_hat;
            adm_eq.yr_hat = adm_eq.dyr_hat*cdt + adm_eq.yr_hat;
            adm_eq.ya_hat = adm_eq.dya_hat*cdt + adm_eq.ya_hat;
        }

void dual_impedance_loop::compute_pose_disp(Vector6d yr_hat,Vector6d dyr_hat,Vector6d ddyr_hat,
                                                Vector6d ya_hat,Vector6d dya_hat,Vector6d ddya_hat){
            //Compute pose displacement from adm equation for relative and absolute pose
            DQ yr_hat_dq,xr_hat_dq,dxr_hat_dq;
            DQ ya_hat_dq,xa_hat_dq,dxa_hat_dq;
            yr_hat_dq = DQ(yr_hat); ya_hat_dq = DQ(ya_hat);  
            //---
            disp.xr_hat = vec8(exp(yr_hat_dq)); 
            disp.xa_hat = vec8(exp(ya_hat_dq));
            xr_hat_dq = DQ(disp.xr_hat); xa_hat_dq = DQ(disp.xa_hat);
            MatrixXd Q8_r = getQ8(xr_hat_dq); MatrixXd Q8_a = getQ8(xa_hat_dq);
            //--
            disp.dxr_hat = Q8_r*dyr_hat;
            disp.dxa_hat = Q8_a*dya_hat; 
            dxr_hat_dq = DQ(disp.dxr_hat); dxa_hat_dq = DQ(disp.dxa_hat); 
            //--
            MatrixXd Q8_r_dot = getQ8_dot(xr_hat_dq,dxr_hat_dq); MatrixXd Q8_a_dot = getQ8_dot(xa_hat_dq,dxa_hat_dq);
            disp.ddxr_hat = Q8_r*ddyr_hat + Q8_r_dot*dyr_hat;
            disp.ddxa_hat = Q8_a*ddya_hat + Q8_a_dot*dya_hat;
}

void dual_impedance_loop::compute_traj(Vector8d xr_d,Vector8d dxr_d,Vector8d ddxr_d,Vector8d xa_d,Vector8d dxa_d,Vector8d ddxa_d,
                            Vector8d xr_hat,Vector8d dxr_hat,Vector8d ddxr_hat,Vector8d xa_hat,Vector8d dxa_hat,Vector8d ddxa_hat){
        //Compute compliant trajectories for relative and absolute pose
        DQ xr_hat_dq, dxr_hat_dq, ddxr_hat_dq,xa_hat_dq, dxa_hat_dq, ddxa_hat_dq;
        xr_hat_dq = DQ(xr_hat); dxr_hat_dq = DQ(dxr_hat); ddxr_hat_dq = DQ(ddxr_hat);
        xa_hat_dq = DQ(xa_hat); dxa_hat_dq = DQ(dxa_hat); ddxa_hat_dq = DQ(ddxa_hat);
        DQ xr_d_dq, dxr_d_dq, ddxr_d_dq,xa_d_dq, dxa_d_dq, ddxa_d_dq;
        xr_d_dq = DQ(xr_d); dxr_d_dq = DQ(dxr_d); ddxr_d_dq = DQ(ddxr_d);
        xa_d_dq = DQ(xa_d); dxa_d_dq = DQ(dxa_d); ddxa_d_dq = DQ(ddxa_d);
        comp.xr_c = hamiplus8(xr_d_dq)*C8()*(xr_hat);
        comp.dxr_c = hamiplus8(dxr_d_dq)*C8()*(xr_hat) + hamiplus8(xr_d_dq)*C8()*(dxr_hat);
        comp.ddxr_c = hamiplus8(ddxr_d_dq)*C8()*(xr_hat) + 2*hamiplus8(dxr_d_dq)*C8()*(dxr_hat) + hamiplus8(xr_d_dq)*C8()*ddxr_hat;
        comp.xa_c = hamiplus8(xa_d_dq)*C8()*(xa_hat);
        comp.dxa_c = hamiplus8(dxa_d_dq)*C8()*(xa_hat) + hamiplus8(xa_d_dq)*C8()*(dxa_hat);
        comp.ddxa_c = hamiplus8(ddxa_d_dq)*C8()*(xa_hat) + 2*hamiplus8(dxa_d_dq)*C8()*(dxa_hat) + hamiplus8(xa_d_dq)*C8()*ddxa_hat;
        }

//--------------------------------INIT----------------------------//

bool dual_impedance_loop::init(ros::NodeHandle& node_handle){

    int n = 0;
	  name_space = node_handle.getNamespace();
	  n = name_space.find("/", 2);
	  name_space = name_space.substr(0,n);

    sub_des_traj_proj_ = node_handle.subscribe(
      "/motion_control_dq/dq_trajectory", 1, &dual_impedance_loop::desiredProjectTrajectoryCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  
    sub_cdts_var = node_handle.subscribe(
      "/motion_control_dq/info_debug", 1, &dual_impedance_loop::cdts_var_Callback, this,
      ros::TransportHints().reliable().tcpNoDelay());


//     sub_des_imp_proj_ =  node_handle.subscribe(  "/motion_control_dq/desired_impedance", 1, 
// 													&dual_impedance_loop::desiredImpedanceProjectCallback, this,
// 													ros::TransportHints().reliable().tcpNoDelay());

    pub_compliant_traj = node_handle.advertise<panda_controllers::CompliantTraj>("/motion_control_dq/compliant_traj", 1);

 
  //---------------INITIALIZE VARIABLES---------------//
    // VARIABLES
    time_prec = ros::Time::now().toSec();     // previous cyle time 
    t = ros::Time::now().toSec();             // current ros time
    wrench_ext_l_.setZero();                  // external wrench on left EE
    wrench_ext_r_.setZero();                  // external wrench on right EE
    pose_d_ << 1,0,0,0,0,0,0,0;               // nominal absolute des trajectory                                         
    dpose_d_.setZero(); 
    ddpose_d_.setZero();
    pose_r_d_ << 1,0,0,0,0,0,0,0;             // nominal relative des trajectory                                         
    dpose_r_d_.setZero();
    ddpose_r_d_.setZero();
    xr_ << 1,0,0,0,0,0,0,0; xa_<< 1,0,0,0,0,0,0,0; x1_<< 1,0,0,0,0,0,0,0; x2_<< 1,0,0,0,0,0,0,0;  
    I8 = MatrixXd::Identity(8, 8); I6 = MatrixXd::Identity(6, 6);
    //Initialize log displcaments
    adm_eq.yr_hat.setZero(), adm_eq.dyr_hat.setZero(), adm_eq.ddyr_hat.setZero();  // log mapping of rel displacement
    adm_eq.ya_hat.setZero(), adm_eq.dya_hat.setZero(), adm_eq.ddya_hat.setZero();  // log mapping of abs displacement
    disp.xr_hat << 1,0,0,0,0,0,0,0;             // pose relative displacement 8x1
    comp.xr_c << 1,0,0,0,0,0,0,0;               // desired relative compliant pose 8x1 
    disp.xa_hat << 1,0,0,0,0,0,0,0;             // pose absolute displacement 8x1
    comp.xa_c << 1,0,0,0,0,0,0,0;               // desired absolute compliant pose 8x1 
    // Initialize stiffness and damping matrices
    KD_r.setIdentity(); BD_r.setIdentity();
    KD_a.setIdentity(); BD_a.setIdentity(); MD.setIdentity();
    KD_r << I6*Kr_DEFAULT; BD_r << I6*Dr_DEFAULT; 
    KD_a << I6*Ka_DEFAULT; BD_r << I6*Da_DEFAULT; 
    count = 0;
   return true;

}

// ------------------------------UPDATE------------------------//

void dual_impedance_loop::update(){

// =================VARIABLES===============//
   Vector8d pose_nom,dpose_nom; 
   pose_nom << 1,0,0,0,0,0,0,0;
   dpose_nom.setZero(); 
   DQ xa_hat_dq;   // displacement between nominal and computed absolute pose DQ
   DQ xr_hat_dq;  //  displacement between nominal and computed relative pose DQ
   DQ pose_r_d_dq, pose_a_d_dq; //Dq nominal trajectories
   DQ x1_dq,x2_dq,xa_dq,xr_dq,rot_r_dq,rot_a_dq;
   Vector3d pos_abs_c,pos_rel_c; //computed abs and rel positions
   Vector4d rot_r; //orientation of relative frame
   Vector6d wrench_rel,wrench_abs, f_log_a, f_log_r; 
   
   // DQ nominal desired poses
   pose_a_d_dq = DQ(pose_d_); pose_a_d_dq = pose_a_d_dq.normalize(); 
   pose_r_d_dq = DQ(pose_r_d_); pose_r_d_dq = pose_r_d_dq.normalize(); 
   
   //Relative impedance
   KD_r(0,0)= K_ROT; KD_r(1,1)= K_ROT; KD_r(2,2)= K_ROT; //rotational
   KD_r(3,3)= Kr_DEFAULT; KD_r(4,4)= Kr_DEFAULT; KD_r(5,5)= Kr_DEFAULT; //translational
   BD_r(0,0)= D_ROT;  BD_r(1,1)= D_ROT; BD_r(2,2)= D_ROT;
   BD_r(3,3)= Dr_DEFAULT; BD_r(4,4)= Dr_DEFAULT; BD_r(5,5)= Dr_DEFAULT;
   //Absolute impedance
   KD_r(0,0)= K_ROT; KD_r(1,1)= K_ROT; KD_r(2,2)= K_ROT;
   KD_r(3,3)= Kr_DEFAULT; KD_r(4,4)= Kr_DEFAULT; KD_r(5,5)= Kr_DEFAULT;
   BD_a(0,0)= D_ROT;  BD_a(1,1)= D_ROT; BD_a(2,2)= D_ROT;
   BD_a(3,3)= Dr_DEFAULT; BD_a(4,4)= Dr_DEFAULT; BD_a(5,5)= Dr_DEFAULT;

   MD =I6*MASS;

   //Dead zone for estimated external forces acting on EEs
   wrench_ext_l_ = dead_zone(wrench_ext_l_,DZ_VALUE); 
   wrench_ext_r_ = dead_zone(wrench_ext_r_,DZ_VALUE);

   //Current poses
   x1_dq = DQ(x1_); x2_dq = DQ(x2_); xr_dq = DQ(xr_); xa_dq = DQ(xa_);

   //Wrench linear mapping to CDTS space 
   wrench_adaptor(wrench_ext_l_,wrench_ext_r_,x1_dq,x2_dq,xa_dq); 

   //External wrench w.r.t compliant frames
   xr_dq = xr_dq.normalize(); 
   xa_dq = xa_dq.normalize();
   rot_r = vec4(xr_dq.rotation()); rot_r_dq = DQ(rot_r); rot_a_dq = xa_dq.rotation();
   wrench_rel << coop_wrench.wr; wrench_abs << coop_wrench.wa;  
   wrench_rel = vec6(rot_r_dq.conj()*DQ(wrench_rel)*rot_r_dq); 
   wrench_abs = vec6(rot_a_dq.conj()*DQ(wrench_abs)*rot_a_dq);
    
    xr_hat_dq = DQ(disp.xr_hat); xa_hat_dq = DQ(disp.xa_hat); 

   // External wrench consistent with log displacement

    f_log_a = wrench_mapping(wrench_abs,xa_hat_dq);
    f_log_r = wrench_mapping(wrench_rel,xr_hat_dq);

    t = ros::Time::now().toSec();

    admittance_eq(f_log_r, f_log_a, adm_eq.yr_hat, adm_eq.dyr_hat,adm_eq.ya_hat, adm_eq.dya_hat,
                                             KD_r, BD_r, MD,KD_a, BD_a, MD, time_prec, t); 

    time_prec = t;

    //Compute frames displacement
    compute_pose_disp(adm_eq.yr_hat,adm_eq.dyr_hat,adm_eq.ddyr_hat,adm_eq.ya_hat,adm_eq.dya_hat,adm_eq.ddya_hat);

    //Compute compliant trajectory

     compute_traj(pose_r_d_,dpose_r_d_,ddpose_r_d_,pose_d_,dpose_d_,ddpose_d_,
                    disp.xr_hat,disp.dxr_hat,disp.ddxr_hat,
                    disp.xa_hat,disp.dxa_hat,disp.ddxa_hat);
    
     pos_abs_c = vec3((DQ(comp.xa_c)).translation()); 
     pos_rel_c = vec3((DQ(comp.xr_c)).translation()); 

  
//   // //---------------PUBLISHING----------------//

    compliant_traj_msg.header.stamp = ros::Time::now();

    for (int i=0; i<8; i++){
         compliant_traj_msg.pose_rel_c[i] = comp.xr_c(i);
         compliant_traj_msg.dpose_rel_c[i] = comp.dxr_c(i);
         compliant_traj_msg.ddpose_rel_c[i] = comp.ddxr_c(i);
         compliant_traj_msg.pose_abs_c[i] = comp.xa_c(i);
         compliant_traj_msg.dpose_abs_c[i] = comp.dxa_c(i);
         compliant_traj_msg.ddpose_abs_c[i] = comp.ddxa_c(i);
       }
    for (int i=0; i<3; i++){
         compliant_traj_msg.pos_rel_c[i] = pos_rel_c(i); 
         compliant_traj_msg.pos_abs_c[i] = pos_abs_c(i); 
         compliant_traj_msg.fa[i] = ((coop_wrench.wa).head(3))(i); 
         compliant_traj_msg.fr[i] = ((coop_wrench.wr).head(3))(i); 
       }   

    if(pose_d_!= pose_nom && pose_r_d_!= pose_nom){
       pub_compliant_traj.publish(compliant_traj_msg);
    }

   count = count++;
}

//---------------------CALLBACKS------------------//

// //----------- DESIRED TRAJECTORY -------------//
void dual_impedance_loop::desiredProjectTrajectoryCallback(
    	const panda_controllers::DesiredProjectTrajectoryConstPtr& msg) {
	pose_d_ << msg->pose_d[0], msg->pose_d[1], msg->pose_d[2],msg->pose_d[3],msg->pose_d[4],msg->pose_d[5],msg->pose_d[6],msg->pose_d[7];		
    dpose_d_ << msg->dpose_d[0], msg->dpose_d[1], msg->dpose_d[2],msg->dpose_d[3],msg->dpose_d[4],msg->dpose_d[5],msg->dpose_d[6],msg->dpose_d[7];	
	ddpose_d_ << msg->ddpose_d[0], msg->ddpose_d[1], msg->ddpose_d[2],msg->ddpose_d[3],msg->ddpose_d[4],msg->ddpose_d[5],msg->ddpose_d[6],msg->ddpose_d[7];	
    pose_r_d_ << msg->pose_r[0], msg->pose_r[1], msg->pose_r[2],msg->pose_r[3],msg->pose_r[4],msg->pose_r[5],msg->pose_r[6],msg->pose_r[7];		
    dpose_r_d_ << msg->dpose_r[0], msg->dpose_r[1], msg->dpose_r[2],msg->dpose_r[3],msg->dpose_r[4],msg->dpose_r[5],msg->dpose_r[6],msg->dpose_r[7];	
	ddpose_r_d_ << msg->ddpose_r[0], msg->ddpose_r[1], msg->ddpose_r[2],msg->ddpose_r[3],msg->ddpose_r[4],msg->ddpose_r[5],msg->ddpose_r[6],msg->ddpose_r[7];	
 } 

void dual_impedance_loop::cdts_var_Callback(const panda_controllers::InfoDebugConstPtr& msg){
    x1_ << msg->x1[0], msg->x1[1], msg->x1[2],msg->x1[3],msg->x1[4],msg->x1[5],msg->x1[6],msg->x1[7];	
    x2_ << msg->x2[0], msg->x2[1], msg->x2[2],msg->x2[3],msg->x2[4],msg->x2[5],msg->x2[6],msg->x2[7];	
    xa_ << msg->abs_pose[0], msg->abs_pose[1], msg->abs_pose[2],msg->abs_pose[3],msg->abs_pose[4],msg->abs_pose[5],msg->abs_pose[6],msg->abs_pose[7];	
    wrench_ext_l_ << msg->wrench_ext_1[0], msg->wrench_ext_1[1],msg->wrench_ext_1[2], msg->wrench_ext_1[3],msg->wrench_ext_1[4],msg->wrench_ext_1[5];
    wrench_ext_r_ << msg->wrench_ext_2[0], msg->wrench_ext_2[1],msg->wrench_ext_2[2], msg->wrench_ext_2[3],msg->wrench_ext_2[4],msg->wrench_ext_2[5];
}

// //----------- DESIRED IMPEDANCE -------------//
// void dual_impedance_loop::desiredImpedanceProjectCallback(
//      		const panda_controllers::DesiredImpedanceConstPtr& msg){

// 	for (int i=0; i<6; i++){
// 		for (int j=0; j<6; j++){
// 			KD(i, j) = msg->stiffness_matrix[i*6 + j];
// 			BD(i, j) = msg->damping_matrix[i*6 + j];
// 		}
// 	}
// }


//--------------------------------------------------------------------------------------------------
// ----------------------------------GET Q, partial derivatives of DQ log-----------------------  //


MatrixXd dual_impedance_loop::getQ4 (DQ_robotics::DQ &rot)
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

 MatrixXd dual_impedance_loop::getQ8 (DQ_robotics::DQ &x)
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

MatrixXd dual_impedance_loop::getQ4_dot(DQ_robotics::DQ &rot,DQ_robotics::DQ &drot)
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

MatrixXd dual_impedance_loop::getQ8_dot(DQ_robotics::DQ &x,DQ_robotics::DQ &dx)
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

    ros::init(argc, argv, "dual_impedance_loop");

    ros::NodeHandle node_impedance;

    ros::Rate loop_rate(200); // outer loop frequency Hz
    
    dual_impedance_loop impedance;

    impedance.init(node_impedance);

   //  ros::spinOnce();
  
    while (ros::ok()){

        ros::spinOnce();

        impedance.update();

        loop_rate.sleep();
    }
    return 0;
}

