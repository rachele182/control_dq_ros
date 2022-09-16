#include <iostream>

#include <unistd.h>
#include <cstdlib>
#include <signal.h>
#include <cmath>
#include <math.h>
//MESSAGES
#include <geometry_msgs/PoseStamped.h>
#include <panda_controllers/CompliantTraj.h>
#include <panda_controllers/DesiredProjectTrajectory.h>
#include <panda_controllers/InfoDebug.h>
#include <panda_controllers/DesiredImpedance.h>

#include "ros/ros.h"
#include <sstream>
//DQ ROBOTICS
#include "dqrobotics/DQ.h"
#include "dqrobotics/utils/DQ_Constants.h"
#include "dqrobotics/utils/DQ_LinearAlgebra.h"

//=====================IMPEDANCE MODULATION ====================//

// DEFINE CONSTANTS
#define     K_OR_DEFAULT 600                         // [Nm/rad]     rot stiffness
#define     D_OR_DEFAULT 2*sqrt(K_OR_DEFAULT*1.5)    // [N*s/rad]    rot damping
#define     K_DEFAULT    600                         // [Nm]         default value translation relative stiffness
#define     D_DEFAULT    2*sqrt(K_DEFAULT*1.5)       // [N*s/m]      default value translation relative damping
#define     MASS         1.5                         // [kg]         apparent mass
#define     K_a_DEFAULT  600                         // [Nm]         default absolute translational stiffness     
#define     D_a_DEFAULT  6*sqrt(K_a_DEFAULT*MASS);   // [Nm]         default absolute translational stiffness     
#define     alpha        0.995                       // riducing factor for impedance

typedef Matrix<double, 8, 1> Vector8d; typedef Matrix<double, 6, 1> Vector6d;

using namespace DQ_robotics; using namespace std;  using namespace panda_controllers; 

//VARIABLES
double k;                                   // stiffness value (1 DOF)
double d;                                  // damping value (1 DOF) 
double K[36];                              // stiffness matrix
double D[36];                              // damping matrix
double K_abs[36];                          // stiffness matrix
double D_abs[36];                          // damping matrix
Vector3d phase;                            // phase of trajectory 
Vector3d f_rel;                            // external wrench relative
Vector8d x1_,x2_,xa_,xr_;                  // leftEE,rightEE,absolute,relative poses 
Vector8d pose_a_d_,dpose_a_d_,ddpose_a_d_; // reference abs traj
Vector8d pose_r_d_,dpose_r_d_,ddpose_r_d_; // reference abs traj

//Initialization of gains translational impedance 

double set_k_init(){
    double k;  
    k = K_DEFAULT;  
    return k;
}

double set_d_init(){
    double d;  
    d = D_DEFAULT;  
    return d;
}

//--------------------------------CALLBACKS------------------------------------//

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
   cout << "Caught signal " << signum << endl;
   // Terminate program
   exit(signum);
}

// Callback for robots coop variabòes

void cdts_var_Callback(const panda_controllers::InfoDebugConstPtr& msg){
    for (int i=0; i<8; i++){
          x1_(i) = msg->x1[i];
          x2_(i) = msg->x2[i];
          xa_(i) = msg->abs_pose[i];
          xr_(i) = msg->rel_pose[i];
          }
}

void CompliantTrajCallback(
    	const panda_controllers::CompliantTraj::ConstPtr& msg) {
			for (int i=0; i<8; i++){
          		pose_a_d_(i) = msg->pose_abs_c[i];
          		dpose_a_d_(i) = msg->dpose_abs_c[i];
          		ddpose_a_d_(i) = msg->ddpose_abs_c[i];
          		pose_r_d_(i) = msg->pose_rel_c[i];
		  		dpose_r_d_(i) = msg->dpose_rel_c[i];
		  		ddpose_r_d_(i) = msg->ddpose_rel_c[i];
          }
          for (int i=0; i<3; i++){
            f_rel(i) = msg->fr_rel_frame[i];
        }
		}										

void desiredProjectTrajectoryCallback(
    	const panda_controllers::DesiredProjectTrajectoryConstPtr& msg) {
    phase << msg->phase_array[0], msg->phase_array[1], msg->phase_array[2]; 
 } 


// =================== FUNCTIONS ===================== //
   
// ==-----Compute impedance gains modulation of translational relative stiffness----- == // 

Vector2d compute_gains(double ki,int phase,ros::Time t_curr,ros::Time time_prec){

    //Parameters
    double k_min, k_max,k_max_2,k_rid,k_default,mass, beta, a0, csi,sc; 
    k_max = 4000;
    k_max_2 = 3000; 
    // k_max = 800;
    // k_max_2 = 900; 
    k_min = 30;
    k_default = K_DEFAULT; 
    k_rid = 200; 
    mass = 1.5; 
    beta = 0.98;
    a0 = 0.95;
    csi = 1; 
    sc = 6; //empyrically overdamped scale factor
    
    //variables
    Vector2d gains; 
    double k;
    double d; 
    double k_temp; 

    if(phase==0){
        ki = k_default;
        d = 2*sqrt(ki); 
    }else if(phase==1){ // approach pahse
            ki = alpha*ki; // decrease k arbitrarly
            d = sc*sqrt(ki*mass); 
            if(ki < k_rid){
                ki = k_rid;
                d = sc*sqrt(ki*mass);
        }       
    }else if(phase==2){ //squeeze phase
        double k_dot; 
                double interval = (t_curr - time_prec).toSec();
                k_dot = beta*(4*a0*sqrt(ki/mass)*pow(ki,3/2))/(sqrt(ki) + 2*a0*csi*sqrt(ki));
                k_temp = ki + k_dot*interval; 
                ki = k_temp;  
                d = sc*sqrt(ki*mass); 
                if(k_temp>k_max){
                    ki = k_max;
                    d = sc*sqrt(ki*mass); 
                }
    }else if(phase==3){ //compensate weight
        double k_dot; 
                double interval = (t_curr - time_prec).toSec();
                k_dot = beta*(4*a0*sqrt(ki/mass)*pow(ki,3/2))/(sqrt(ki) + 2*a0*csi*sqrt(ki));
                k_temp = ki + k_dot*interval; 
                ki = k_temp;  
                d = sc*sqrt(ki*mass); 
                if(k_temp>k_max_2){
                    ki = k_max_2;
                    d = sc*sqrt(ki*mass); 
                }
    }else if(phase==4){ //release and eq phase
        ki = alpha*ki; // decrease k arbitrarly
        d = 2*sqrt(ki*mass); 
            if(ki < k_default){
              ki = k_default;
              d = sc*sqrt(ki*mass);
        }       
    }
        k = ki; 
        gains << k,d; 
    return gains; 
    }


// ==-----Compute impedance gains modulation of rotational relative stiffness----- == // 

Vector2d compute_gains_rot(double ki,int phase,ros::Time t_curr,ros::Time time_prec){

    //Parameters
    double k_default,k_rid,mass, beta, a0, csi,sc; 
    k_default = K_OR_DEFAULT; 
    k_rid = 20; 
    mass = 1.5; 
    beta = 0.98;
    a0 = 0.95;
    csi = 1; 
    sc = 2; //critically damped

    //variables
    Vector2d gains_rot; 
    double k; double d; double k_temp; 

    if(phase==0){ //approach phase 
        ki = k_default;
        d = 2*sqrt(ki); 
    }else if(phase ==1){
        ki = ki;
        d = 2*sqrt(ki);        
    }else if(phase==2){ // squeeze
            ki = alpha*ki; // decrease k arbitrarly
            d = sc*sqrt(ki*mass); 
            if(ki < k_rid){
                ki = k_rid;
                d = sc*sqrt(ki*mass);
            }       
    }else if(phase==3){ //compensate weight (reincrease rotational stiffness)
        double k_dot; 
        double interval = (t_curr - time_prec).toSec();
        k_dot = beta*(4*a0*sqrt(ki/mass)*pow(ki,3/2))/(sqrt(ki) + 2*a0*csi*sqrt(ki));
        k_temp = ki + k_dot*interval; 
        ki = k_temp;  
        d = 2*sqrt(ki*mass); 
        if(k_temp>k_default){
            ki = k_default;
            d = 2*sqrt(ki*mass); 
        } 
    }else if(phase==4){ //release phase
        ki = ki;
        d = 2*sqrt(ki*mass); 
    }
        k = ki; 
        gains_rot << k,d; 
    return gains_rot; 
    }

// ==-----Compute impedance gains modulation of translational absolute stiffness----- == // 

Vector2d compute_abs_gains(double ki,int phase,ros::Time t_curr,ros::Time time_prec){

    //Parameters
    double k_min, k_max,k_max_2,k_rid,k_default,mass, beta, a0, sc,csi; 
    k_max = 700;
    k_max_2 = 800; 
    k_default = K_a_DEFAULT; 
    k_rid = 300; 
    mass = 1.5; 
    beta = 0.98;
    a0 = 0.95;
    csi = 1; 
    sc = 6; //empyrically overdamped scale factor
    //variables
    Vector2d gains_abs; 
    double k,d, k_temp; 

    if(phase==0){
        ki = k_default;
        d = 2*sqrt(ki); 
    }else if(phase==1){ // approach pahse
            ki = alpha*ki; // decrease k arbitrarly
            d = sc*sqrt(ki*mass); 
            if(ki < k_rid){
                ki = k_rid;
                d = sc*sqrt(ki*mass);
        }       
    }else if(phase==2){ //squeeze phase
        double k_dot; 
                double interval = (t_curr - time_prec).toSec();
                k_dot = beta*(4*a0*sqrt(ki/mass)*pow(ki,3/2))/(sqrt(ki) + 2*a0*csi*sqrt(ki));
                k_temp = ki + k_dot*interval; 
                ki = k_temp;  
                d = sc*sqrt(ki*mass); 
                if(k_temp>k_max){
                    ki = k_max;
                    d = 2*sqrt(ki*mass); 
                }
    }else if(phase==3){ //compensate weight
        double k_dot; 
                double interval = (t_curr - time_prec).toSec();
                k_dot = beta*(4*a0*sqrt(ki/mass)*pow(ki,3/2))/(sqrt(ki) + 2*a0*csi*sqrt(ki));
                k_temp = ki + k_dot*interval; 
                ki = k_temp;  
                d = sc*sqrt(ki*mass); 
                if(k_temp>k_max_2){
                    ki = k_max_2;
                    d = 2*sqrt(ki*mass); 
                }
    }else if(phase==4){ //release and eq phase
        ki = alpha*ki; // decrease k arbitrarly
        d = 2*sqrt(ki*mass); 
            if(ki < k_default){
              ki = k_default;
              d = 2*sqrt(ki*mass);
        }       
    }
        k = ki; 
        gains_abs << k,d; 
    return gains_abs; 
    }

    Vector2d compute_gains_abs_rot(double ki,int phase,ros::Time t_curr,ros::Time time_prec){

    //Parameters
    double k_default,k_rid,mass, beta, a0, csi,sc; 
    k_default = K_OR_DEFAULT; 
    k_rid = 50; 
    mass = 1.5; 
    beta = 0.98;
    a0 = 0.95;
    csi = 1; 
    sc = 2; //critically damped

    //variables
    Vector2d gains_abs_rot; 
    double k; double d; double k_temp; 

    if(phase==0){ //approach phase 
        ki = k_default;
        d = 2*sqrt(ki); 
    }else if(phase ==1){
        ki = ki;
        d = 2*sqrt(ki);        
    }else if(phase==2){ // squeeze
            ki = alpha*ki; // decrease k arbitrarly
            d = sc*sqrt(ki*mass); 
            if(ki < k_rid){
                ki = k_rid;
                d = sc*sqrt(ki*mass);
            }       
    }else if(phase==3){ //compensate weight (reincrease rotational stiffness)
        double k_dot; 
        double interval = (t_curr - time_prec).toSec();
        k_dot = beta*(4*a0*sqrt(ki/mass)*pow(ki,3/2))/(sqrt(ki) + 2*a0*csi*sqrt(ki));
        k_temp = ki + k_dot*interval; 
        ki = k_temp;  
        d = 2*sqrt(ki*mass); 
        if(k_temp>k_default){
            ki = k_default;
            d = 2*sqrt(ki*mass); 
        } 
    }else if(phase==4){ //release phase
        ki = ki;
        d = 2*sqrt(ki*mass); 
    }
        k = ki; 
        gains_abs_rot << k,d; 
    return gains_abs_rot; 
    }



int main(int argc, char **argv)
{

  using namespace panda_controllers; 

  ros::init(argc, argv, "modulator");

  ros::NodeHandle node_imp;

  ros::Publisher pub_impedance = node_imp.advertise<panda_controllers::DesiredImpedance>("/motion_control_dq/desired_impedance", 1);

  ros::Subscriber sub_traj_nom =  node_imp.subscribe("/motion_control_dq/dq_trajectory", 1, 
                                                &desiredProjectTrajectoryCallback);

  ros::Subscriber sub_traj_comp =  node_imp.subscribe("/motion_control_dq/compliant_traj", 1, 
                                                &CompliantTrajCallback);

  ros::Subscriber sub_ext_forces =  node_imp.subscribe("/motion_control_dq/info_debug", 1, 
                                                &cdts_var_Callback);


  ros::Rate loop_rate(200);

  panda_controllers::DesiredImpedance imp_msg;

  signal(SIGINT, signal_callback_handler);

  // Variables  
  Vector8d pose_nom;
  double int_pos; 
  pose_nom << 1,0,0,0,0,0,0,0;
  int sw; // single or dual arm
  Vector2d gains_x,gains_y,gains_z; Vector2d gains_rot_x,gains_rot_y,gains_rot_z;
  Vector2d gains_abs_x,gains_abs_y,gains_abs_z; Vector2d gains_abs_rot_x,gains_abs_rot_y,gains_abs_rot_z;
  double kx,ky,kz,dx,dy,dz;                                    //translational relative impedance
  double kx_rot,ky_rot,kz_rot,dx_rot,dy_rot,dz_rot;            // rotational relative impedance
  double kx_a,ky_a,kz_a,dx_a,dy_a,dz_a;                        // translational absolute impedance
  double kxa_rot, kya_rot,kza_rot,dxa_rot,dya_rot,dza_rot;     // rotational absolute impedance
  double K[36];            // stiffness matrix 6x6
  double D[36];            // damping matrix 6x6 
  double K_abs[36];        // absolute stiffness matrix
  double D_abs[36];        // absolute damping matrix
  int count = 0;           // counter
  ros::Time t_curr;        // current time
  ros::Time time_prec;     // previous cycle time

  // initialize array  
  for (int i = 0; i < 36; i++) {
        K[i] = 0;
        D[i] = 0; 
        K_abs[i] = 0;
        D_abs[i] = 0;
    }
 
  while (ros::ok())
  {
    if(count == 0){
        cout << "choice: (0:single, 1:dual)" << endl; 
        cin >> sw;
    }

    ros::spinOnce();

    if(count == 0){
        //initialize
        time_prec = ros::Time::now();
        kx = set_k_init(); ky = set_k_init(); kz = set_k_init();  kx_rot = K_OR_DEFAULT; ky_rot = K_OR_DEFAULT; kz_rot = K_OR_DEFAULT; 
        dx = set_d_init(); dy = set_d_init(); dz = set_d_init();  dx_rot = D_OR_DEFAULT; dy_rot = D_OR_DEFAULT; dz_rot = D_OR_DEFAULT; 
        kx_a = K_a_DEFAULT; ky_a = K_a_DEFAULT; kz_a = K_a_DEFAULT; dx_a = D_a_DEFAULT; dy_a = D_a_DEFAULT; dz_a = D_a_DEFAULT; 
        kxa_rot = K_OR_DEFAULT; kya_rot = K_OR_DEFAULT; kza_rot = K_OR_DEFAULT; dxa_rot = D_OR_DEFAULT; dya_rot = D_OR_DEFAULT; dza_rot = D_OR_DEFAULT;
    }
        t_curr = ros::Time::now();
        
        //translational gains compuation
        gains_x = compute_gains(kx,phase(2),t_curr,time_prec);
        gains_y = compute_gains(ky,phase(2),t_curr,time_prec);
        gains_z = compute_gains(kz,phase(2),t_curr,time_prec);
        //rot gains computation (//reduce on all rot axis?)
        gains_rot_x = compute_gains_rot(kx_rot,phase(2),t_curr,time_prec); 
        gains_rot_y = compute_gains_rot(ky_rot,phase(2),t_curr,time_prec); 
        gains_rot_z = compute_gains_rot(kz_rot,phase(2),t_curr,time_prec); 
        // translational abs gains computation
        gains_abs_x = compute_abs_gains(kx_a,phase(2),t_curr,time_prec);
        gains_abs_y = compute_abs_gains(ky_a,phase(2),t_curr,time_prec);
        gains_abs_z = compute_abs_gains(kz_a,phase(2),t_curr,time_prec); 
        //rot abs gain computation
        gains_abs_rot_x = compute_gains_abs_rot(kxa_rot,phase(2),t_curr,time_prec); 
        gains_abs_rot_y = compute_gains_abs_rot(kya_rot,phase(2),t_curr,time_prec); 
        gains_abs_rot_z = compute_gains_abs_rot(kza_rot,phase(2),t_curr,time_prec); 

        //Translational relative imp
        kx = gains_x(0);  ky = gains_y(0); kz = gains_z(0); 
        dx = gains_x(1);  dy = gains_y(1); dz = gains_z(1);

        //Rotational rel imp
        kx_rot = gains_rot_x(0); ky_rot = gains_rot_y(0); kz_rot = gains_rot_z(0);
        dx_rot = gains_rot_x(1); dy_rot = gains_rot_y(1); dz_rot = gains_rot_z(1);

        //Translational abs imp
        kx_a = gains_abs_x(0); ky_a = gains_abs_y(0); kz_a = gains_abs_z(0); 
        dx_a = gains_abs_x(1); dy_a = gains_abs_y(1); dz_a = gains_abs_z(1);
        
        //Rotational abs imp
        kxa_rot = gains_abs_rot_x(0); kya_rot = gains_abs_rot_y(0); kza_rot = gains_abs_rot_z(0);
        dxa_rot = gains_abs_rot_x(1); dya_rot = gains_abs_rot_y(1); dza_rot = gains_abs_rot_z(1);

        time_prec = t_curr; 

        // Final impedance matrices
        K[0] = kx_rot; K[7] = ky_rot; K[14] = kz_rot;
        K[21] = kx; K[28] = ky; K[35] = kz;

        D[0] = dx_rot; D[7] = dy_rot; D[14] = dz_rot;
        D[21] = dx; D[28] = dy; D[35] = dz;

        K_abs[0] = kxa_rot; K_abs[7] = kya_rot; K_abs[14] = kza_rot;
        K_abs[21] = K_a_DEFAULT; K_abs[28] = K_a_DEFAULT; K_abs[35] =  K_a_DEFAULT;

        D_abs[0] = dxa_rot; D_abs[7] = dya_rot; D_abs[14] = dza_rot;
        D_abs[21] = D_a_DEFAULT;  D_abs[28] = D_a_DEFAULT; D_abs[35] = D_a_DEFAULT;

        // Publish desired impedance
         imp_msg.header.stamp = ros::Time::now();

             for ( int i = 0; i <36; i++){
             imp_msg.stiffness_matrix[i] = K[i];
             imp_msg.damping_matrix[i] = D[i];
             imp_msg.stiffness_abs_matrix[i] = K_abs[i];
             imp_msg.damping_abs_matrix[i] = D_abs[i]; 
             }

         pub_impedance.publish(imp_msg);

         loop_rate.sleep();

         count = count+1;
  }
 return 0;
}




