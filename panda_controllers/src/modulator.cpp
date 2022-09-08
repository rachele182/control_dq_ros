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
#define     F_MAX       6                           // [N]          disturbance threshold (from dead-zone computation)
#define     F_INT_MAX   10                          // [N]          maximum tollerated force interaction
#define     K_OR        600                         // [Nm/rad]     rot stiffness
#define     D_OR        2*sqrt(K_OR*1.5)            // [N*s/rad]    rot damping
#define     K_DEFAULT   200                         // [Nm]         default value translation stiffness
#define     D_DEFAULT   2*sqrt(K_DEFAULT*1.5)       // [N*s/m]      default value translation stiffness
#define     MASS        1.5                         // [kg]         apparent mass

typedef Matrix<double, 8, 1> Vector8d;
typedef Matrix<double, 6, 1> Vector6d;

using namespace DQ_robotics;
using namespace std; 
using namespace panda_controllers; 

//VARIABLES

double k;                                // stiffness value (1 DOF)
double d;                                // damping value (1 DOF) 
double K[36];                            // stiffness matrix
double D[36];                            // damping matrix
Vector3d phase;                          // phase of trajectory 
Vector3d f_rel;                          // external wrench relative
Vector8d x1_,x2_,xa_,xr_;                //leftEE,rightEE,absolute,relative poses 
Vector8d pose_a_d_,dpose_a_d_,ddpose_a_d_; //reference abs traj
Vector8d pose_r_d_,dpose_r_d_,ddpose_r_d_; //reference abs traj

//Initialization of gains

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
   

// -----Compute impedance gains modulation----- // 

Vector2d compute_gains(double ki,int phase, double f_ext, double F_max, double F_int_max,ros::Time t_curr,ros::Time time_prec){

    //Parameters
    double k_min, k_max,k_max_2,k_default,mass, beta, a0, csi,sc; 
    k_max = 500;
    k_max_2 = 600; 
    k_min = 30;
    k_default = 200; 
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
    double int_pos; 

    if(phase==0){
        ki = k_default;
        d = 2*sqrt(ki); 
    }else if(phase==1){ //
        // d = sc*sqrt(ki*mass); 
        // if(std::abs(f_ext) > F_max){
            ki = 0.998*ki; // decrease k arbitrarly
            d = sc*sqrt(ki*mass); 
            if(ki < k_default){
                ki = k_default;
                d = sc*sqrt(ki*mass); 
                    // }
        }       
    }else if(phase==2){ //squeeze
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
    }else if(phase==3){
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
    }
        k = ki; 
        gains << k,d; 
    return gains; 
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
  Vector2d gains_x,gains_y,gains_z;
  double kx,ky,kz,dx,dy,dz; 
  double K[36];            // stiffness matrix 6x6
  double D[36];            // damping matrix 6x6 
  int count = 0;           // counter
  ros::Time t_curr;        // current time
  ros::Time time_prec;     // previous cycle time

  // initialize array  
  for (int i = 0; i < 36; i++) {
        K[i] = 0;
        D[i] = 0; 
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
        kx = set_k_init();
        ky = set_k_init();
        kz = set_k_init();
        dx = set_d_init();
        dy = set_d_init();
        dz = set_d_init();
    }

        t_curr = ros::Time::now();
         
        gains_x = compute_gains(kx,phase(0),f_rel(0),F_MAX,F_INT_MAX,t_curr,time_prec);
        gains_y = compute_gains(ky,phase(1),f_rel(1),F_MAX,F_INT_MAX,t_curr,time_prec);
        gains_z = compute_gains(kz,phase(2),f_rel(2),F_MAX,F_INT_MAX,t_curr,time_prec);
        
        kx = gains_x(0); dx = gains_x(1); 
        ky = gains_y(0); dy = gains_y(1); 
        kz = gains_z(0); dz = gains_z(1);

        time_prec = t_curr; 

        // Final impedance matrices
        K[0] = K_OR;
        K[7] = K_OR;
        K[14] = K_OR;
        K[21] = kx;
        K[28] = ky;
        K[35] = kz;

        D[0] = D_OR;
        D[7] = D_OR;
        D[14] = D_OR;
        D[21] = dx;
        D[28] = dy;
        D[35] = dz;

        // Publish desired impedance
         imp_msg.header.stamp = ros::Time::now();

             for ( int i = 0; i <36; i++){
             imp_msg.stiffness_matrix[i] = K[i];
             imp_msg.damping_matrix[i] = D[i];
             }

         pub_impedance.publish(imp_msg);

         loop_rate.sleep();

         count = count+1;
  }
 return 0;
}




