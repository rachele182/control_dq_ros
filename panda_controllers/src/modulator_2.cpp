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
#define     K_OR        500                         // [Nm/rad]     rot stiffness
#define     D_OR        2*sqrt(K_OR*1.5)            // [N*s/rad]    rot damping
#define     K_DEFAULT   300                         // [Nm]         default value translation stiffness
#define     D_DEFAULT   2*sqrt(K_DEFAULT*1.5)       // [N*s/m]      default value translation stiffness
#define     MASS        1.5                         // [kg]         apparent mass
#define     DZ_VALUE    5   

typedef Matrix<double, 8, 1> Vector8d;
typedef Matrix<double, 6, 1> Vector6d;

using namespace DQ_robotics;
using namespace std; 
using namespace panda_controllers; 

//VARIABLES

double k;                  // stiffness value (1 DOF)
double d;                  // damping value (1 DOF) 
double K[36];              // stiffness matrix
double D[36];              // damping matrix
DQ pos_dq;                 // EE position DQ
DQ or_dq;                  // EE rotation DQ
Vector3d position;         // current EE position
Vector4d orientation;      // current EE orientation
Vector3d position_d_;      // nominal desired trajectory
Vector3d position_c_;      // compute compliant trajectory
Vector3d phase;            // phases of nominal trajectory 
Vector8d pose_d;           // nominal pose
Vector8d pose_c;           // compliant pose

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

// Callback for robot pose
void poseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  orientation << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z; //w real part  
}

void CompliantTrajCallback(
    	const panda_controllers::CompliantTraj::ConstPtr& msg) {
	position_c_ << msg->position_c[0], msg->position_c[1], msg->position_c[2];
    pose_c << msg->pose_c[0], msg->pose_c[1], msg->pose_c[2],msg->pose_c[3],msg->pose_c[4],msg->pose_c[5],msg->pose_c[6],msg->pose_c[7];		
        }

void desiredProjectTrajectoryCallback(
    	const panda_controllers::DesiredProjectTrajectoryConstPtr& msg) {
	position_d_ << msg->position_d[0], msg->position_d[1], msg->position_d[2]; 
    pose_d << msg->pose_d[0], msg->pose_d[1], msg->pose_d[2],msg->pose_d[3],msg->pose_d[4],msg->pose_d[5],msg->pose_d[6],msg->pose_d[7];
    phase << msg->phase_array[0], msg->phase_array[1], msg->phase_array[2]; 
 } 


// =================== FUNCTIONS ===================== //

// -----Compute impedance gains modulation----- // 

Vector2d compute_gains(double ki,int phase,double pos,ros::Time t_curr,ros::Time time_prec){

    //Parameters
    double k_min, k_max, mass, beta, a0, csi,z_int; 
    double sc; 
    k_max = 500;
    k_min = 30;
    mass = 1.5; 
    beta = 0.98;
    a0 = 0.95;
    csi = 1; 
    z_int = 0.15; 
    sc = 8; //overdamping factor
    
    //variables
    Vector2d gains; 
    double k;
    double d; 
    double k_temp; 
    double int_pos; 

    if(phase==0){
        ki = K_DEFAULT; 
        d = 2*sqrt(ki*mass);
    }else if(phase==1){
            ki = 0.95*ki; // decrease k arbitrarly during interaction phase
            d = sc*sqrt(ki*mass); // euristic overdamped to avoid oscillation
            if(ki < k_min){
                ki = k_min;
                d = sc*sqrt(ki*mass); 
                    }
    }else if(phase==2){
        double k_dot; 
            if(pos>z_int){ //safe to increase K
                double interval = (t_curr - time_prec).toSec();
                k_dot = beta*(4*a0*sqrt(ki/mass)*pow(ki,3/2))/(sqrt(ki) + 2*a0*csi*sqrt(ki));
                k_temp = ki + k_dot*interval; 
                ki = k_temp;  
                d = 2*sqrt(ki*mass); 
                if(k_temp>k_max){
                    ki = k_max;
                    d = 2*sqrt(ki*mass); 
                }
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

  ros::Subscriber sub_pose =  node_imp.subscribe("/motion_control_dq/franka_ee_pose", 1, 
                                                &poseCallback);

  ros::Rate loop_rate(200);

  panda_controllers::DesiredImpedance imp_msg;

  signal(SIGINT, signal_callback_handler);

  // Variables  
  Vector8d pose_nom;
  double int_pos; 
  pose_nom << 1,0,0,0,0,0,0,0;
  Vector3d ph; 
  Vector2d gains_x; 
  Vector2d gains_y;
  Vector2d gains_z;
  double kx,ky,kz,dx,dy,dz; 
  double K[36];            // stiffness matrix 6x6
  double D[36];            // damping matrix 6x6 
  int sw;                  // flag to switch between impedance and admittance
  int count = 0;           // counter
  Vector3d pos;            // EE position
  Vector4d rot;            // EE rotation
  Vector8d pose_d_;        // deired nominal traj
  Vector8d pose_c_;        // deired computed traj
  DQ pose_dq;              // current pose
  DQ pose_d_dq;            // desired nominal pose
  DQ pose_c_dq;            // desired compliant pose
  Vector3d e_pos;          // position error
  Vector6d wrench_n;       // external wrench
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
        cout << "choice: (0:impedance, 1:admittance)" << endl; 
        cin >> sw;
    }

    ros::spinOnce();

    pos << position;
    rot << orientation; 

    pos_dq = DQ(pos);
    or_dq = DQ(rot); 

    //current pose DQ
    pose_dq = or_dq + 0.5*E_*(pos_dq*or_dq); 
    pose_dq = pose_dq.normalize();

    if(count == 0){
        //initialize
        time_prec = ros::Time::now();
        pose_d_ << pose_nom; 
        pose_c_ << pose_nom; 
    }
        or_dq = DQ(rot); 
        pos_dq = DQ(pos);
        pose_dq = or_dq + 0.5*E_*(pos_dq*or_dq); 
        pose_dq = pose_dq.normalize();
      
        if(sw==0){
            if(pose_d_ != pose_nom){
                //nominal desired pose DQ
                pose_d_ << pose_d; 
                pose_d_dq = DQ(pose_d_);
                pose_d_dq = pose_d_dq.normalize(); 
              }
        }else if (sw==1){

            if(pose_c_ != pose_nom){
                //compliant desired pose DQ
                pose_c_ << pose_c; 
                pose_c_dq = DQ(pose_c_);
                pose_c_dq = pose_c_dq.normalize();
            }
        }
  
        t_curr = ros::Time::now();

        ph = phase; 

        kx = set_k_init();
        ky = set_k_init();
        kz = set_k_init();
        dx = set_d_init();
        dy = set_d_init();
        dz = set_d_init();
        
        gains_x = compute_gains(kx,ph(0),position(0),t_curr,time_prec);
        gains_y = compute_gains(kz,ph(1),position(1),t_curr,time_prec);
        gains_z = compute_gains(kz,ph(2),position(2),t_curr,time_prec);

        kx = gains_x(0); 
        dx = gains_x(1); 
        ky = gains_y(0); 
        dy = gains_y(1); 
        kz = gains_z(0);
        dz = gains_z(1);

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




