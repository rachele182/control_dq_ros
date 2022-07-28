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
#define     K_OR        300                         // [Nm/rad]     rot stiffness
#define     D_OR        2*sqrt(K_OR*1.5)            // [N*s/rad]    rot damping
#define     K_DEFAULT   100                         // [Nm]         default value translation stiffness
#define     D_DEFAULT   4*sqrt(K_DEFAULT*1.5)       // [N*s/m]      default value translation stiffness
#define     MASS        1.5                         // [kg]         apparent mass
#define     DZ_VALUE      6   

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
int phase;                 // phase of trajectory 
Vector6d wrench_ext;       // external wrench
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
    phase = msg->phase; 
 } 

void f_ext_Callback(const panda_controllers::InfoDebugConstPtr& msg){
  wrench_ext << msg->wrench_ext[0], msg->wrench_ext[1],msg->wrench_ext[2], msg->wrench_ext[3],msg->wrench_ext[4],msg->wrench_ext[5];
}

// =================== FUNCTIONS ===================== //

Vector6d dead_zone(Vector6d wrench_ext, double dz_value){
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
   

// -----Compute impedance gains modulation----- // 

double compute_k(double ki,int phase, double f_ext,double pos,double e_pos, double F_max, double F_int_max,ros::Time t_curr,ros::Time time_prec){

    //Parameters
    double k_min, k_max, mass, beta, a0, csi,safe_tr,z_int; 
    k_max = 500;
    k_min = 30;
    mass = 1.5; 
    beta = 0.98;
    a0 = 0.95;
    csi = 1; 
    z_int = 0.15; 
    
    //variables
    double k;
    double k_temp; 
    double int_pos; 

    if(phase==0){
        if(std::abs(f_ext) > F_max){
            ki = 0.995*ki; // decrease k arbitrarly
            if(ki < k_min){
                ki = k_min;
                    }
            if(ki*std::abs(e_pos) > F_int_max){
                k_temp = (F_int_max)/(std::abs(e_pos));
                if(k_temp<ki){
                    ki = k_temp;
                    if(k_temp < k_min){
                        ki = k_min;
                    }
                }     
         }
        }       
    }else if(phase==1){
        double k_dot; 
            if(pos>z_int){ //safe to increase K
                double interval = (t_curr - time_prec).toSec();
                k_dot = beta*(4*a0*sqrt(ki/mass)*pow(ki,3/2))/(sqrt(ki) + 2*a0*csi*sqrt(ki));
                k_temp = ki + k_dot*interval; 
                ki = k_temp;  
                if(k_temp>k_max){
                    ki = k_max;
                }
            }
        }
        k = ki; 
    return k; 
    }



double compute_d(double k, double mass){
    double d;
    d = 4*sqrt(mass*k);
return d; 
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
                                                &f_ext_Callback);

  ros::Subscriber sub_pose =  node_imp.subscribe("/motion_control_dq/franka_ee_pose", 1, 
                                                &poseCallback);

  ros::Rate loop_rate(100);

  panda_controllers::DesiredImpedance imp_msg;

  signal(SIGINT, signal_callback_handler);

  // Variables  
  Vector8d pose_nom;
  double int_pos; 
  pose_nom << 1,0,0,0,0,0,0,0;
  int ph; 
  double kx,ky,kz,dx,dy,dz; 
  double K[36];            // stiffness matrix 6x6
  double D[36];            // damping matrix 6x6 
  int sw;                  // flag to switch between impedance and admittance
  int count = 0;           // counter
  Vector3d pos;            // EE position
  Vector4d rot;            // EE rotation
  Vector8d pose_d_;        // deired nominal traj
  Vector8d pose_c_;        // deired computed traj
  DQ e_dq;                 // DQ pose error
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

    //Dead zone ext forces
    wrench_n = dead_zone(wrench_ext,DZ_VALUE); 

    //current pose DQ
    pose_dq = or_dq + 0.5*E_*(pos_dq*or_dq); 
    pose_dq = pose_dq.normalize();

    if(count == 0){
        //initialize
        time_prec = ros::Time::now();
        pose_d_ << pose_nom; 
        pose_c_ << pose_nom; 
        kx = set_k_init();
        ky = set_k_init();
        kz = set_k_init();
        dx = set_d_init();
        dy = set_d_init();
        dz = set_d_init();
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
                e_dq = pose_dq.conj()*pose_d_dq; 
                e_dq = e_dq.normalize(); 
              }
            else{
                e_dq = DQ(pose_nom); 
            }
        }else if (sw==1){

            if(pose_c_ != pose_nom){
                //compliant desired pose DQ
                pose_c_ << pose_c; 
                pose_c_dq = DQ(pose_c_);
                pose_c_dq = pose_c_dq.normalize();
                e_dq = pose_dq.conj()*pose_c_dq; 
                e_dq = e_dq.normalize(); 
            }
            else{
                e_dq = DQ(pose_nom); 
            }
        }
  
        e_pos = vec3(e_dq.translation()); 
        t_curr = ros::Time::now();

        ph = phase; 
        // kx = compute_k(kx,ph,wrench_n(0),position(0),e_pos(0),F_MAX,F_INT_MAX,t_curr,time_prec);
        // ky = compute_k(ky,ph,wrench_n(1),position(1),e_pos(1),F_MAX,F_INT_MAX,t_curr,time_prec);
        kz = compute_k(kz,ph,wrench_n(2),position(2),e_pos(2),F_MAX,F_INT_MAX,t_curr,time_prec);
        
        dx = compute_d(kx,MASS);
        dy = compute_d(ky,MASS);
        dz = compute_d(kz,MASS);

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




