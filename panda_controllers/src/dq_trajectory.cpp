#include <iostream>

#include <unistd.h>
#include <cstdlib>
#include <signal.h>
#include <cmath>
#include <math.h>


#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include "utils/parsing_utilities.h"
#include "ros/ros.h"

#include <sstream>
#include <eigen_conversions/eigen_msg.h>

// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Pose.h"


#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "dqrobotics/DQ.h"
#include "dqrobotics/utils/DQ_Constants.h"
#include "dqrobotics/utils/DQ_LinearAlgebra.h"
#include <panda_controllers/DesiredProjectTrajectory.h>


// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Pose.h"


//-----------------------DEFINE--------------------------//

typedef Matrix<double, 8, 1> Vector8d;

using namespace std; 
using namespace DQ_robotics;
using namespace panda_controllers; 

using DQ_robotics::E_;
using DQ_robotics::i_;
using DQ_robotics::j_;
using DQ_robotics::k_;
using DQ_robotics::C8;

//initial position and orientation from Franka
Vector3d pos_d;
Vector4d or_d; 

// desired trajectory 
Vector3d pos_des;
Vector3d pos_check;
Vector4d or_check;  
Vector3d vel_des;
Vector3d acc_des; 

// DQ trajectory
DQ pos_des_dq;  
DQ vel_des_dq;  
DQ acc_des_dq; 
DQ x_des_dq;  
DQ dx_des_dq;  
DQ ddx_des_dq;  
DQ or_dq; 

struct traj_t_struct{
      Vector3d p_des;
      Vector3d v_des;
      Vector3d a_des;
  } traj_t;

struct traj_dq_struct{
      Vector8d x_des;
      Vector8d dx_des;
      Vector8d ddx_des;
  } trajdq;


// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
   cout << "Caught signal " << signum << endl;
   // Terminate program
   exit(signum);
}

// INIT

// Call back for robot pose
void poseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  
  pos_d << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  or_d << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z; //w real part  
}

// Trajectories demos

void demo_inf_XY(Eigen::Vector3d pos_i, double t){
    Vector3d tmp;
    tmp << sin(t)/8, sin(t/2)/4, 0;
    traj_t.p_des << pos_i + tmp;
    traj_t.v_des << cos(t)/8, cos(t/2)/8, 0;
    traj_t.a_des << -sin(t)/8, -sin(t/2)/16, 0;
    
}

void demo_inf_XYZ(Eigen::Vector3d pos_i, double t,double zf,double tf){
    Vector3d tmp;
    tmp << sin(t)/8, sin(t/2)/4, ((zf-pos_i(2))/tf)*t;
    traj_t.p_des << pos_i + tmp;
    traj_t.v_des << cos(t)/8, cos(t/2)/8, (zf-pos_i(2))/tf;
    traj_t.a_des << -sin(t)/8, -sin(t/2)/16, 0;
    
}

void demo_circle_xy(Eigen::Vector3d pos_i, double t,double zf,double tf){
  Vector3d tmp;
  tmp << 0.1*cos(t), 0.1*sin(t), ((zf-pos_i(2))/tf)*t;
  traj_t.p_des << pos_i + tmp;
  traj_t.v_des << -0.1*sin(t), 0.1*cos(t), (zf-pos_i(2))/tf;
  traj_t.a_des << -0.1*cos(t), -0.1*sin(t), 0;
}

// Minimum jerk interpolation

void interpolator_pos( Vector3d pos_i, Vector3d pos_f,
                              double tf, double t) {

        traj_t.p_des << pos_i + (pos_i - pos_f)*(15*pow((t/tf),4) - 6*pow((t/tf),5) -10*pow((t/tf),3));
        traj_t.v_des << (pos_i - pos_f)*(60*(pow(t,3)/pow(tf,4)) - 30*(pow(t,4)/pow(tf,5)) -30*(pow(t,2)/pow(tf,3)));
        traj_t.a_des << (pos_i - pos_f)*(180*(pow(t,2)/pow(tf,4)) - 120*(pow(t,3)/pow(tf,5)) -60*(t/pow(tf,3)));
  }


// COMPUTE DQ TRAJECTORY

  void gen_traj_dq ( Vector3d pos_des, Vector3d vel_des,
                       Vector3d acc_des, DQ or_dq
                              ){  
        pos_des_dq = DQ(pos_des);       
        x_des_dq = or_dq + 0.5*E_*(pos_des_dq*or_dq); 
        x_des_dq = x_des_dq * x_des_dq.inv().norm();
        trajdq.x_des << vec8(x_des_dq); 

        vel_des_dq = DQ(vel_des);
        dx_des_dq = 0.5*E_*(vel_des_dq*or_dq); 
        trajdq.dx_des << vec8(dx_des_dq); 

        acc_des_dq = DQ(acc_des);
        ddx_des_dq = 0.5*E_*(acc_des_dq*or_dq); 
        trajdq.ddx_des << vec8(ddx_des_dq); 
  }

int main(int argc, char **argv)
{
  using namespace panda_controllers; 
  using namespace XmlRpc;

  ros::init(argc, argv, "dq_traj");

  ros::NodeHandle node_handle;

  ros::Publisher pub_cmd = node_handle.advertise<panda_controllers::DesiredProjectTrajectory>("/motion_control_dq/dq_trajectory", 1000);

  ros::Subscriber sub_cmd =  node_handle.subscribe("/motion_control_dq/franka_ee_pose", 1, 
                                                &poseCallback);

  ros::Rate loop_rate(100);

  panda_controllers::DesiredProjectTrajectory traj_msg;
   
  Eigen::Vector3d pos_f;
  Eigen::Vector3d or_f;
  double tf;
  double zf;
  Eigen::Vector3d vel;
  Eigen::Vector3d pos_init;
  Eigen::Vector3d or_init;
  XmlRpc::XmlRpcValue traj_par;
  // std::map<std::string, std::vector<double>>  traj_par; 

  int N_ACTION;
  Eigen::MatrixXd ACTIONS;
  Eigen::MatrixXi TYPE;

  signal(SIGINT, signal_callback_handler);

  ros::Time t_init;
  double t = 0;
  int choice;
  int demo = -1;
  int yaml = 0;
  int n_act = 0;

  while (ros::ok()){

    demo = -1;
    if (yaml==1){
      choice = 3;
    }else{
      cout<<"choice:   (1:jerk_traj, 2:demos, 3:yaml) "<<endl;
      cin>>choice;
    }
    
    if (choice == 1){
      cout<<"time_f "<<endl;
      cin>>tf;
      cout<<"final_position "<<endl;
      cin>> pos_f.x();
      cin>> pos_f.y();
      cin>> pos_f.z();
    }else if (choice == 2){
      cout<<"select demo:   (1: infinite XY , 2: infinite XYZ , 3: circle_xy "<<endl;
      cin>>demo;
      cout<<"insert time_f: "<<endl;
      cin>>tf;
      if ((demo==2) || (demo==3)){
        cout<<"insert zf: "<<endl;
        cin>>zf;
      }
    }else if (choice == 3){
      if (yaml==0){
        yaml = 1;
        if(!node_handle.getParam("/traj_par", traj_par)){
          ROS_ERROR("Could not get the XmlRpc value.");
        }
        if(!parseParameter(traj_par, N_ACTION, "N_ACTION")){
          ROS_ERROR("Could not parse traj_par.");
        }
        if(!parseParameter(traj_par, ACTIONS, "ACTIONS")){
          ROS_ERROR("Could not parse traj_par.");
        }
		if(!parseParameter(traj_par, TYPE, "TYPE")){
          ROS_ERROR("Could not parse traj_par."); 
        }
      }
      if(n_act == N_ACTION){
        yaml = 0;
        n_act = 0;
      }else{
        tf = ACTIONS(n_act,0);
        pos_f(0) = ACTIONS(n_act,1);
        pos_f(1) = ACTIONS(n_act,2);
        pos_f(2) = ACTIONS(n_act,3);
        choice = 1;
        n_act++;
      }
    }

    ros::spinOnce();

    t_init = ros::Time::now();
    // pos_init = pos;
    if(yaml==0){
      pos_init = pos_d;
    }else{
      if (n_act==1){
        pos_init = pos_d;
      }
      else{
        pos_init(0) = ACTIONS(n_act-2,1);
        pos_init(1) = ACTIONS(n_act-2,2);
        pos_init(2) = ACTIONS(n_act-2,3);
      }
    }
    pos_init << pos_d;
    or_init << or_d; 

    t = (ros::Time::now() - t_init).toSec();

    while (t <= tf)
    {
      if (choice == 1){
        interpolator_pos(pos_init, pos_f, tf, t);
      } else if (choice == 2){
        if (demo == 1){
          demo_inf_XY(pos_init, t);
        }else if(demo==2){
          demo_inf_XYZ(pos_init,t,zf,tf);
        }else if (demo==3){
          demo_circle_xy(pos_init,t,zf,tf);
        }
      }

      traj_msg.header.stamp = ros::Time::now();
      traj_msg.pose.position.x = traj_t.p_des.x();
      traj_msg.pose.position.y = traj_t.p_des.y();
      traj_msg.pose.position.z = traj_t.p_des.z();
      traj_msg.pose.orientation.w = or_d.w(); 
      traj_msg.pose.orientation.x = or_d.x(); 
      traj_msg.pose.orientation.y = or_d.y(); 
      traj_msg.pose.orientation.z = or_d.z(); 
     
      traj_msg.velocity.position.x = traj_t.v_des.x();
      traj_msg.velocity.position.y = traj_t.v_des.y();
      traj_msg.velocity.position.z = traj_t.v_des.z();
      traj_msg.velocity.orientation.x = 0;
      traj_msg.velocity.orientation.y = 0;
      traj_msg.velocity.orientation.z = 0;

      traj_msg.acceleration.position.x = traj_t.a_des.x();
      traj_msg.acceleration.position.y = traj_t.a_des.y();
      traj_msg.acceleration.position.z = traj_t.a_des.z();
      traj_msg.acceleration.orientation.x = 0;
      traj_msg.acceleration.orientation.y = 0;
      traj_msg.acceleration.orientation.z = 0;
 
      for (int i=0; i<8; i++){
         traj_msg.pose_d[i] = trajdq.x_des(i);
         traj_msg.dpose_d[i] = trajdq.dx_des(i);
         traj_msg.ddpose_d[i] = trajdq.ddx_des(i);
       }

      x_des_dq = DQ(trajdq.x_des);
      x_des_dq = x_des_dq.normalize();

      pos_check << vec3(x_des_dq.translation()); 

      or_check << vec4(x_des_dq.P()); 

      traj_msg.position_d[0] = pos_check(0);
      traj_msg.position_d[1] = pos_check(1);
      traj_msg.position_d[2] = pos_check(2);

 
      pub_cmd.publish(traj_msg);

      loop_rate.sleep();

      t = (ros::Time::now() - t_init).toSec();
    }
  }
  return 0;
}
