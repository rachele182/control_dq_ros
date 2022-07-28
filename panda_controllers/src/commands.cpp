#include <iostream>

#include <unistd.h>
#include <cstdlib>
#include <signal.h>
#include <cmath>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "dqrobotics/DQ.h"
#include "dqrobotics/utils/DQ_Constants.h"
#include "dqrobotics/utils/DQ_LinearAlgebra.h"
#include <panda_controllers/DesiredProjectTrajectory.h>
#include "ros/ros.h"
#include <sstream>

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

//external traj
nav_msgs::Path ext_traj;
int ext_traj_len = 40;

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

int demo_int_traj (Vector3d pos_i, double time) {
  Vector3d tmp;
  double t_f;
  double t;  
  double z_int;
  int phase; 
  phase = 0; 
  // z_int = 0.035; 
  z_int = 0.13; 
  Vector3d pos_f;
  if(time>=0 && time<10){
    tmp << pos_i; 
    pos_f << pos_i(0), pos_i(1), z_int; 
    t_f = 10; 
    t = time;
  }else if (time>=10 && time<12){
    tmp << pos_i(0), pos_i(1), z_int; 
    pos_f << tmp;
    t_f = 2; 
    t = time - 10; 
  }else if (time>=12 && time<22){
    tmp << pos_i(0), pos_i(1), z_int; 
    pos_f << pos_i(0), pos_i(1), 0.2; 
    t_f = 10;
    t = time - 12; 
    phase = 1; 
  }else {
    tmp << pos_i(0), pos_i(1), 0.2; 
    pos_f << tmp;
    t_f = 1000; 
    t = time - 22;  
    phase = 1; 
  }
   traj_t.p_des << tmp + (tmp - pos_f)*(15*pow((t/t_f),4) - 6*pow((t/t_f),5) -10*pow((t/t_f),3));
   traj_t.v_des << (tmp - pos_f)*(60*(pow(t,3)/pow(t_f,4)) - 30*(pow(t,4)/pow(t_f,5)) -30*(pow(t,2)/pow(t_f,3)));
   traj_t.a_des << (tmp - pos_f)*(180*(pow(t,2)/pow(t_f,4)) - 120*(pow(t,3)/pow(t_f,5)) -60*(t/pow(t_f,3)));

  return phase; 
}

// Minimum jerk interpolation

void interpolator_pos( Vector3d pos_i, Vector3d pos_f,
                              double tf, double t) {

        traj_t.p_des << pos_i + (pos_i - pos_f)*(15*pow((t/tf),4) - 6*pow((t/tf),5) -10*pow((t/tf),3));
        traj_t.v_des << (pos_i - pos_f)*(60*(pow(t,3)/pow(tf,4)) - 30*(pow(t,4)/pow(tf,5)) -30*(pow(t,2)/pow(tf,3)));
        traj_t.a_des << (pos_i - pos_f)*(180*(pow(t,2)/pow(tf,4)) - 120*(pow(t,3)/pow(tf,5)) -60*(t/pow(tf,3)));
  }

void dummy(Vector3d pos_i,double tf){
        traj_t.p_des << pos_i;
        traj_t.v_des << 0,0,0;
        traj_t.a_des << 0,0,0; 
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

  ros::init(argc, argv, "commands");

  ros::NodeHandle node_handle;

  ros::Publisher pub_cmd = node_handle.advertise<panda_controllers::DesiredProjectTrajectory>("/motion_control_dq/dq_trajectory", 1000);

  ros::Subscriber sub_cmd =  node_handle.subscribe("/motion_control_dq/franka_ee_pose", 1, 
                                                &poseCallback);

  
  ros::Rate loop_rate(100);

  panda_controllers::DesiredProjectTrajectory traj_msg;

  signal(SIGINT, signal_callback_handler);
  
  Vector3d pos_f;
  Vector3d pos_init; 
  Vector4d or_init; 
  double tf;
  double t_reset=10;
  double zf;
  ros::Time t_init;
  double t = 0;
  int choice;
  int ph; 
  int demo = -1;
  int ext = -1;
  double dt = 0.1;

  
  while (ros::ok())
  {
    cout<<"choice:   (1:jerk_trajectory, 2:demos, 3:dummy) "<<endl;
    cin>>choice;

     if (choice == 1){
      cout<<"insert pos_f (x y z)"<<endl;
      cin>>pos_f(0);
      cin>>pos_f(1);
      cin>>pos_f(2);
      cout<<"insert time_f: "<<endl;
      cin>>tf;

    }else if (choice == 2){
      cout<<"select demo:  (1: infinite XY , 2: infinite XYZ , 3: circle_xy , 4:int_traj"<<endl;
      cin>>demo;
      if(demo ==4){
         tf = 30; 
         cout<<"int traj loaded: "<<endl;
      }else{
        cout<<"insert time_f: "<<endl;
        cin>>tf;
      }  
      if ((demo==2) || (demo==3)){
        cout<<"insert zf: "<<endl;
        cin>>zf;
      }
    }else if (choice == 3){

       cout<<"insert time_f: "<<endl;
       cin>>tf;
    }

    ros::spinOnce();
    
    pos_init << pos_d;
    or_init << or_d; 


    t_init = ros::Time::now();
    t = (ros::Time::now() - t_init).toSec();

    while (t <= tf){
       if (choice == 1){

        or_dq = DQ(or_init); 
        
        interpolator_pos(pos_init, pos_f, tf, t);

        pos_des << traj_t.p_des;
        vel_des << traj_t.v_des;
        acc_des << traj_t.a_des; 

        gen_traj_dq(pos_des,vel_des,acc_des,or_dq); 

        }else if (choice == 2){

          or_dq = DQ(or_init);

          if (demo == 1){
          demo_inf_XY(pos_init, t);
          }else if(demo==2){
          demo_inf_XYZ(pos_init,t,zf,tf);
          }else if (demo==3){
          demo_circle_xy(pos_init,t,zf,tf);
          }else if (demo==4){
          ph = demo_int_traj(pos_init,t);
          }

          pos_des << traj_t.p_des;
          vel_des << traj_t.v_des;
          acc_des << traj_t.a_des; 
          gen_traj_dq(pos_des,vel_des,acc_des,or_dq);

        }else if (choice == 3){
          or_dq = DQ(or_init);
          dummy(pos_init,tf); 

          pos_des << traj_t.p_des;
          vel_des << traj_t.v_des;
          acc_des << traj_t.a_des; 
          gen_traj_dq(pos_des,vel_des,acc_des,or_dq);

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

      if(demo==4){
         traj_msg.phase = ph; 
      }
     
 
     pub_cmd.publish(traj_msg);
 
     loop_rate.sleep();

     t = (ros::Time::now() - t_init).toSec();

  } 
  }
  return 0;
}
