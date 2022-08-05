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
#include <panda_controllers/InfoDebug.h>
#include "ros/ros.h"
#include <sstream>

//-----------------------DEFINE--------------------------//
typedef Matrix<double, 8, 1> Vector8d;

using namespace std; using namespace DQ_robotics; using namespace panda_controllers; using DQ_robotics::E_; 

////////////==============SINGLE ARM ======================////
//initial position and orientation from Franka
Vector3d pos_d;
Vector4d or_d; 
// desired trajectory 
Vector3d pos_des,pos_check,vel_des,acc_des;
Vector4d or_check;  
// DQ trajectory
DQ pos_des_dq, vel_des_dq, acc_des_dq;
DQ x_des_dq,dx_des_dq,ddx_des_dq; 
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

////// =====================DUAL ARM=======================//
DQ rot_1, rot_2; 
Vector8d pose_abs,pose_rel,pose_1,pose_2;  
Vector8d x1_des,dx1_des,ddx1_des,x2_des,dx2_des,ddx2_des;
Vector8d xa_des,dxa_des; 
Vector3d pos1,pos2,pos_1_des,pos_2_des,vel_1_des,vel_2_des,acc_1_des,acc_2_des; 
DQ p1_d,v1_d,a1_d,p2_d,v2_d,a2_d,x1_des_dq,dx1_des_dq,ddx1_des_dq,x2_des_dq,dx2_des_dq,ddx2_des_dq;

struct traj_dual_struct{
      Vector3d p1_des;
      Vector3d v1_des;
      Vector3d a1_des;
      Vector3d p2_des;
      Vector3d v2_des;
      Vector3d a2_des;
  } traj_dual;

struct traj_dual_dq_struct{
      Vector8d x1_des;
      Vector8d dx1_des;
      Vector8d ddx1_des;
      Vector8d x2_des;
      Vector8d dx2_des;
      Vector8d ddx2_des;
      Vector8d xa_des;
      Vector8d dxa_des;
      Vector8d ddxa_des;
      Vector8d xr_des;
      Vector8d dxr_des;
      Vector8d ddxr_des;
  } traj_dual_dq;  

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
   cout << "Caught signal " << signum << endl;
   // Terminate program
   exit(signum);
}

// =============================== CALLBACKS ====================================///

// Call back for robot pose
void poseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) { 
  pos_d << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  or_d << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z; //w real part  
}

// Dual arm callbacks

void CoopPoseCallback(
    const panda_controllers::InfoDebugConstPtr& msg) {

  pose_abs << msg->abs_pose[0], msg->abs_pose[1], msg->abs_pose[2],msg->abs_pose[3],msg->abs_pose[4],msg->abs_pose[5],msg->abs_pose[6],msg->abs_pose[7];
  pose_rel << msg->rel_pose[0], msg->rel_pose[1], msg->rel_pose[2],msg->rel_pose[3],msg->rel_pose[4],msg->rel_pose[5],msg->rel_pose[6],msg->rel_pose[7];
  pose_1 << msg->x1[0], msg->x1[1], msg->x1[2],msg->x1[3],msg->x1[4],msg->x1[5],msg->x1[6],msg->x1[7];
  pose_2 << msg->x2[0], msg->x2[1], msg->x2[2],msg->x2[3],msg->x2[4],msg->x2[5],msg->x2[6],msg->x2[7];
  pos1 << msg->pos_1[0], msg->pos_1[1],msg->pos_1[2];
  pos2 << msg->pos_2[0], msg->pos_2[1],msg->pos_2[2];
  }

// ================================ DEMOS ========================================//

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
  z_int = 0.14; 
  Vector3d pos_f;
  if(time>=0 && time<20){
    tmp << pos_i; 
    pos_f << pos_i(0), pos_i(1), z_int; 
    t_f = 20; 
    t = time;
  }else if (time>=20 && time<21){
    tmp << pos_i(0), pos_i(1), z_int; 
    pos_f << tmp;
    t_f = 1; 
    t = time - 20; 
  }else if (time>=21 && time<31){
    tmp << pos_i(0), pos_i(1), z_int; 
    pos_f << pos_i(0), pos_i(1), 0.3; 
    t_f = 10;
    t = time - 21; 
    phase = 1; 
  }else {
    tmp << pos_i(0), pos_i(1), 0.3; 
    pos_f << tmp;
    t_f = 1000; 
    t = time - 31;  
    phase = 1; 
  }
   traj_t.p_des << tmp + (tmp - pos_f)*(15*pow((t/t_f),4) - 6*pow((t/t_f),5) -10*pow((t/t_f),3));
   traj_t.v_des << (tmp - pos_f)*(60*(pow(t,3)/pow(t_f,4)) - 30*(pow(t,4)/pow(t_f,5)) -30*(pow(t,2)/pow(t_f,3)));
   traj_t.a_des << (tmp - pos_f)*(180*(pow(t,2)/pow(t_f,4)) - 120*(pow(t,3)/pow(t_f,5)) -60*(t/pow(t_f,3)));

  return phase; 
}

Vector3d demo_int_traj_2 (Vector3d pos_i, double time) {
  Vector3d tmp;
  double t_f;
  double t;  
  double z_int;
  Vector3d phase; 
  phase.setZero();  
  z_int = 0.14; 
  Vector3d pos_f;

  if(time>=0 && time<15){ //go down 
    tmp << pos_i; 
    pos_f << pos_i(0), pos_i(1), z_int+0.05; 
    t_f = 15; 
    t = time;
  }else if (time>=15 && time<20){ //start interaction phase
    tmp << pos_i(0), pos_i(1), z_int+0.05; 
    pos_f <<  pos_i(0), pos_i(1), z_int; 
    t_f = 5; 
    t = time - 15; 
    phase(2) = 1; 
  }else if (time>=20 && time<21){ //pause
    tmp << pos_i(0), pos_i(1), z_int; 
    pos_f << tmp; 
    t_f = 1;
    t = time - 20; 
    phase(2) = 1; 
  }else if (time>=21 && time<31){ //go up
    tmp << pos_i(0), pos_i(1), z_int; 
    pos_f << pos_i(0), pos_i(1), 0.3;
    t_f = 10;
    t = time - 21; 
    phase(2) = 2; 
  }else {
    tmp << pos_i(0), pos_i(1), 0.3; 
    pos_f << tmp;
    t_f = 1000; 
    t = time - 31;  
    phase(2) = 2; 
  }
   traj_t.p_des << tmp + (tmp - pos_f)*(15*pow((t/t_f),4) - 6*pow((t/t_f),5) -10*pow((t/t_f),3));
   traj_t.v_des << (tmp - pos_f)*(60*(pow(t,3)/pow(t_f,4)) - 30*(pow(t,4)/pow(t_f,5)) -30*(pow(t,2)/pow(t_f,3)));
   traj_t.a_des << (tmp - pos_f)*(180*(pow(t,2)/pow(t_f,4)) - 120*(pow(t,3)/pow(t_f,5)) -60*(t/pow(t_f,3)));

  return phase; 
}

// ============================= FUNCTIONS ================================== //

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

void abs_traj (Vector3d pos_i, double time) {
  Vector3d tmp,pos_f; 
  double t_f,t; 

  if(time>=0 && time<10){ //go down 10 cm
    tmp << pos_i; 
    pos_f << pos_i(0), pos_i(1), pos_i(2)-0.2; 
    t_f = 10; 
    t = time;
  }else if (time>=10 && time<12){ //pause
    tmp << pos_i(0), pos_i(1), pos_i(2)-0.2; 
    pos_f << tmp; 
    t_f = 2;
    t = time - 10; 
  }else if (time>=12 && time<22){ //go up
    tmp << pos_i(0), pos_i(1), pos_i(2)-0.2; 
    pos_f << pos_i(0), pos_i(1), pos_i(2);
    t_f = 10;
    t = time - 12; 
  }else {
    tmp << pos_i(0), pos_i(1), pos_i(2); 
    pos_f << tmp;
    t_f = 1000; 
    t = time - 22; 
  }
   traj_t.p_des << tmp + (tmp - pos_f)*(15*pow((t/t_f),4) - 6*pow((t/t_f),5) -10*pow((t/t_f),3));
   traj_t.v_des << (tmp - pos_f)*(60*(pow(t,3)/pow(t_f,4)) - 30*(pow(t,4)/pow(t_f,5)) -30*(pow(t,2)/pow(t_f,3)));
   traj_t.a_des << (tmp - pos_f)*(180*(pow(t,2)/pow(t_f,4)) - 120*(pow(t,3)/pow(t_f,5)) -60*(t/pow(t_f,3)));
}

void dual_traj (Vector3d pos_in_1, Vector3d pos_in_2, double time) {
  Vector3d tmp1,tmp2;
  double t_f, t;  
  Vector3d pos_1_f, pos_2_f; 

  if(time>=0 && time<5){ //go down 10 cm
    tmp1 << pos_in_1; 
    tmp2 << pos_in_2;
    pos_1_f << tmp1(0), tmp1(1), tmp1(2)-0.1; 
    pos_2_f << tmp2(0), tmp2(1), tmp2(2)-0.1; 
    t_f = 5; 
    t = time;
  }else if (time>=5 && time<7){ //pause
    tmp1 << pos_in_1(0), pos_in_1(1), pos_in_1(2)-0.1; 
    tmp2 << pos_in_2(0), pos_in_2(1), pos_in_2(2)-0.1; 
    pos_1_f << tmp1; 
    pos_2_f << tmp2;
    t_f = 2;
    t = time - 5; 
  }else if (time>=7 && time<12){ //come closer
    tmp1 << pos_in_1(0), pos_in_1(1), pos_in_1(2)-0.1; 
    tmp2 << pos_in_2(0), pos_in_2(1), pos_in_2(2)-0.1; 
    pos_1_f << pos_in_1(0), pos_in_1(1)-0.15, pos_in_1(2)-0.1; 
    pos_2_f << pos_in_2(0), pos_in_2(1)+ 0.15, pos_in_2(2)-0.1;
    t_f = 5;
    t = time - 7; 
  }else if (time>=12 && time<14){ //pause
    tmp1 << pos_in_1(0), pos_in_1(1)-0.15, pos_in_1(2)-0.1; 
    tmp2 << pos_in_2(0), pos_in_2(1)+0.15, pos_in_2(2)-0.1; 
    pos_1_f << tmp1; 
    pos_2_f << tmp2;
    t_f = 2;
    t = time - 12; 
  }else if (time>=14 && time<19){ //go back again
    tmp1 << pos_in_1(0), pos_in_1(1)-0.15, pos_in_1(2)-0.1; 
    tmp2 << pos_in_2(0), pos_in_2(1)+0.15, pos_in_2(2)-0.1;
    pos_1_f << tmp1(0), tmp1(1)+0.15, tmp1(2); 
    pos_2_f << tmp2(0), tmp2(1)-0.15, tmp2(2);
    t_f = 5;
    t = time - 14; 
  }else if (time>=19 && time<24){ //go up again
    tmp1 << pos_in_1(0), pos_in_1(1), pos_in_1(2)-0.1; 
    tmp2 << pos_in_2(0), pos_in_2(1), pos_in_2(2)-0.1;
    pos_1_f << tmp1(0), tmp1(1), tmp1(2)+0.1; 
    pos_2_f << tmp2(0), tmp2(1), tmp2(2)+0.1;
    t_f = 5; 
    t = time - 19; 
  }else { // eq phase
    tmp1 << pos_in_1(0), pos_in_1(1), pos_in_1(2); 
    tmp2 << pos_in_2(0), pos_in_2(1), pos_in_2(2); 
    pos_1_f << tmp1; 
    pos_2_f << tmp2;
    t_f = 1000; 
    t = time - 24; 
  }
   //des traj for left arm
   traj_dual.p1_des << tmp1 + (tmp1 - pos_1_f)*(15*pow((t/t_f),4) - 6*pow((t/t_f),5) -10*pow((t/t_f),3));
   traj_dual.v1_des << (tmp1 - pos_1_f)*(60*(pow(t,3)/pow(t_f,4)) - 30*(pow(t,4)/pow(t_f,5)) -30*(pow(t,2)/pow(t_f,3)));
   traj_dual.a1_des << (tmp1 - pos_1_f)*(180*(pow(t,2)/pow(t_f,4)) - 120*(pow(t,3)/pow(t_f,5)) -60*(t/pow(t_f,3)));
   
   //des traj for right arm
   traj_dual.p2_des << tmp2 + (tmp2 - pos_2_f)*(15*pow((t/t_f),4) - 6*pow((t/t_f),5) -10*pow((t/t_f),3));
   traj_dual.v2_des << (tmp2 - pos_2_f)*(60*(pow(t,3)/pow(t_f,4)) - 30*(pow(t,4)/pow(t_f,5)) -30*(pow(t,2)/pow(t_f,3)));
   traj_dual.a2_des << (tmp2 - pos_2_f)*(180*(pow(t,2)/pow(t_f,4)) - 120*(pow(t,3)/pow(t_f,5)) -60*(t/pow(t_f,3)));
}

// ========================= COMPUTE DQ TRAJ ========================//
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

// ================= COMPUTE DQ TRAJECTORIES FOR EACH ARM ====================///

  void gen_each_arm_traj_dq ( Vector3d pos1_des, Vector3d vel1_des,
                       Vector3d acc1_des, DQ or1_dq , Vector3d pos2_des, Vector3d vel2_des,
                       Vector3d acc2_des, DQ or2_dq
                              ){  
        //poses
        p1_d = DQ(pos1_des);       
        x1_des_dq = or1_dq + 0.5*E_*(p1_d*or1_dq); 
        x1_des_dq = x1_des_dq * x1_des_dq.inv().norm();
        traj_dual_dq.x1_des << vec8(x1_des_dq); 

        p2_d = DQ(pos2_des);       
        x2_des_dq = or2_dq + 0.5*E_*(p2_d*or2_dq); 
        x2_des_dq = x2_des_dq * x2_des_dq.inv().norm();
        traj_dual_dq.x2_des << vec8(x2_des_dq);

        //velocities
        v1_d = DQ(vel1_des);
        dx1_des_dq = 0.5*E_*(v1_d*or1_dq); 
        traj_dual_dq.dx1_des << vec8(dx1_des_dq); 

        v2_d = DQ(vel2_des);
        dx2_des_dq = 0.5*E_*(v2_d*or2_dq); 
        traj_dual_dq.dx2_des << vec8(dx2_des_dq); 

        //accelerations
        a1_d = DQ(acc1_des);
        ddx1_des_dq = 0.5*E_*(a1_d*or1_dq); 
        traj_dual_dq.ddx1_des << vec8(ddx1_des_dq); 

        a2_d = DQ(acc2_des);
        ddx2_des_dq = 0.5*E_*(a2_d*or2_dq); 
        traj_dual_dq.ddx2_des << vec8(ddx2_des_dq); 

  }

// ====================COMPUTE CDTS TRAJECTORIES FROM DESIRED ARMS TRAJ ===//
//From des position and orientation (fixed) for each arm computes cdts desired trajectories

  void gen_dual_traj_dq (Vector8d x1_des, Vector8d dx1_des,
                        Vector8d ddx1_des, Vector8d x2_des, Vector8d dx2_des,
                        Vector8d ddx2_des, Vector8d xa_old, Vector8d dxa_old,double period,
                              int count){  
    DQ x1_des_dq, x2_des_dq,dx1_des_dq,dx2_des_dq,ddx1_des_dq,ddx2_des_dq; 
    x1_des_dq = DQ(x1_des); dx1_des_dq = DQ(dx1_des); ddx1_des_dq = DQ(ddx1_des); 
    x2_des_dq = DQ(x2_des); dx2_des_dq = DQ(dx2_des); ddx2_des_dq = DQ(ddx2_des);
    //Compute desired relative pose
    traj_dual_dq.xr_des = vec8((x2_des_dq).conj()*x1_des_dq);
    traj_dual_dq.dxr_des = vec8((dx2_des_dq).conj()*x1_des_dq + (x2_des_dq).conj()*dx1_des_dq);
    traj_dual_dq.ddxr_des = vec8((ddx2_des_dq).conj()*x1_des_dq + 2*((dx2_des_dq).conj()*dx1_des_dq) + (x2_des_dq).conj()*ddx1_des_dq); 
    //Compute desired absolute pose
    DQ xr_des_dq;  xr_des_dq = DQ(traj_dual_dq.xr_des); 

    traj_dual_dq.xa_des = vec8(x2_des_dq*pow(xr_des_dq,0.5));
    if(count==0){
      traj_dual_dq.dxa_des.setZero();
      traj_dual_dq.ddxa_des.setZero();
    }else {
      traj_dual_dq.dxa_des = (traj_dual_dq.xa_des - xa_old)/period;
      traj_dual_dq.ddxa_des = (traj_dual_dq.dxa_des - dxa_old)/period;
    }
  }

// ==================

int main(int argc, char **argv)
{

  using namespace panda_controllers; 

  ros::init(argc, argv, "commands");

  ros::NodeHandle node_handle;

  ros::Publisher pub_cmd = node_handle.advertise<panda_controllers::DesiredProjectTrajectory>("/motion_control_dq/dq_trajectory", 1000);

  ros::Subscriber sub_cmd =  node_handle.subscribe("/motion_control_dq/franka_ee_pose", 1, 
                                                &poseCallback);

  ros::Subscriber sub_coop_var =  node_handle.subscribe("/motion_control_dq/info_debug", 1, 
                                                &CoopPoseCallback);
  ros::Rate loop_rate(100);

  panda_controllers::DesiredProjectTrajectory traj_msg;

  signal(SIGINT, signal_callback_handler);
  
  //===== VARIABLEAS ======== //
  Vector3d pos_f,pos_init,phase;
  Vector4d or_init; 
  //dual arm
  DQ pose_1_dq, pose_2_dq;
  Vector3d pos_in_1, pos_in_2; 
  Vector4d or_1, or_2; 
  Vector8d pose_abs_in; 
  double tf,zf,Ts;
  ros::Time t_init; 
  Ts = 0.01; //sampling time node 
  double t = 0;
  int choice,ph; 
  int dual = 0; 
  int demo = -1;
  int count = 0;
  
  while (ros::ok())
  {
    cout<<"choice:   (1:jerk_trajectory, 2:demos, 3:dummy, 4:abs_traj_z_axis, 5:dual_traj) "<<endl;
    cin>>choice;

    if(choice == 1){
      cout<<"insert pos_f (x y z)"<<endl;
      cin>>pos_f(0); 
      cin>>pos_f(1);
      cin>>pos_f(2);
      cout<<"insert time_f: "<<endl;
      cin>>tf;

    }else if (choice == 2){
      cout<<"select demo:  (1: infinite XY , 2: infinite XYZ , 3: circle_xy , 4:int_traj, 5_int_traj_2"<<endl;
      cin>>demo;
      if(demo ==4 || demo ==5){
         tf = 40; 
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
      cout << "select single or arm control: (0: single, 1:dual)" << endl; 
      cin >> dual; 
      cout<<"insert time_f: "<<endl;
      cin>>tf;   
    }else if (choice == 4){
      tf = 30; 
      cout<< "loading traj for absolute pose.."<<endl; 
    }else if(choice == 5){
      tf = 40;
      cout<< "loading dual trajectory..."<< endl; 
    }

    ros::spinOnce();
    pos_init << pos_d;
    or_init << or_d; 

    //dual arm
    pose_1_dq = DQ(pose_1); pose_2_dq = DQ(pose_2); pose_1_dq = pose_1_dq.normalize(); pose_2_dq = pose_2_dq.normalize(); 
    pos_in_1 << pos1; pos_in_2 << pos2;
    or_1 = vec4(pose_1_dq.rotation()); or_2 = vec4(pose_2_dq.rotation()); 
    pose_abs_in << pose_abs; 

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
          }else if (demo==5){
          phase = demo_int_traj_2(pos_init,t);  
          }

          pos_des << traj_t.p_des;
          vel_des << traj_t.v_des;
          acc_des << traj_t.a_des; 
          gen_traj_dq(pos_des,vel_des,acc_des,or_dq);

        }else if (choice == 3){
          if(dual==0){
            or_dq = DQ(or_init);
            dummy(pos_init,tf); 
          }else if (dual==1){
            DQ pose_abs_dq; pose_abs_dq = DQ(pose_abs); pose_abs_dq = pose_abs_dq.normalize(); 
            Vector4d rot_a;
            rot_a << vec4(pose_abs_dq.rotation());   
            pos_init << vec3(pose_abs_dq.translation());
            or_dq = DQ(rot_a);
            dummy(pos_init,tf);  
          }
          
          pos_des << traj_t.p_des;
          vel_des << traj_t.v_des;
          acc_des << traj_t.a_des; 
          gen_traj_dq(pos_des,vel_des,acc_des,or_dq);

        }else if (choice == 4){
          DQ pose_abs_dq; Vector4d rot_a;
          pose_abs_dq = DQ(pose_abs); pose_abs_dq = pose_abs_dq.normalize(); 
          rot_a << vec4(pose_abs_dq.rotation());  
          pos_init << vec3(pose_abs_dq.translation());
          or_dq = DQ(rot_a); 
          abs_traj(pos_init,t); 
          pos_des << traj_t.p_des;
          vel_des << traj_t.v_des;
          acc_des << traj_t.a_des; 
          gen_traj_dq(pos_des,vel_des,acc_des,or_dq);

        }else if (choice == 5){
          rot_1 = DQ(or_1); rot_2 = DQ(or_2); 
          dual_traj(pos_in_1,pos_in_2,t); // compute EE traj for each arm
          pos_1_des << traj_dual.p1_des; vel_1_des << traj_dual.v1_des; acc_1_des << traj_dual.a1_des;
          pos_2_des << traj_dual.p2_des; vel_2_des << traj_dual.v2_des; acc_2_des << traj_dual.a2_des;

          gen_each_arm_traj_dq(pos_1_des,vel_1_des,acc_1_des,rot_1,
                                pos_2_des,vel_2_des,acc_2_des,rot_2); //computes DQ traj for each arm
        
          x1_des << traj_dual_dq.x1_des;  dx1_des << traj_dual_dq.dx1_des; ddx1_des << traj_dual_dq.ddx1_des;
          x2_des << traj_dual_dq.x2_des;  dx1_des << traj_dual_dq.dx2_des; ddx2_des << traj_dual_dq.ddx2_des;  
          //update to compute derivative
          if(count==0){
            xa_des << pose_abs_in; 
            dxa_des << 0,0,0,0,0,0,0,0; 
          }else{
            xa_des << traj_dual_dq.xa_des;
            dxa_des << traj_dual_dq.dxa_des; 
          }
          gen_dual_traj_dq(x1_des,dx1_des,ddx1_des,
                            x2_des,dx2_des,ddx2_des,xa_des,dxa_des,
                                   Ts,count); //computes des abs and relative trajs
        }
        
        ///===========================PUBLISHING =========================///

        if(choice==3 || choice==4) {
          // publish also des relative pose
          DQ pose_rel_dq;
          pose_rel_dq = DQ(pose_rel); 
          pose_rel_dq = pose_rel_dq.normalize(); 
          Vector3d pos_r;
          pos_r = vec3(pose_rel_dq.translation());
          for (int i=0; i<8; i++){
           traj_msg.pose_r[i] = pose_rel(i);
           traj_msg.dpose_r[i] = 0;
           traj_msg.ddpose_r[i] = 0;
         }
          for (int i=0; i<3; i++){
          traj_msg.position_r[i] = pos_r(i);
         }

        }

        if(choice==1 || choice ==2 || choice==3 || choice==4){
          traj_msg.header.stamp = ros::Time::now();

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

        }else if(choice==5){
          traj_msg.header.stamp = ros::Time::now();

          DQ xa_dq,xr_dq,x1_dq,x2_dq;
          Vector3d pa,pr,p1,p2; 
          xa_dq = DQ(traj_dual_dq.xa_des); xa_dq = xa_dq.normalize(); 
          xr_dq = DQ(traj_dual_dq.xr_des); xr_dq = xr_dq.normalize(); 
          x1_dq = DQ(traj_dual_dq.x1_des); x1_dq = x1_dq.normalize(); 
          x2_dq = DQ(traj_dual_dq.x2_des); x2_dq = x2_dq.normalize(); 
          pa = vec3(xa_dq.translation()); pr = vec3(xr_dq.translation()); 
          p1 = vec3(x1_dq.translation()); p2 = vec3(x2_dq.translation());

          for (int i=0; i<8; i++){
              traj_msg.pose_d[i] = traj_dual_dq.xa_des(i);
              traj_msg.dpose_d[i] = traj_dual_dq.dxa_des(i);
              traj_msg.ddpose_d[i] = traj_dual_dq.ddxa_des(i);
              traj_msg.pose_r[i] = traj_dual_dq.xr_des(i);
              traj_msg.dpose_r[i] = traj_dual_dq.dxr_des(i);
              traj_msg.ddpose_r[i] = traj_dual_dq.ddxr_des(i);
          }
          for (int i=0; i<3; i++){
                traj_msg.position_r[i] = pr(i);
                traj_msg.position_d[i] = pa(i);
                traj_msg.position_1[i] = p1(i);
                traj_msg.position_2[i] = p2(i);
              }  
        }  

        if(demo==4){
          traj_msg.phase = ph; 
        }

        if(demo==5){
           traj_msg.phase_array[0] = phase(0); 
           traj_msg.phase_array[1] = phase(1); 
           traj_msg.phase_array[2] = phase(2); 
        }
    
  
     pub_cmd.publish(traj_msg);
 
     loop_rate.sleep();

     t = (ros::Time::now() - t_init).toSec();  

     count = count+1;   
  } 
  
  }
  return 0;
}
