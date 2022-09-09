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

////// =====================DUAL ARM=======================//
DQ rot_r, rot_a; 
DQ pos_r_des_dq, pos_a_des_dq, vel_r_des_dq, vel_a_des_dq, acc_r_des_dq, acc_a_des_dq; 
Vector8d pose_abs,pose_rel,pose_1,pose_2;  
Vector8d xa_des,dxa_des,ddxa_des,xr_des,dxr_des,ddxr_des; 
Vector3d pos_r_des,vel_r_des,acc_r_des,pos_a_des,vel_a_des,acc_a_des; 

//==== Recorded poses ====== //
DQ x_abs_rec_dq, x_rel_rec_dq, rot_a_rec_dq, rot_r_rec_dq; 
Vector8d x_abs_rec, x_rel_rec; 
Vector3d pos_abs_rec, pos_rel_rec; 
x_abs_rec << -0.0258561, 0.913526, 0.404352, -0.0360581, -0.355649, -0.0730816 ,0.117307 , -0.281012;
x_rel_rec << -0.00924998, -0.690086, -0.723567, 0.0120759, 0.0104255, 0.144312, -0.137745, 0.0013573;  
x_abs_rec_dq = (DQ(x_abs_rec)).normalize(); x_rel_rec_dq = (DQ(x_rel_rec)).normalize(); 
pos_abs_rec = vec3(x_abs_rec_dq.translation()); pos_rel_rec = vec3(x_rel_dq.translation()); 
rot_a_rec_dq = x_abs_rec_dq.rotation(); rot_r_rec_dq = x_rel_dq.rotation(); 

struct traj_dual_struct{
      Vector3d pr_des;
      Vector3d vr_des;
      Vector3d ar_des;
      Vector3d pa_des;
      Vector3d va_des;
      Vector3d a_des; 
  } traj_dual;

struct traj_dual_dq_struct{
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

// =============================== CALLBACKS ====================================//

void CoopPoseCallback(
    const panda_controllers::InfoDebugConstPtr& msg) {

  pose_abs << msg->abs_pose[0], msg->abs_pose[1], msg->abs_pose[2],msg->abs_pose[3],msg->abs_pose[4],msg->abs_pose[5],msg->abs_pose[6],msg->abs_pose[7];
  pose_rel << msg->rel_pose[0], msg->rel_pose[1], msg->rel_pose[2],msg->rel_pose[3],msg->rel_pose[4],msg->rel_pose[5],msg->rel_pose[6],msg->rel_pose[7];
  pose_1 << msg->x1[0], msg->x1[1], msg->x1[2],msg->x1[3],msg->x1[4],msg->x1[5],msg->x1[6],msg->x1[7];
  pose_2 << msg->x2[0], msg->x2[1], msg->x2[2],msg->x2[3],msg->x2[4],msg->x2[5],msg->x2[6],msg->x2[7];
  }

// ============================= FUNCTIONS ================================== //


void dummy(Vector3d posr_i,Vector3d posa_i, double tf){
        traj_dual.pr_des << posr_i; //relative
        traj_dual.pa_des << posa_i; //absolute
        traj_dual.vr_des.setZero();
        traj_dual.va_des.setZero();
        traj_dual.ar_des.setZero();
        traj_dual.a_des.setZero();
}

// ================================ DEMOS ========================================//
Vector3d demo_contact (Vector3d posr_i, Vector3d posa_i, double time) {
  Vector3d tmpr,tmpa;
  Vector3d phase; 
  double t_f, t,pc;  
  Vector3d pos_r_f, pos_a_f; 
  double xr_offset;  // offset on x_axis relative frame
  double zr_offset;  // offset on z_axis relative frame
  zr_offset = 0.7; //m
  xr_offset = 0*0.05; //m
  pc = 0.30; 

  if(time>=0 && time<2){ //initial pause
    tmpr << posr_i; 
    tmpa << posa_i; 
    pos_r_f << tmpr; 
    pos_a_f << tmpa; 
    t_f = 2; 
    t = time;
    phase.setZero(); 
  }else if (time>=2 && time<7){ //approach phase
    tmpr << posr_i(0), posr_i(1), posr_i(2); 
    tmpa << posa_i; 
    pos_r_f << tmpr(0) + xr_offset, tmpr(1), pc; 
    pos_a_f << tmpa;  
    t_f = 5;
    t = time - 2; 
    phase(2) = 1; 
  }else if (time>=7 && time<12){ //pause (squeeze phase)
    tmpr << posr_i(0) + xr_offset, posr_i(1), pc; 
    tmpa << posa_i; 
    pos_r_f << tmpr; 
    pos_a_f << tmpa;  
    t_f = 5;
    t = time - 7; 
    phase(2) = 2; 
  }else if (time>=12 && time<17){ //release
    tmpr << posr_i(0) + xr_offset, posr_i(1), pc; 
    tmpa << posa_i; 
    pos_r_f << posr_i(0) + xr_offset, posr_i(1), posr_i(2); 
    pos_a_f << tmpa; 
    t_f = 5; 
    t = time - 12; 
    phase(2) = 1; 
  }else { // eq phase
    tmpr << posr_i(0) + xr_offset, posr_i(1), posr_i(2); 
    tmpa << posa_i; 
    pos_r_f << tmpr; 
    pos_a_f << tmpa;
    t_f = 1000; 
    t = time - 17; 
    phase(2) = 1; 
  }
     
   //des traj for relative var
   traj_dual.pr_des << tmpr + (tmpr - pos_r_f)*(15*pow((t/t_f),4) - 6*pow((t/t_f),5) -10*pow((t/t_f),3));
   traj_dual.vr_des << (tmpr - pos_r_f)*(60*(pow(t,3)/pow(t_f,4)) - 30*(pow(t,4)/pow(t_f,5)) -30*(pow(t,2)/pow(t_f,3)));
   traj_dual.ar_des << (tmpr - pos_r_f)*(180*(pow(t,2)/pow(t_f,4)) - 120*(pow(t,3)/pow(t_f,5)) -60*(t/pow(t_f,3)));
  
   //des trj for absolute var
   traj_dual.pa_des << tmpa + (tmpa - pos_a_f)*(15*pow((t/t_f),4) - 6*pow((t/t_f),5) -10*pow((t/t_f),3));
   traj_dual.va_des << (tmpa - pos_a_f)*(60*(pow(t,3)/pow(t_f,4)) - 30*(pow(t,4)/pow(t_f,5)) -30*(pow(t,2)/pow(t_f,3)));
   traj_dual.a_des << (tmpa - pos_a_f)*(180*(pow(t,2)/pow(t_f,4)) - 120*(pow(t,3)/pow(t_f,5)) -60*(t/pow(t_f,3)));

   return phase; 
}



Vector3d demo_coop (Vector3d posr_i, Vector3d posa_i, double time) {
  Vector3d tmpr,tmpa;
  Vector3d phase; 
  double t_f, t;  
  Vector3d pos_r_f, pos_a_f; 
  double za_offset; double zr_offset; double pc; 
  za_offset = 0.15; //m 
  zr_offset = 0.72; //m
  pc = 0.30; 

  if(time>=0 && time<2){ //initial pause
    tmpr << posr_i; 
    tmpa << posa_i; 
    pos_r_f << tmpr; 
    pos_a_f << tmpa; 
    t_f = 2; 
    t = time;
    phase.setZero(); 
  }else if (time>=2 && time<7){ //approach phase
    tmpr << posr_i(0), posr_i(1), posr_i(2); 
    tmpa << posa_i; 
    // pos_r_f << tmpr(0), tmpr(1), tmpr(2) - zr_offset; 
    pos_r_f << tmpr(0), tmpr(1), pc; 
    pos_a_f << tmpa;  
    t_f = 5;
    t = time - 2; 
    phase(2) = 1;
  }else if (time>=7 && time<12){ //pause
   // pos_r_f << tmpr(0), tmpr(1), tmpr(2) - zr_offset; 
    tmpr << posr_i(0), posr_i(1), pc; 
    tmpa << posa_i; 
    pos_r_f << tmpr; 
    pos_a_f << tmpa;  
    t_f = 5;
    t = time - 7; 
    phase(2) = 2; 
  }else if (time>=12 && time<17){ //go up
    // tmpr << posr_i(0), posr_i(1), posr_i(2) - zr_offset; 
    tmpr << posr_i(0), posr_i(1), pc; 
    tmpa << posa_i; 
    pos_r_f << tmpr; 
    pos_a_f << tmpa(0), tmpa(1), tmpa(2) + za_offset; 
    t_f = 5;
    t = time - 12; 
    phase(2) = 3; 
  }else if (time>=17 && time<22){ //go down
    // tmpr << posr_i(0), posr_i(1), posr_i(2) - zr_offset; 
    tmpr << posr_i(0), posr_i(1), pc; 
    tmpa << posa_i(0), posa_i(1), posa_i(2) + za_offset; 
    pos_r_f << tmpr; 
    pos_a_f << posa_i; 
    t_f = 5;
    t = time - 17; 
    phase(2) = 3; 
  }else if (time>=22 && time<27){ //release
    // tmpr << posr_i(0), posr_i(1), posr_i(2) - zr_offset; 
    tmpr << posr_i(0), posr_i(1), pc; 
    tmpa << posa_i; 
    pos_r_f << posr_i(0), posr_i(1), posr_i(2); 
    pos_a_f << tmpa; 
    t_f = 5; 
    t = time - 22; 
    phase(2) = 4; 
  }else { // eq phase
    tmpr << posr_i(0), posr_i(1), posr_i(2); 
    tmpa << posa_i; 
    pos_r_f << tmpr; 
    pos_a_f << tmpa;
    t_f = 1000; 
    t = time - 27; 
    phase(2) = 4; 
  }
     
   //des traj for relative var
   traj_dual.pr_des << tmpr + (tmpr - pos_r_f)*(15*pow((t/t_f),4) - 6*pow((t/t_f),5) -10*pow((t/t_f),3));
   traj_dual.vr_des << (tmpr - pos_r_f)*(60*(pow(t,3)/pow(t_f,4)) - 30*(pow(t,4)/pow(t_f,5)) -30*(pow(t,2)/pow(t_f,3)));
   traj_dual.ar_des << (tmpr - pos_r_f)*(180*(pow(t,2)/pow(t_f,4)) - 120*(pow(t,3)/pow(t_f,5)) -60*(t/pow(t_f,3)));
  
   //des trj for absolute var
   traj_dual.pa_des << tmpa + (tmpa - pos_a_f)*(15*pow((t/t_f),4) - 6*pow((t/t_f),5) -10*pow((t/t_f),3));
   traj_dual.va_des << (tmpa - pos_a_f)*(60*(pow(t,3)/pow(t_f,4)) - 30*(pow(t,4)/pow(t_f,5)) -30*(pow(t,2)/pow(t_f,3)));
   traj_dual.a_des << (tmpa - pos_a_f)*(180*(pow(t,2)/pow(t_f,4)) - 120*(pow(t,3)/pow(t_f,5)) -60*(t/pow(t_f,3)));

   return phase; 
}



// ========================= COMPUTE DQ TRAJ ========================//
  void gen_dual_traj_dq (Vector3d posr_des, Vector3d velr_des,
                       Vector3d accr_des, DQ or_dq, Vector3d posa_des, Vector3d vela_des,
                       Vector3d acca_des, DQ ora_dq
                              ){  
        //==DQ traj for relative pose   
        DQ x_des_dq,dx_des_dq,ddx_des_dq;  

        pos_r_des_dq = DQ(posr_des);    
        x_des_dq = or_dq + 0.5*E_*(pos_r_des_dq*or_dq); 
        x_des_dq = x_des_dq * x_des_dq.inv().norm();
        traj_dual_dq.xr_des << vec8(x_des_dq); 

        vel_r_des_dq = DQ(velr_des);
        dx_des_dq = 0.5*E_*(vel_r_des_dq*or_dq); 
        traj_dual_dq.dxr_des << vec8(dx_des_dq); 

        acc_r_des_dq = DQ(accr_des);
        ddx_des_dq = 0.5*E_*(acc_r_des_dq*or_dq); 
        traj_dual_dq.ddxr_des << vec8(ddx_des_dq); 

        //==DQ traj for abs pose
        DQ xa_des_dq,dxa_des_dq,ddxa_des_dq;    

        pos_a_des_dq = DQ(posa_des);   
        xa_des_dq = ora_dq + 0.5*E_*(pos_a_des_dq*ora_dq); 
        xa_des_dq = xa_des_dq * xa_des_dq.inv().norm();
        traj_dual_dq.xa_des << vec8(xa_des_dq); 

        vel_a_des_dq = DQ(vela_des);
        dxa_des_dq = 0.5*E_*(vel_a_des_dq*ora_dq); 
        traj_dual_dq.dxa_des << vec8(dxa_des_dq); 

        acc_a_des_dq = DQ(acca_des);
        ddxa_des_dq = 0.5*E_*(acc_a_des_dq*ora_dq); 
        traj_dual_dq.ddxa_des << vec8(ddxa_des_dq); 
  }


  

// ==================

int main(int argc, char **argv)
{

  using namespace panda_controllers; 

  ros::init(argc, argv, "commands_dual");

  ros::NodeHandle node_handle;

  ros::Publisher pub_cmd = node_handle.advertise<panda_controllers::DesiredProjectTrajectory>("/motion_control_dq/dq_trajectory", 1000);


  ros::Subscriber sub_coop_var =  node_handle.subscribe("/motion_control_dq/info_debug", 1, 
                                                &CoopPoseCallback);
  ros::Rate loop_rate(100);

  panda_controllers::DesiredProjectTrajectory traj_msg;

  signal(SIGINT, signal_callback_handler);
  
//   //===== VARIABLES ======== //
Vector3d pos_r_f,pos_r_in, pos_a_f,pos_a_in;
Vector3d phase;
Vector4d or_r, or_a; 
DQ pose_1_dq, pose_2_dq, pose_a_dq, pose_r_dq;
DQ or_r_dq; DQ or_a_dq; 
Vector8d pose_abs_in,pose_rel_in; 
double tf,zf,Ts;
ros::Time t_init; 
Ts = 0.01; //sampling time node 
double t = 0;
int count = 0;
int choice; 
  
  while (ros::ok())
  {
    cout<<"choice:   (1:dummy, 2:contact, 3:demo) "<<endl;
    cin>>choice;

    if(choice == 1){
      cout<<"insert time_f: "<<endl;
      cin>>tf;   
    }else if (choice == 2){
      tf = 25; 
      cout <<"contact_demo_loaded.." << endl; 
    }else if(choice == 3){
      tf = 40;
      cout<< "demo_box.."<<endl; 
    }

    ros::spinOnce();

    //initialize cdts variables
    pose_a_dq = DQ(pose_abs); pose_a_dq = pose_a_dq.normalize(); or_a = vec4(pose_a_dq.rotation()); 
    pose_r_dq = DQ(pose_rel); pose_r_dq = pose_r_dq.normalize(); or_r = vec4(pose_r_dq.rotation()); 
    pose_abs_in << pose_abs; pos_a_in = vec3(pose_a_dq.translation()); 
    pose_rel_in << pose_rel; pos_r_in = vec3(pose_r_dq.translation()); 
  
    t_init = ros::Time::now();
    t = (ros::Time::now() - t_init).toSec();

  while (t <= tf){
       if (choice == 1){
        or_r_dq = DQ(or_r);
        or_a_dq = DQ(or_a); 
        dummy(pos_r_in,pos_a_in, tf); 
        pos_r_des << traj_dual.pr_des; vel_r_des << traj_dual.vr_des; acc_r_des << traj_dual.ar_des; 
        pos_a_des << traj_dual.pa_des; vel_a_des << traj_dual.va_des; acc_a_des << traj_dual.a_des; 
        gen_dual_traj_dq(pos_r_des,vel_r_des,acc_r_des,or_r_dq, pos_a_des,vel_a_des,acc_a_des, or_a_dq); 

      }else if (choice == 2){
        
        or_r_dq = DQ(or_r);
        or_a_dq = DQ(or_a);
        phase = demo_contact(pos_r_in, pos_a_in, t); 
        pos_r_des << traj_dual.pr_des; vel_r_des << traj_dual.vr_des; acc_r_des << traj_dual.ar_des;
        pos_a_des << traj_dual.pa_des; vel_a_des << traj_dual.va_des; acc_a_des << traj_dual.a_des; 
        gen_dual_traj_dq(pos_r_des,vel_r_des,acc_r_des,or_r_dq, pos_a_des,vel_a_des,acc_a_des, or_a_dq); 

      }else if (choice == 3){

        or_r_dq = DQ(or_r);
        or_a_dq = DQ(or_a);
        phase = demo_coop (pos_r_in, pos_a_in, t); 
        pos_r_des << traj_dual.pr_des; vel_r_des << traj_dual.vr_des; acc_r_des << traj_dual.ar_des;
        pos_a_des << traj_dual.pa_des; vel_a_des << traj_dual.va_des; acc_a_des << traj_dual.a_des; 
        gen_dual_traj_dq(pos_r_des,vel_r_des,acc_r_des,or_r_dq, pos_a_des,vel_a_des,acc_a_des, or_a_dq); 

      }
     

///===========================PUBLISHING =========================///
      //==== DEBUG utils=====///
    DQ x; DQ dx; DQ ddx; 
    x = DQ(traj_dual_dq.xa_des); dx = DQ(traj_dual_dq.dxa_des); ddx = DQ(traj_dual_dq.ddxa_des); 
    DQ p_dot_dq; DQ acc_dot_dq;
    p_dot_dq = 2*D(dx)*(P(x)).conj() + 2*D(x)*(P(dx)).conj();
    acc_dot_dq =  2*D(ddx)*(P(x)).conj() + 4*D(dx)*(P(dx)).conj() + 2*D(x)*(P(ddx)).conj();
    Vector3d p_dot; Vector3d acc_dot;
    p_dot = vec3(p_dot_dq); acc_dot = vec3(acc_dot_dq);  
  
    DQ xr; DQ dxr; DQ ddxr; 
    xr = DQ(traj_dual_dq.xr_des); dxr = DQ(traj_dual_dq.dxr_des); ddxr = DQ(traj_dual_dq.ddxr_des); 
    DQ pr_dot_dq; DQ accr_dot_dq;
    pr_dot_dq = 2*D(dxr)*(P(xr)).conj() + 2*D(xr)*(P(dxr)).conj();
    accr_dot_dq =  2*D(ddxr)*(P(xr)).conj() + 4*D(dxr)*(P(dxr)).conj() + 2*D(xr)*(P(ddxr)).conj();
    Vector3d pr_dot; Vector3d accr_dot;
    pr_dot = vec3(pr_dot_dq); accr_dot = vec3(accr_dot_dq);
    


    traj_msg.header.stamp = ros::Time::now();

          DQ xa_dq,xr_dq,x1_dq,x2_dq;
          Vector3d pa,pr,p1,p2; 
          Vector4d or_r; 
          xa_dq = DQ(traj_dual_dq.xa_des); xa_dq = xa_dq.normalize();
          xr_dq = DQ(traj_dual_dq.xr_des); xr_dq = xr_dq.normalize(); 
          pa = vec3(xa_dq.translation()); pr = vec3(xr_dq.translation()); 
          or_r = vec4(xr_dq.rotation()); 

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
              traj_msg.phase_array[i] = phase(i); 
              traj_msg.vel_r[i] = pr_dot(i);
              traj_msg.acc_r[i] = accr_dot(i); 
              traj_msg.vel_a[i] = p_dot(i);
              traj_msg.acc_a[i] = acc_dot(i); 
              }  

        traj_msg.phase_array[0] = phase(0); 
        traj_msg.phase_array[1] = phase(1); 
        traj_msg.phase_array[2] = phase(2); 

    pub_cmd.publish(traj_msg);
 
    loop_rate.sleep();

     t = (ros::Time::now() - t_init).toSec();  

     count = count+1;   
  } 
  
  }
  return 0;
}
