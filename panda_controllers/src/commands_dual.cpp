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
typedef Matrix<double, 4, 1> Vector4d;

using namespace std; using namespace DQ_robotics; using namespace panda_controllers; using DQ_robotics::E_; 
using DQ_robotics::i_; using DQ_robotics::j_; using DQ_robotics::k_;

////// =====================DUAL ARM=======================//
DQ rot_r, rot_a; 
DQ pos_r_des_dq, pos_a_des_dq, vel_r_des_dq, vel_a_des_dq, acc_r_des_dq, acc_a_des_dq; 
Vector8d pose_abs,pose_rel,pose_1,pose_2;  
Vector8d xa_des,dxa_des,ddxa_des,xr_des,dxr_des,ddxr_des; 
Vector3d pos_r_des,vel_r_des,acc_r_des,pos_a_des,vel_a_des,acc_a_des;
// RECORDED POSES
DQ x_abs_rec_dq, x_rel_rec_dq; 
Vector8d x_abs_rec, x_rel_rec; 
Vector3d pos_abs_rec, pos_rel_rec; 
Vector4d rot_a_rec, rot_r_rec; 

struct traj_dual_struct{
      Vector3d pr_des;
      Vector3d vr_des;
      Vector3d ar_des;
      Vector4d or_r_des; 
      Vector4d dor_r_des; 
      Vector4d ddor_r_des; 
      Vector3d pa_des;
      Vector3d va_des;
      Vector3d a_des; 
      Vector4d or_a_des; 
      Vector4d dor_a_des;
      Vector4d ddor_a_des;
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
  Vector3d p_rel_ref, p_abs_ref;  
  double t_f, t,pc;  
  Vector3d pos_r_f, pos_a_f; 
  double xr_offset;  // offset on x_axis relative frame
  double zr_offset;  // offset on z_axis relative frame
  zr_offset = 0.7;   //m 
  pc = 0.8; 
  p_rel_ref <<0.0094338,0.00907125,0.489452-0.02; 
  p_abs_ref << 0.440214, 0.744113, 0.232157;  
  // p_rel_ref << posr_i(0), posr_i(1), 0.488; 
  // p_abs_ref << posa_i(0), posa_i(1), posa_i(2); 


  if(time>=0 && time<2){ //initial pause
    tmpr << posr_i; 
    tmpa << posa_i; 
    pos_r_f << tmpr; 
    pos_a_f << tmpa; 
    t_f = 2; 
    t = time;
    phase.setZero(); 
  }else if (time>=2 && time<12){ //approach phase
    tmpr << posr_i(0), posr_i(1), posr_i(2); 
    tmpa << posa_i; 
    pos_r_f << p_rel_ref; 
    pos_a_f << p_abs_ref;   
    t_f = 10;
    t = time - 2; 
    phase(2) = 1; 
  }else if (time>=12 && time<17){ //pause (squeeze phase)
    tmpr << p_rel_ref; 
    tmpa << p_abs_ref; ; 
    pos_r_f << tmpr; 
    pos_a_f << tmpa;  
    t_f = 5;
    t = time - 12; 
    phase(2) = 2; 
  }else if (time>=17 && time<27){ //release
    tmpr << p_rel_ref; 
    tmpa << p_abs_ref; 
    pos_r_f << posr_i(0), posr_i(1), posr_i(2); 
    pos_a_f << tmpa; 
    t_f = 10; 
    t = time - 17; 
    phase(2) = 4; 
  }else { // eq phase
    tmpr << posr_i(0), posr_i(1), posr_i(2); 
    tmpa << p_abs_ref; 
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


Vector3d demo_coop (Vector3d posr_i, Vector3d posa_i,Vector3d rel_ref, Vector3d abs_ref, double time) {
  double t_f, t; 
  Vector3d phase; 
  Vector3d tmpr,tmpa; 
  Vector3d pos_r_f, pos_a_f; 
  double za_offset; 
  za_offset = 0.15; 

  if(time>=0 && time<2){ //initial pause
    tmpr << posr_i; 
    tmpa << posa_i; 
    pos_r_f << tmpr; 
    pos_a_f << tmpa; 
    t_f = 2; 
    t = time;
    phase.setZero(); 
  }else if (time>=2 && time<9){ //approach phase
    tmpr << posr_i(0), posr_i(1), posr_i(2); 
    tmpa << posa_i; 
    pos_r_f << rel_ref; 
    pos_a_f << abs_ref;  
    t_f = 7;
    t = time - 2; 
    phase(2) = 1;
  }else if (time>=9 && time<14){ //pause
    tmpr << rel_ref; 
    tmpa << abs_ref; 
    pos_r_f << tmpr; 
    pos_a_f << tmpa;  
    t_f = 5;
    t = time - 9; 
    phase(2) = 2; 
  }else if (time>=14 && time<19){ //go up
    tmpr << rel_ref;
    tmpa << abs_ref(0), abs_ref(1), abs_ref(2); 
    pos_r_f << tmpr; 
    pos_a_f << tmpa(0), tmpa(1), tmpa(2) + za_offset; 
    t_f = 5;
    t = time - 14; 
    phase(2) = 3; 
  }else if (time>=19 && time<24){ //go down
    tmpr << rel_ref;
    tmpa << abs_ref(0), abs_ref(1), abs_ref(2) + za_offset; 
    pos_r_f << tmpr; 
    pos_a_f << abs_ref(0), abs_ref(1), abs_ref(2); 
    t_f = 5;
    t = time - 19; 
    phase(2) = 3; 
  }else if (time>=24 && time<29){ //release
    tmpr <<  rel_ref;
    tmpa <<  abs_ref(0), abs_ref(1), abs_ref(2); 
    pos_r_f << posr_i(0), posr_i(1), posr_i(2); 
    pos_a_f << tmpa; 
    t_f = 5; 
    t = time - 24; 
    phase(2) = 4; 
  }else { // eq phase
    tmpr << posr_i(0), posr_i(1), posr_i(2); 
    tmpa << abs_ref(0), abs_ref(1), abs_ref(2); 
    pos_r_f << tmpr; 
    pos_a_f << tmpa;
    t_f = 1000; 
    t = time - 29; 
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

Vector3d demo_box (Vector3d posr_i,Vector4d or_i, Vector3d posa_i, Vector4d or_a_i,
                    Vector3d pos_rel_rec, Vector4d rot_r_rec, Vector3d pos_a_rec, Vector4d rot_a_rec, double time) {
  Vector3d tmpr,tmpa;
  Vector4d tmp_or,tmpa_or;
  Vector3d phase; 
  double t_f, t;  
  Vector3d pos_r_f, pos_a_f; 
  Vector4d or_r_f, or_a_f; 
  double z_offset = 0.15; 
  double zr_offset = 0.25; 

  if(time>=0 && time<2){ //initial pause
    tmpr << posr_i; 
    tmpa << posa_i; 
    pos_r_f << tmpr; 
    pos_a_f << tmpa; 
    tmp_or << or_i;
    tmpa_or << or_a_i;
    or_r_f << rot_r_rec; 
    or_a_f << rot_a_rec; 
    t_f = 2; 
    t = time;
    phase.setZero(); 
  }else if (time>=2 && time<10){ //approach phase
    tmpr << posr_i; 
    tmpa << posa_i; 
    pos_r_f << pos_rel_rec; 
    pos_a_f << pos_a_rec;  
    tmp_or << rot_r_rec;
    tmpa_or << rot_a_rec;
    or_r_f << tmp_or;
    or_a_f << tmpa_or; 
    // or_r_f << rot_r_rec; 
    // or_a_f << rot_a_rec;
    t_f = 8;
    t = time - 2; 
    phase(2) = 1;
  }else if (time>=10 && time<15){ //pause
    tmpr << pos_rel_rec; 
    tmpa << pos_a_rec;   
    pos_r_f << tmpr; 
    pos_a_f << tmpa;  
    // tmp_or << rot_r_rec;
    // tmpa_or << rot_a_rec;
    tmp_or << rot_r_rec;
    tmpa_or << rot_a_rec;
    or_r_f << tmp_or; 
    or_a_f << tmpa_or; 
    t_f = 5;
    t = time - 10; 
    phase(2) = 2; 
  }else if (time>=15 && time<20){ //go up
    tmpr << pos_rel_rec; 
    tmpa << pos_a_rec;   
    pos_r_f << tmpr; 
    pos_a_f << tmpa(0), tmpa(1), tmpa(2) + z_offset;  
    // tmp_or << rot_r_rec;
    // tmpa_or << rot_a_rec;
    tmp_or << rot_r_rec;
    tmpa_or << rot_a_rec;
    or_r_f << tmp_or; 
    or_a_f << tmpa_or; 
    t_f = 5;
    t = time - 15; 
    phase(2) = 2; 
  }else if (time>=20 && time<24){ //go down
    tmpr << pos_rel_rec; 
    tmpa << pos_a_rec(0), pos_a_rec(1), pos_a_rec(2) + z_offset;   
    pos_r_f << tmpr; 
    pos_a_f << pos_a_rec(0), pos_a_rec(1), pos_a_rec(2);  
    // tmp_or << rot_r_rec;
    // tmpa_or << rot_a_rec;
    tmp_or << rot_r_rec;
    tmpa_or << rot_a_rec;
    or_r_f << tmp_or; 
    or_a_f << tmpa_or; 
    t_f = 4;
    t = time - 20; 
    phase(2) = 3; 
  }else if (time>=24 && time<29){ //release
    tmpr << pos_rel_rec; 
    tmpa << pos_a_rec(0), pos_a_rec(1), pos_a_rec(2);   
    pos_r_f << tmpr(0), tmpr(1), tmpr(2) + zr_offset; 
    pos_a_f << pos_a_rec(0), pos_a_rec(1), pos_a_rec(2);  
    // tmp_or << rot_r_rec;
    // tmpa_or << rot_a_rec;
    tmp_or << rot_r_rec;
    tmpa_or << rot_a_rec;
    or_r_f << tmp_or; 
    or_a_f << tmpa_or; 
    t_f = 5; 
    t = time - 24; 
    phase(2) = 4; 
  }else { // eq phase
    tmpr << pos_rel_rec(0), pos_rel_rec(1), pos_rel_rec(2) + zr_offset; 
    tmpa << pos_a_rec(0), pos_a_rec(1), pos_a_rec(2);   
    pos_r_f << tmpr; 
    pos_a_f << tmpa;  
    tmp_or << rot_r_rec;
    tmpa_or << rot_a_rec;
    // tmp_or << rot_r_rec;
    // tmpa_or << rot_a_rec;
    or_r_f << tmp_or; 
    or_a_f << tmpa_or; 
    t = time - 29; 
    t_f = 1000; 
    phase(2) = 4; 
  }
   
   //des traj for relative var
   traj_dual.pr_des << tmpr + (tmpr - pos_r_f)*(15*pow((t/t_f),4) - 6*pow((t/t_f),5) -10*pow((t/t_f),3));
   traj_dual.vr_des << (tmpr - pos_r_f)*(60*(pow(t,3)/pow(t_f,4)) - 30*(pow(t,4)/pow(t_f,5)) -30*(pow(t,2)/pow(t_f,3)));
   traj_dual.ar_des << (tmpr - pos_r_f)*(180*(pow(t,2)/pow(t_f,4)) - 120*(pow(t,3)/pow(t_f,5)) -60*(t/pow(t_f,3)));

   traj_dual.or_r_des << tmp_or + (tmp_or - or_r_f)*(15*pow((t/t_f),4) - 6*pow((t/t_f),5) -10*pow((t/t_f),3));
   traj_dual.dor_r_des << (tmp_or - or_r_f)*(60*(pow(t,3)/pow(t_f,4)) - 30*(pow(t,4)/pow(t_f,5)) -30*(pow(t,2)/pow(t_f,3)));
   traj_dual.ddor_r_des << (tmp_or - or_r_f)*(180*(pow(t,2)/pow(t_f,4)) - 120*(pow(t,3)/pow(t_f,5)) -60*(t/pow(t_f,3))); 

   //des trj for absolute var
   traj_dual.pa_des << tmpa + (tmpa - pos_a_f)*(15*pow((t/t_f),4) - 6*pow((t/t_f),5) -10*pow((t/t_f),3));
   traj_dual.va_des << (tmpa - pos_a_f)*(60*(pow(t,3)/pow(t_f,4)) - 30*(pow(t,4)/pow(t_f,5)) -30*(pow(t,2)/pow(t_f,3)));
   traj_dual.a_des << (tmpa - pos_a_f)*(180*(pow(t,2)/pow(t_f,4)) - 120*(pow(t,3)/pow(t_f,5)) -60*(t/pow(t_f,3)));

   traj_dual.or_a_des << tmpa_or + (tmpa_or - or_a_f)*(15*pow((t/t_f),4) - 6*pow((t/t_f),5) -10*pow((t/t_f),3));
   traj_dual.dor_a_des << (tmpa_or - or_a_f)*(60*(pow(t,3)/pow(t_f,4)) - 30*(pow(t,4)/pow(t_f,5)) -30*(pow(t,2)/pow(t_f,3)));
   traj_dual.ddor_a_des << (tmpa_or - or_a_f)*(180*(pow(t,2)/pow(t_f,4)) - 120*(pow(t,3)/pow(t_f,5)) -60*(t/pow(t_f,3)));

   return phase; 
}

//Utils
Vector3d compute_ref(Vector8d pose_rel_in){
  DQ pose_rel_dq; pose_rel_dq = (DQ(pose_rel_in)).normalize(); 
  DQ rot_r; 
  DQ transl; 
  DQ pose_d_dq; 
  Vector3d pos_in; 
  Vector3d pos_d_rel;
  Vector3d pos_d_world; 
  Vector3d pos_rel_in_world; 
  //Translate the ref using the world reference but keeping the orientation fixed
  rot_r = pose_rel_dq.rotation(); 
  pos_in = vec3(pose_rel_dq.translation()); 
  transl = 1 + E_*0.5*(0.0*i_+ 0.0*j_-0.7*k_); 
  pose_d_dq = transl*pose_rel_dq; 
  pos_rel_in_world = vec3(rot_r*DQ(pos_in)*rot_r.conj()); 
  pos_d_rel = vec3(pose_d_dq.translation()); 
  pos_d_world = vec3(rot_r*DQ(pos_d_rel)*rot_r.conj()); 
  return pos_d_rel; 
  
}

//lifting
Vector3d demo_lifting (Vector3d posr_i, Vector3d posa_i, double time) {
  Vector3d tmpr,tmpa;
  Vector3d phase; 
  Vector3d p_rel_ref, p_abs_ref;  
  double t_f, t,pc;  
  Vector3d pos_r_f, pos_a_f; 
  p_rel_ref << posr_i; 
  p_abs_ref << posa_i(0), posa_i(1), posa_i(2)+0.25; 
  pc = 0.12; 
  //pc = 0.11; 

  if(time>=0 && time<5){ //initial 
    tmpr << posr_i; 
    tmpa << posa_i; 
    pos_r_f << posr_i(0),posr_i(1),posr_i(2)-pc; 
    pos_a_f << tmpa; 
    t_f = 5; 
    t = time;
    phase(2)= 1; 
  }else if (time>=5 && time<10){ //adaptive phase (increase relative to grasp)
    tmpr << posr_i(0),posr_i(1),posr_i(2)-pc;  
    tmpa << posa_i; 
    pos_r_f << posr_i(0),posr_i(1),posr_i(2)-pc; 
    pos_a_f << tmpa;   
    t_f = 5;
    t = time - 5; 
    phase(2) = 2; 
  }else if (time>=10 && time<15){ // lift phase
    tmpr << posr_i(0),posr_i(1),posr_i(2)-pc; 
    tmpa << posa_i; 
    pos_r_f << tmpr; 
    pos_a_f << p_abs_ref;  
    t_f = 5;
    t = time - 10; 
    phase(2) = 3; 
  }else if (time>=15 && time<20){ // come back down 
    tmpr << posr_i(0),posr_i(1),posr_i(2)-pc; 
    tmpa << p_abs_ref; 
    pos_r_f << tmpr; 
    pos_a_f << p_abs_ref; 
    t_f = 5; 
    t = time - 15; 
    phase(2) = 3; 
  }else { // eq phase
    tmpr << posr_i(0),posr_i(1),posr_i(2)-pc; 
    tmpa << p_abs_ref; 
    pos_r_f << tmpr; 
    pos_a_f << tmpa;
    t_f = 1000; 
    t = time - 20; 
    phase(2) = 3; 
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
// Fixed orientation
  void gen_dual_traj_dq (Vector3d posr_des, Vector3d velr_des,
                       Vector3d accr_des, DQ or_dq, DQ or_n, Vector3d posa_des, Vector3d vela_des,
                       Vector3d acca_des, DQ ora_dq, Vector3d phase
                              ){  
        //==DQ traj for relative pose   
        DQ x_des_dq,dx_des_dq,ddx_des_dq;  
        DQ or_dq_; //relative orientation
        
        // if(phase(2)==0 || phase(2)==1 || phase(2) ==2) {
        //   or_dq_ = or_dq; 
        // }else{
        //   or_dq_ = or_n; //new relative rotation after stiff modulation
        // }
        or_dq_ = or_dq; 
        pos_r_des_dq = DQ(posr_des);    
        x_des_dq = or_dq_ + 0.5*E_*(pos_r_des_dq*or_dq_); 
        x_des_dq = x_des_dq * x_des_dq.inv().norm();
        traj_dual_dq.xr_des << vec8(x_des_dq); 

        vel_r_des_dq = DQ(velr_des);
        dx_des_dq = 0.5*E_*(vel_r_des_dq*or_dq_); 
        traj_dual_dq.dxr_des << vec8(dx_des_dq); 

        acc_r_des_dq = DQ(accr_des);
        ddx_des_dq = 0.5*E_*(acc_r_des_dq*or_dq_); 
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

// Compute desired trajectory podition and orientation
void gen_nom_traj_dq (Vector3d posr_des, Vector3d velr_des, Vector3d accr_des, Vector4d or_d, Vector4d dor_d, Vector4d ddor_d, 
                       Vector3d posa_des, Vector3d vela_des,Vector3d acca_des, Vector4d oa_d, Vector4d doa_d, Vector4d ddoa_d
                              ){  

        //==DQ traj for relative pose   
        DQ x_des_dq,dx_des_dq,ddx_des_dq; 
        DQ or_dq, dor_dq, ddor_dq; 
        //pose
        pos_r_des_dq = DQ(posr_des); or_dq = DQ(or_d); 
        x_des_dq = or_dq + 0.5*E_*(pos_r_des_dq*or_dq); 
        x_des_dq = x_des_dq * x_des_dq.inv().norm();
        traj_dual_dq.xr_des << vec8(x_des_dq); 
        //dpose
        vel_r_des_dq = DQ(velr_des); dor_dq = DQ(dor_d); 
        dx_des_dq = dor_dq + 0.5*E_*(vel_r_des_dq*or_dq) + 0.5*E_*(pos_r_des_dq*dor_dq); 
        traj_dual_dq.dxr_des << vec8(dx_des_dq); 

        acc_r_des_dq = DQ(accr_des); ddor_dq = DQ(ddor_d); 
        ddx_des_dq = ddor_dq + 0.5*E_*(acc_r_des_dq*or_dq) + E_*(vel_r_des_dq*dor_dq) + 0.5*E_*(pos_r_des_dq*ddor_dq); 
        traj_dual_dq.ddxr_des << vec8(ddx_des_dq); 

        //==DQ traj for abs pose
        DQ xa_des_dq,dxa_des_dq,ddxa_des_dq;    
        DQ ora_dq, dora_dq, ddora_dq; 

        pos_a_des_dq = DQ(posa_des); ora_dq = DQ(oa_d); 
        xa_des_dq = ora_dq + 0.5*E_*(pos_a_des_dq*ora_dq); 
        xa_des_dq = xa_des_dq * xa_des_dq.inv().norm();
        traj_dual_dq.xa_des << vec8(xa_des_dq); 

        vel_a_des_dq = DQ(vela_des); dora_dq = DQ(doa_d); 
        dxa_des_dq = dora_dq + 0.5*E_*(vel_a_des_dq*ora_dq) + 0.5*E_*(pos_a_des_dq*dora_dq); 
        traj_dual_dq.dxa_des << vec8(dxa_des_dq); 

        acc_a_des_dq = DQ(acca_des); ddora_dq = DQ(ddoa_d); 
        ddxa_des_dq = ddora_dq + 0.5*E_*(acc_a_des_dq*ora_dq) + E_*(vel_a_des_dq*dora_dq) + 0.5*E_*(pos_a_des_dq*ddora_dq); 
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
  ros::Rate loop_rate(200);

  panda_controllers::DesiredProjectTrajectory traj_msg;

  signal(SIGINT, signal_callback_handler);
  
////===== VARIABLES ======== //
//==== Recorded poses ====== //
// x_abs_rec << -0.0177016,0.898249,0.43819, -0.0287141,-0.352547,-0.0679401,0.108691,-0.249321;
// x_rel_rec << -0.00452771,-0.632865,-0.774156,0.0119421,0.0104606,0.1547,-0.126511,0.00102879;  
x_abs_rec << -0.0946609 ,0.90378,0.34483,-0.235189,-0.294969,-0.151636,0.121554,-0.285763; 
x_rel_rec << -0.00159868,-0.745495,-0.666262,0.0181218,0.013443,0.13342,-0.148851,0.0172249; 
x_abs_rec_dq = (DQ(x_abs_rec)).normalize(); x_rel_rec_dq = (DQ(x_rel_rec)).normalize(); 
pos_abs_rec = vec3(x_abs_rec_dq.translation()); pos_rel_rec = vec3(x_rel_rec_dq.translation()); 
rot_a_rec = vec4(x_abs_rec_dq.rotation()); rot_r_rec = vec4(x_rel_rec_dq.rotation()); 


Vector3d pos_r_f,pos_r_in, pos_a_f,pos_a_in;
Vector3d phase;
Vector4d or_r, or_a, rot_2; 
DQ pose_1_dq, pose_2_dq, pose_a_dq, pose_r_dq;
DQ or_r_dq; DQ or_a_dq; DQ or_2_dq; 
Vector8d pose_abs_in,pose_rel_in; 
double tf,zf,Ts;
ros::Time t_init; 
Ts = 0.01; //sampling time node 
double t = 0;
int count = 0;
int choice; 
  
  while (ros::ok())
  {
    cout<<"choice:   (1:dummy, 2:contact, 3:demo, 4:demo_box_2, 5:demo_lifting) "<<endl;
    cin>>choice;

    if(choice == 1){
      cout<<"insert time_f: "<<endl;
      cin>>tf;   
    }else if (choice == 2){
      tf = 30; 
      cout <<"contact_demo_loaded.." << endl; 
    }else if(choice == 3){
      tf = 40;
      cout<< "demo_box.."<<endl; 
    }else if(choice == 4){
      tf = 40;
      cout<< "starting demo"<<endl; 
    }else if(choice == 5){
      tf = 25;
      cout<< "starting demo lifting"<<endl; 
    }
    ros::spinOnce();

    //initialize cdts variables
    pose_a_dq = DQ(pose_abs); pose_a_dq = pose_a_dq.normalize(); or_a = vec4(pose_a_dq.rotation()); 
    pose_r_dq = DQ(pose_rel); pose_r_dq = pose_r_dq.normalize(); or_r = vec4(pose_r_dq.rotation());
    pose_2_dq = DQ(pose_2); 
    pose_abs_in << pose_abs; pos_a_in = vec3(pose_a_dq.translation()); or_2_dq = pose_2_dq.rotation(); 
    pose_rel_in << pose_rel; pos_r_in = vec3(pose_r_dq.translation()); rot_2 = vec4(or_2_dq); 
    double offset = 0.05; // inside
    pos_rel_rec <<-0.028356, 0.0274467, 0.399717-offset; 
    pos_abs_rec << pos_a_in; 
  
    t_init = ros::Time::now();
    t = (ros::Time::now() - t_init).toSec();

  while (t <= tf){
       if (choice == 1){
        Vector3d dummy_phase; dummy_phase.setZero(); 
        or_r_dq = DQ(or_r);
        or_a_dq = DQ(or_a); 
        dummy(pos_r_in,pos_a_in, tf); 
        pos_r_des << traj_dual.pr_des; vel_r_des << traj_dual.vr_des; acc_r_des << traj_dual.ar_des; 
        pos_a_des << traj_dual.pa_des; vel_a_des << traj_dual.va_des; acc_a_des << traj_dual.a_des; 
        gen_dual_traj_dq(pos_r_des,vel_r_des,acc_r_des,or_r_dq,or_r_dq, pos_a_des,vel_a_des,acc_a_des, or_a_dq,dummy_phase); 


      }else if (choice == 2){
        Vector4d or_n; 
        or_n = vec4(pose_r_dq.rotation()); 
        or_a_dq = DQ(or_a);
        DQ or_n_dq; 
        or_n_dq = DQ(or_n); 
        or_r_dq = DQ(or_r); //initial orientation
        phase = demo_contact(pos_r_in, pos_a_in, t); 
        pos_r_des << traj_dual.pr_des; vel_r_des << traj_dual.vr_des; acc_r_des << traj_dual.ar_des;
        pos_a_des << traj_dual.pa_des; vel_a_des << traj_dual.va_des; acc_a_des << traj_dual.a_des; 
        gen_dual_traj_dq(pos_r_des,vel_r_des,acc_r_des,or_r_dq, or_n_dq, pos_a_des,vel_a_des,acc_a_des, or_a_dq,phase); 

      }else if (choice == 3){

        Vector3d ref; 
        or_a = vec4(pose_a_dq.rotation());
        or_a_dq = DQ(or_a);
        DQ or_n; or_n = pose_r_dq.rotation(); 
        or_r_dq = DQ(or_r);
        ref = compute_ref(pose_rel_in); 
        phase = demo_coop(pos_r_in, pos_a_in,ref,pos_a_in,t);
        pos_r_des << traj_dual.pr_des; vel_r_des << traj_dual.vr_des; acc_r_des << traj_dual.ar_des;
        pos_a_des << traj_dual.pa_des; vel_a_des << traj_dual.va_des; acc_a_des << traj_dual.a_des; 
        gen_dual_traj_dq(pos_r_des,vel_r_des,acc_r_des,or_r_dq,or_n,pos_a_des,vel_a_des,acc_a_des, or_a_dq,phase); 

      }else if (choice == 4){
        rot_a_rec << or_a; rot_r_rec << or_r; 
        phase = demo_box(pos_r_in,or_r, pos_a_in,or_a,pos_rel_rec,rot_r_rec,pos_abs_rec,rot_a_rec,t); 
        pos_r_des << traj_dual.pr_des; vel_r_des << traj_dual.vr_des; acc_r_des << traj_dual.ar_des;
        pos_a_des << traj_dual.pa_des; vel_a_des << traj_dual.va_des; acc_a_des << traj_dual.a_des; 
        gen_nom_traj_dq(pos_r_des,vel_r_des,acc_r_des,traj_dual.or_r_des,traj_dual.dor_r_des, traj_dual.ddor_r_des,
                        pos_a_des,vel_a_des, acc_a_des,traj_dual.or_a_des,traj_dual.dor_a_des, traj_dual.ddor_a_des); 
      }else if (choice == 5){
        or_a_dq = DQ(or_a);
        DQ or_n; 
        or_n = DQ(or_r); 
        or_r_dq = DQ(or_r);
        phase = demo_lifting(pos_r_in, pos_a_in, t); 
        pos_r_des << traj_dual.pr_des; vel_r_des << traj_dual.vr_des; acc_r_des << traj_dual.ar_des;
        pos_a_des << traj_dual.pa_des; vel_a_des << traj_dual.va_des; acc_a_des << traj_dual.a_des; 
        gen_dual_traj_dq(pos_r_des,vel_r_des,acc_r_des,or_r_dq, or_n, pos_a_des,vel_a_des,acc_a_des, or_a_dq,phase); 

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
          Vector4d or_r, or_a; 
          xa_dq = DQ(traj_dual_dq.xa_des); xa_dq = xa_dq.normalize();
          xr_dq = DQ(traj_dual_dq.xr_des); xr_dq = xr_dq.normalize(); 
          pa = vec3(xa_dq.translation()); pr = vec3(xr_dq.translation()); 
          or_r = vec4(xr_dq.rotation()); or_a = vec4(xa_dq.rotation()); 

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
          for (int i=0; i<4; i++){
              traj_msg.rot_r[i] = or_r(i);
              traj_msg.rot_a[i] = or_a(i);
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
