#ifndef DYNAMICS_H_   /* Include guard */
#define DYNAMICS_H_

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class Dynamics{
    public:
        Dynamics(double rho, double phi, VectorXd Xb);
        ~Dynamics();
        
        VectorXd get_tau_G(VectorXd q);
        VectorXd get_tau_F(VectorXd qd);
        VectorXd get_tau_model(VectorXd q, VectorXd qd, VectorXd qdd);
        MatrixXd get_C(VectorXd q, VectorXd qd);
        MatrixXd get_M(VectorXd q);
    
    private:
        // VectorXd _Xb_G;
        // VectorXd _Xb_C;
        // VectorXd _Xb_M;
        // VectorXd _Xb_F;

        double XB_s1, XB_s2, XB_s3, XB_s4, XB_s5, XB_s6, XB_s7, XB_s8, XB_s9, XB_s10,
               XB_s11, XB_s12, XB_s13, XB_s14, XB_s15, XB_s16, XB_s17, XB_s18, XB_s19, XB_s20,
               XB_s21, XB_s22, XB_s23, XB_s24, XB_s25, XB_s26, XB_s27, XB_s28, XB_s29, XB_s30, 
               XB_s31, XB_s32, XB_s33, XB_s34, XB_s35, XB_s36, XB_s37, XB_s38, XB_s39, XB_s40, 
               XB_s41, XB_s42, XB_s43, XB_s44, XB_s45, XB_s46, XB_s47, XB_s48, XB_s49, XB_s50,
               XB_s51, XB_s52, XB_s53, XB_s54, XB_s55, XB_s56, XB_s57, XB_s58, XB_s59;   

        double srho, sphi, crho, cphi;
};

#endif // DYNAMICS_H_