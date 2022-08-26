/*
 * Momentum_observer.cpp
 * File to implement the generalized momentum observer to estimate external torques acting on a 7dof manipulator.
 * 
 *
 */

    #include "momentum_observer.h"
    
    Eigen::Vector7d r (const Eigen::Matrix7d &M, const Eigen::Matrix7d &C, const Eigen::Vector7d &g,const Eigen::Vector7d &tau_m,
                    const Eigen::Vector7d &r_prec,const Eigen::Matrix7d &gain,double Ts){
        Vector7d beta = g - C.transpose(); 
        Vector7d dp_hat = tau_m - beta + r_prec; 
        Vector7d p_hat = dp_hat*Ts + p_hat;
        Vector7d r = gain*(M*dq-p_hat-p0); 

                    }

    // Vector7d r mom_obs(M, g,C,gain,r_prec){
    //     beta = (g_mario) - C_t*dq;
    //     // left arm with EE
    //     // beta = (gravity) - C_t*dq;
    //     tau_J = tau_J - initial_tau_ext; //remove torque sensor bias
    //     p_dot_hat = tau_J - beta + r;
    //     //integrate  
    //     p_int_hat  = p_dot_hat*(period.toSec()) + p_int_hat;
    //     //right arm no EE
    //     r = Ko*(m_mario*dq - p_int_hat - p0); 
    // }

    