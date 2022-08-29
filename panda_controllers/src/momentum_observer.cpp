/*
 * Momentum_observer.cpp
 * File to implement the generalized momentum observer to estimate external torques acting on a 7dof manipulator.
 * 
 *
 */
    // #pragma once
#include <panda_controllers/momentum_observer.h>
    
   Eigen::Vector14d get_r_hat(const Eigen::Matrix7d &M, const Eigen::Matrix7d &C, const Eigen::Vector7d &g,const Eigen::Vector7d &tau_m,
                   const Eigen::Vector7d &dq,const Eigen::Vector7d &p0, const Eigen::Vector7d &p_hat_in ,const Eigen::Vector7d &r_last,const Eigen::Matrix7d &gain, double Ts){
        Eigen::Vector7d beta, dp_hat, p_hat, r_hat; 
        Eigen::Vector14d observer; 
        beta = g - (C.transpose())*dq; 
        dp_hat = tau_m - beta + r_last; 
        p_hat = dp_hat*Ts + p_hat_in;
        r_hat = gain*(M*dq-p_hat- p0);
        observer.head(7) = p_hat;
        observer.tail(7) = r_hat;
    return observer;     
    }
