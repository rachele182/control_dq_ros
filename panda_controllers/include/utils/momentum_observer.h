/*
  Momentum_observer.h
 * Header file for momentum observer function.
 * Inputs: M = mass matrix M(q) [n_joints X n_joints]
 *         C = coriolis matrix C(q,dq) [n_joints X n_joints]
 *         g = gravity vector g(q) [n_joints X 1]
 *         tau_m = active measured motor torques [n_joints X 1]
 *         r_prec = previously computed momentum obsever
 *         gain = gain observer matrix [n_joints X n_joints] (tunable)
 *         Ts = sampling time [s]
 */

#include <Eigen/Core>

#ifndef INCLUDE_MOMENTUM_OBSERVER_H_
#define INCLUDE_MOMENTUM_OBSERVER_H_

#define NUMBER_OF_JOINTS 7

namespace Eigen {

	typedef Eigen::Matrix< double , NUMBER_OF_JOINTS , NUMBER_OF_JOINTS > Matrix7d;
	typedef Eigen::Matrix< double , NUMBER_OF_JOINTS , 1 > Vector7d;
}

using namespace std;

Eigen::Vector7d r(const Eigen::Matrix7d &M, const Eigen::Matrix7d &C, const Eigen::Vector7d &g,const Eigen::Vector7d &tau_m,
                    const Eigen::Vector7d &r_prec,const Eigen::Matrix7d &gain, double Ts);

#endif /* INCLUDE_MOMENTUM_OBSERVER_H_ */
