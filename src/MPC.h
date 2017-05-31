#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

const size_t N = 20;
const double dt = 0.05; // planning time = N * dt = 1s
const int latency_ind = 4; //latency_ind = delay(100ms) / dt = 4

const double ref_v = 50;

const double cte_weight_in_cost = 1;
const double epsi_weight_in_cost = 1;
const double v_weight_in_cost = 1;
const double steer_weight_in_cost = 1;
const double throttle_weight_in_cost = 10;
const double steerchange_weight_in_cost = 1000;
const double throttlechange_weight_in_cost = 1;

struct Solution {
  
		vector<double> X;
		vector<double> Y;
		vector<double> Psi;
		vector<double> V;
		vector<double> Cte;
		vector<double> EPsi;
		vector<double> Delta;
		vector<double> A;
};

class MPC {
public:
  MPC();
  
  virtual ~MPC();
  
  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  //vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  Solution Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
  double delta_prev {0};
  double a_prev {0.1};
  
};


#endif /* MPC_H */
