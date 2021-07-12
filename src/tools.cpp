#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd out_vector(4);
  VectorXd e(4) ;

  if (estimations.empty() ||  ground_truth.empty()) {
    out_vector << 0,0,0,0;
    return out_vector;
      }

  out_vector << (estimations.back() - ground_truth.back()); // Error cvector alculation
  out_vector = out_vector.cwiseAbs();  // Compute square of differences
  //std::cout << out_vector << std::endl;
  return out_vector;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
   
   MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  if ((px==0) && (py==0)) {
      std::cout << "Error: px and py are 0" << std::endl << Hj << std::endl;
    return Hj;
    };
     // compute the Jacobian matrix
    float powerP = pow(px,2) + pow(py,2);
    float den = sqrt(powerP);
    
    Hj << px/den, py/den, 0, 0,
    -py/powerP, px/powerP, 0, 0,
    py*(vx*py - vy*px)/pow(powerP,3/2), px*(vy*px - vx*py)/pow(powerP,3/2), px/den, py/den;
 
  return Hj;
}
