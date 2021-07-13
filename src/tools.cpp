#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

// Calculate RMSE for the whole vector
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd out_vector(4);
  VectorXd e(4) ;

  if (estimations.empty() ||  ground_truth.empty()) {
    out_vector << 0,0,0,0;
    return out_vector;
  }
    // compute RMSE
  for(int i=0; i < estimations.size(); ++i){
    VectorXd err_ = estimations[i] - ground_truth[i];
    err_ = err_.array()*err_.array();
    out_vector += err_;
  }

  // compute the mean
  out_vector = out_vector/estimations.size();

  // compute the square root
  out_vector = out_vector.array().sqrt();


  for(int i=0; i < estimations.size(); ++i){
    VectorXd err_ = estimations[i] - ground_truth[i];
    err_ = err_.array()*err_.array();
    out_vector += err_;
  }

  // compute the mean
  out_vector = out_vector/estimations.size();

  // compute the square root
  out_vector = out_vector.array().sqrt();
  //std::cout << out_vector << std::endl;
  return out_vector;
}

