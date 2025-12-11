#ifndef MEK_FILTER_HPP
#define MEK_FILTER_HPP

#include "Eigen/Core"
#include <Eigen/Dense>

class MEK_Filter {
private:
  Eigen::MatrixXd vehicleState;
  Eigen::MatrixXd errorState;

public:
  MEK_Filter(Eigen::MatrixXd initialState) : vehicleState(initialState) {}

  
};

#endif MEK_FILTER_HPP
