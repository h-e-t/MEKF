#pragma once

#include "Eigen/Geometry"
#include <Eigen/Core>

using Eigen::Matrix;

struct VehicleState
{
    Matrix<float, 16, 1> state;
    Matrix<float, 15, 15> covariance;
};
