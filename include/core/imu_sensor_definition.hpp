#pragma once
#include <Eigen/Core>

using Eigen::MatrixXd;
using Eigen::Matrix;

struct IMUSensorDefinition
{
    // Noise Covariances
    Matrix<float, 3, 3> accelerometerNoiseCov;
    Matrix<float, 3, 3> gyroscopeNoiseCov;

    Matrix<float, 3, 3> magnetometerCov;

    Matrix<float, 3, 3> gpsPosNoiseCov;
    Matrix<float, 3, 3> gpsVelNoiseCov;

    float barometerCov;

    // Bias Definitions
    Matrix<float, 3, 1> accelerometerBias;
    Matrix<float, 3, 1> gyroscopeBias;
    Matrix<float, 3, 1> magnetometerBias;

    // Bias Covariances
    Matrix<float, 3, 3> accelerometerBiasCov;
    Matrix<float, 3, 3> gyroscopeBiasCov;

    // Sensor Suite Noise floor
    Matrix<float, 15, 15> CovarianceNoiseFloor;

    Matrix<float, 6, 6> GPSNoiseCovariance;
};
