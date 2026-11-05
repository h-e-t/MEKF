#pragma once

#include "Eigen/Core"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

#include <core/mekf_core.hpp>

#define grav_magnitude_residual .4

using Eigen::Matrix3d, Eigen::MatrixXf, Eigen::Matrix, Eigen::Quaternionf, std::cout,
    std::endl, Eigen::cos, Eigen::sin, Eigen::Matrix3f;
// %==================================================================
// % State Vector Definitions
// %
// % TRUE STATE VECTOR:
// %  [0:3]   quaternion attitude
// %  [4:6]   velocity
// %  [7:9]  position
// % [10:12]  gyro bias
// % [13:15]  accel bias
// %
// % ERROR STATE VECTOR:
// %  [0:2]   attitude error (3-vector)
// %  [3:5]   velocity error
// %  [6:8]   position error
// % [9:11]  gyro bias error
// % [12:14]  accel bias error
// %==================================================================

class MEKF
{
  private:
    VehicleState vehicleState_;
    IMUSensorDefinition IMU_;

    float consecutiveGravUpdates = 0;

    Matrix<float, 3, 1> estimatorGravity;
    Matrix<float, 3, 1> estimatorMagneticField;

    Matrix<float, 3, 3> skewSymmetric(Matrix<float, 3, 1> vec);

    Quaternionf quatExp(Quaternionf q);

    Matrix<float, 15, 15> calculateProcessCovariance(float dt);

    Matrix<float, 15, 15>
    calculateApproximateSTM(Matrix<float, 3, 1> gyroMeasurement,
                            Matrix<float, 3, 1> accelerometerMeasurement,
                            Quaternionf &attitude, float dt);

    void checkFloor();

  public:
    MEKF(IMUSensorDefinition IMU);

    MEKF(VehicleState initState, IMUSensorDefinition IMU);

    void propagate(Matrix<float, 3, 1> gyroMeasurement,
                   Matrix<float, 3, 1> accelerometerMeasurement, float dt);

    void updateWithGravity(Matrix<float, 3, 1> accelerometerMeasurement);

    void updateWithMagnetometer(Matrix<float, 3, 1>);

    void updateWithBarometer(float altitude);

    void updateWithGPS(Matrix<float, 3, 1> position, Matrix<float, 3, 1> velocity);

    Matrix<float, 16, 1> inline getFilterState() { return vehicleState_.state; };

    Matrix<float, 15, 1> inline getFilterCovariance()
    {
        return vehicleState_.covariance.diagonal();
    };
};
