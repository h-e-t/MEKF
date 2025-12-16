#pragma once

#include <core/mekf_core.hpp>

/*
 * imu_defaults.hpp
 * ----------------
 * Provides a small helper to construct a sane default IMU sensor definition.
 * This is intentionally conservative; values are placeholders and should be
 * adjusted for the specific IMU sensor and units used in your project.
 *
 * Usage:
 *    #include "defaults.hpp"        // or directly include imu_defaults.hpp
 *    IMUSensorDefinition imu = defaults::DefaultIMU();
 */

// initalestimate = [initQuat 0 0 0 0 0 0 0 0 0 0 0 0];

// intialestimateCovariance=eye(15)*.1;
// intialestimateCovariance(1:3,1:3) = eye(3);

// gyro_cov    = diag([.0046 .0046 .0046]);
// gyro_bias_cov    = diag([.05 .05 .05]);

// accel_cov   = diag([.017 .017 .017]);
// accel_bias_cov    = diag([.05 .05 .05]);

// mag_cov   = diag([.04 .04 .04]);

// gps_cov   = diag([sqrt(.2) sqrt(.2) sqrt(.3) sqrt(1.17) sqrt(1.17) sqrt(1.5*1.17)]);

// baro_cov    = .1;

// dt          = .0025;

namespace defaults
{
inline IMUSensorDefinition DefaultIMU()
{
    IMUSensorDefinition d;

    Matrix<float, 3, 3> I3 = Matrix<float, 3, 3>::Identity();
    Matrix<float, 3, 3> Z33 = Matrix<float, 3, 3>::Zero();
    Matrix<float, 3, 1> Z31 = Matrix<float, 3, 1>::Zero();
    Matrix<float, 15, 15> I15 = Matrix<float, 15, 15>::Identity();

    // Covariance matrices (3x3) -- these values are variance terms on the diagonal.
    // Adjust these to match the IMU datasheet (units: rad/sec for gyro, m/s^2 for accel)
    d.accelerometerNoiseCov = I3 * .017;
    d.gyroscopeNoiseCov = I3 * .0046;

    d.magnetometerCov = I3 * .04;

    // GPS covariance (position and velocity) in meters/meters-per-second squared
    d.gpsPosNoiseCov = I3 * 2.25; // e.g. 1.5 m std dev
    d.gpsVelNoiseCov = I3 * 0.25; // e.g. 0.5 m/s std dev

    d.GPSNoiseCovariance.diagonal().block<3, 1>(0, 0) = d.gpsVelNoiseCov.diagonal();
    d.GPSNoiseCovariance.diagonal().block<3, 1>(3, 0) = d.gpsPosNoiseCov.diagonal();

    // Barometer variance (scalar), units depend on how barometer data is used (e.g.
    // meters)
    d.barometerCov = .5f; // 1 meter variance

    // Bias vectors (3x1). Defaults are zero; if you have prior bias estimates, populate
    // them here.
    d.accelerometerBias = Z31;
    d.gyroscopeBias = Z31;
    d.magnetometerBias = Z31;

    // Bias covariances (small values to allow slow drift)
    d.accelerometerBiasCov = I3 * .05;
    d.gyroscopeBiasCov = I3 * .05;

    // CovarianceNoiseFloor: a floor to apply to the full state covariance matrix to
    // avoid ill-conditioning. This is kept as a small default 9x9 identity matrix
    // (commonly equal to the number of state elements, adjust for your state size).
    d.CovarianceNoiseFloor.diagonal() << .0012, .0012, .0012, .05, .05, .05, .2, .2, 1,
        .05, .05, .05, .05, .05, .05;

    return d;
}
/*
 * Example usage (compile-time sample):
 *
 * #include "defaults.hpp"
 * int main() {
 *     auto imu = defaults::DefaultIMU();
 *     std::cout << "Accelerometer Covariance:\n" << imu.accelerometerNoiseCov <<
 * std::endl;
 * }
 */
} // namespace defaults
