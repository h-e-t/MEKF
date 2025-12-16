#include "Eigen/Core"
#include "Eigen/Geometry"
#include <cmath>
#include <iostream>
#include <mekf.hpp>

// ---- PUBLIC FUNCTIONS ----

// Default contstructor with default gravities.
MEKF::MEKF(VehicleState initState, IMUSensorDefinition IMU)
    : vehicleState_(initState), IMU_(IMU), estimatorGravity(0, 0, 9.81),
      estimatorMagneticField(27.550, -2.4169, -16.0849) {

      };

MEKF::MEKF(IMUSensorDefinition IMU)
    : IMU_(IMU), estimatorGravity(0, 0, 9.81),
      estimatorMagneticField(27.550, -2.4169, -16.0849)
{
    vehicleState_.state = Matrix<float, 16, 1>();
    vehicleState_.state << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    vehicleState_.covariance.setZero();
    vehicleState_.covariance.diagonal() << 1, 1, 1, 0.1000, 0.1000, 0.1000, 0.1000,
        0.1000, 0.1000, 0.1000, 0.1000, 0.1000, 0.1000, 0.1000, 0.1000;
};

void MEKF::updateWithGravity(Matrix<float, 3, 1> accelerometerMeasurement)
{

    // consecutiveGravUpdates += 15;

    if (abs(accelerometerMeasurement.norm() - estimatorGravity.norm()) <
        grav_magnitude_residual)
    {
        consecutiveGravUpdates += 1;

        if (consecutiveGravUpdates < 15)
            return;
    }
    else
    {
        consecutiveGravUpdates = 0;
        return;
    }
    Matrix<float, 3, 3> I3;
    Matrix<float, 15, 15> I15;

    I3.setIdentity();
    I15.setIdentity();

    Matrix<float, 16, 1> &state = vehicleState_.state;
    Matrix<float, 15, 15> &cov = vehicleState_.covariance;

    Quaternionf currentAttitude(state(0), state(1), state(2), state(3));

    Matrix<float, 15, 1> delX;
    Matrix<float, 3, 15> H;

    // Error State
    delX.setZero();

    // Measurement Matrix
    H.setZero();

    // ---- BLOCK (1,1) ----
    H.block<3, 3>(0, 0) =
        skewSymmetric(currentAttitude.toRotationMatrix().transpose() * -estimatorGravity);

    // ---- BLOCK (1,5) ----
    H.block<3, 3>(0, 12) = I3;

    Matrix<float, 3, 1> gravDelta =
        (accelerometerMeasurement - state.block<3, 1>(13, 0)) -
        currentAttitude.toRotationMatrix().transpose() * -estimatorGravity;

    MatrixXf Kgain = cov * H.transpose() *
                     (H * cov * H.transpose() + IMU_.accelerometerNoiseCov).inverse();

    delX = Kgain * gravDelta;

    cov = (I15 - Kgain * H) * cov;

    // Attitude Update
    state.block<4, 1>(0, 0) =
        (currentAttitude * Quaternionf(1, 0, delX(1) / 2.0f, delX(2) / 2.0f))
            .coeffsScalarFirst();

    // TODO: Consider updating all states
    // Partial Update (only to gyro and accel bias)
    state.block<6, 1>(10, 0) += delX.block<6, 1>(9, 0);

    checkFloor();
};

void MEKF::updateWithBarometer(float altitude)
{
    Matrix<float, 15, 15> I15;
    Matrix<float, 1, 3> Z13;

    I15.setIdentity();
    Z13.setZero();

    Matrix<float, 16, 1> &state = vehicleState_.state;
    Matrix<float, 15, 15> &cov = vehicleState_.covariance;

    Matrix<float, 15, 1> delX;
    Matrix<float, 1, 15> H;

    // Error State
    delX.setZero();

    // Measurement Matrix
    H.setZero();

    H.block<1, 3>(0, 6) << 0, 0, 1;

    float del_z = altitude - state(9);

    // Gain Calculation
    MatrixXf Kgain =
        cov * H.transpose() *
        (H * cov * H.transpose() + Matrix<float, 1, 1>(IMU_.barometerCov)).inverse();

    delX += Kgain * del_z;

    // Covariance Update
    cov = (I15 - Kgain * H) * cov * (I15 - Kgain * H).transpose() +
          Kgain * IMU_.barometerCov * Kgain.transpose();

    // Updates only the position and other states, not attitude
    state.block<12, 1>(4, 0) += delX.block<12, 1>(3, 0);

    checkFloor();
};

void MEKF::updateWithMagnetometer(Matrix<float, 3, 1> magneticMeasurement)
{
    // float declination = -0.0875;

    Matrix<float, 16, 1> &state = vehicleState_.state;
    Matrix<float, 15, 15> &cov = vehicleState_.covariance;

    Quaternionf currentAttitude(state(0), state(1), state(2), state(3));

    Matrix<float, 3, 1> estimated_mag = Matrix<float, 3, 1>::Zero();
    Matrix<float, 3, 1> del_beta = Matrix<float, 3, 1>::Zero();

    Matrix<float, 3, 3> H = Matrix<float, 3, 3>::Zero();
    Matrix<float, 3, 3> I3 = Matrix<float, 3, 3>::Identity();

    estimated_mag = currentAttitude.toRotationMatrix() * magneticMeasurement;

    float true_declination =
        std::atan2(estimatorMagneticField(1), estimatorMagneticField(0));
    float estimated_declination = std::atan2(estimated_mag(1), estimated_mag(0));

    del_beta(2) = true_declination - estimated_declination;

    H.diagonal() << 0, 0, 1;

    // cout << "del beta" << endl << del_beta;
    // cout << "attitude covariance" << endl << cov.block<3, 3>(0, 0) << endl;
    // cout << "cov * H" << endl << cov.block<3, 3>(0, 0) * H << endl;
    // cout << "H cov H.T" << endl << H * cov.block<3, 3>(0, 0) * H.transpose() << endl;
    // cout << "Mag cov" << endl << IMU_.magnetometerCov << endl;

    MatrixXf Kgain =
        cov.block<3, 3>(0, 0) * H *
        (IMU_.magnetometerCov + H * cov.block<3, 3>(0, 0) * H.transpose()).inverse();

    Matrix<float, 3, 1> alpha = Kgain * del_beta / 2;

    state.block<4, 1>(0, 0) =
        (currentAttitude * Quaternionf(1, alpha(0), alpha(1), alpha(2)))
            .coeffsScalarFirst();

    cov.block<3, 3>(0, 0) = (I3 - Kgain) * cov.block<3, 3>(0, 0);

    checkFloor();
};

void MEKF::propagate(Matrix<float, 3, 1> gyroMeasurement,
                     Matrix<float, 3, 1> accelerometerMeasurement, float dt)
{

    Matrix<float, 16, 1> &state = vehicleState_.state;
    Matrix<float, 15, 15> &cov = vehicleState_.covariance;

    Quaternionf currentAttitude(state(0), state(1), state(2), state(3));

    // Quaternion Propagation
    currentAttitude =
        currentAttitude * quatExp(Quaternionf(0, dt * gyroMeasurement / 2.0f));

    // Outputs attitude in yaw pitch roll
    // cout << rad2deg(currentAttitude.toRotationMatrix().canonicalEulerAngles(2, 1, 0));

    // body to inertial rotation matrix
    Matrix<float, 3, 3> DCM_be = currentAttitude.toRotationMatrix();

    Matrix<float, 3, 1> vkm =
        (DCM_be * accelerometerMeasurement + estimatorGravity) * dt +
        state.block<3, 1>(4, 0);

    Matrix<float, 3, 1> rkm = vkm * dt + state.block<3, 1>(7, 0);

    auto STM = calculateApproximateSTM(gyroMeasurement, accelerometerMeasurement,
                                       currentAttitude, dt);

    cov = STM * cov * STM.transpose() + calculateProcessCovariance(dt);

    // Updating attitude
    state.block<4, 1>(0, 0) = currentAttitude.coeffsScalarFirst().normalized();

    // Updating velocity
    state.block<3, 1>(4, 0) = vkm;

    // Updating position
    state.block<3, 1>(7, 0) = rkm;
};

// ---- HELPER FUNCTIONS ----

Matrix<float, 15, 15> MEKF::calculateProcessCovariance(float dt)
{
    Matrix3f Z3 = Matrix3f::Zero();

    auto sw = IMU_.gyroscopeNoiseCov;
    auto sf = IMU_.accelerometerNoiseCov;

    auto sbw = IMU_.gyroscopeBiasCov;
    auto sbf = IMU_.accelerometerBiasCov;

    Matrix<float, 15, 15> Qd;

    Qd.setZero();

    // ---- DIAG TERMS ----
    // Blocks are 3x3 matrices

    // ---- BLOCK (1,1) ----  attitude covariance
    Qd.block<3, 3>(0, 0) = sw * dt + sbw * dt * dt * dt / 3.0f;

    // ---- BLOCK (2,2) ----  velocity covariance propagation
    Qd.block<3, 3>(3, 3) = sf * dt + sbf * dt * dt * dt / 3.0f;

    // ---- BLOCK (3,3) ----  position covariance propagation
    Qd.block<3, 3>(6, 6) =
        (sbf * dt * dt * dt * dt * dt) / 20.0f + (sf * dt * dt * dt) / 3.0f;

    // ---- BLOCK (4,4) ----  position covariance propagation
    Qd.block<3, 3>(9, 9) = dt * sbw;

    // ---- BLOCK (5,5) ----  gyro bias covariance propagation
    Qd.block<3, 3>(12, 12) = dt * sbf;

    // ---- CROSS TERMS ----

    // ---- BLOCKS (1, 4)/(4,1) ----
    Qd.block<3, 3>(0, 9) = -sbw * dt * dt / 2.0;
    Qd.block<3, 3>(9, 0) = -sbw * dt * dt / 2.0;

    // ---- BLOCKS (2, 3)/(3,2) ----
    Qd.block<3, 3>(3, 6) = sbf * dt * dt * dt * dt / 8.0f + sf * dt * dt / 2.0f;
    Qd.block<3, 3>(6, 3) = sbf * dt * dt * dt * dt / 8.0f + sf * dt * dt / 2.0f;

    // ---- BLOCKS (2, 5)/(5, 2) ----
    Qd.block<3, 3>(3, 12) = -sbf * dt * dt / 2.0f;
    Qd.block<3, 3>(12, 3) = -sbf * dt * dt / 2.0f;

    // ---- BLOCKS (3, 5)/(5, 3) ----
    Qd.block<3, 3>(6, 12) = -sbf * dt * dt / 6.0f;
    Qd.block<3, 3>(12, 6) = -sbf * dt * dt / 6.0f;

    return Qd;
};

Matrix<float, 15, 15>
MEKF::calculateApproximateSTM(Matrix<float, 3, 1> gyroMeasurement,
                              Matrix<float, 3, 1> accelerometerMeasurement,
                              Quaternionf &attitude, float dt)
{
    Matrix<float, 3, 3> I3;
    I3.setIdentity();

    Matrix<float, 15, 15> I15;
    I15.setIdentity();

    Matrix<float, 15, 15> F;
    F.setZero();

    // ---- BLOCK (1,1) ----
    F.block<3, 3>(0, 0) = -skewSymmetric(gyroMeasurement);

    // ---- BLOCK (1,4) ----
    F.block<3, 3>(0, 9) = -I3;

    // ---- BLOCK (2,1) ----
    F.block<3, 3>(3, 0) = -attitude.toRotationMatrix().cast<float>() *
                          skewSymmetric(accelerometerMeasurement);

    // ---- BLOCK (2,5) ----
    F.block<3, 3>(3, 12) = -attitude.toRotationMatrix().cast<float>();

    // ---- BLOCK (3,2) ----
    F.block<3, 3>(6, 3) = I3;

    return I15 + F * dt + 0.5 * F * F * dt * dt;
};

Matrix<float, 3, 3> MEKF::skewSymmetric(Matrix<float, 3, 1> vec)
{
    Matrix<float, 3, 3> skew;

    skew.block<1, 3>(0, 0) << 0, -vec(2), vec(1);
    skew.block<1, 3>(1, 0) << vec(2), 0, -vec(0);
    skew.block<1, 3>(2, 0) << -vec(1), vec(0), 0;

    return skew;
};

Quaternionf MEKF::quatExp(Quaternionf q)
{
    Quaternionf qe;

    float expS = exp(q.w());
    float vNorm = q.vec().norm();

    Matrix<float, 3, 1> normedVec = sin(vNorm) * q.vec() / vNorm;

    qe.w() = expS * cos(vNorm);
    qe.x() = expS * normedVec.x();
    qe.y() = expS * normedVec.y();
    qe.z() = expS * normedVec.z();

    return qe;
};

void MEKF::checkFloor()
{
    for (int i = 0; i < vehicleState_.covariance.rows(); i++)
    {
        if (vehicleState_.covariance(i, i) < IMU_.CovarianceNoiseFloor(i, i))
        {
            vehicleState_.covariance(i, i) = IMU_.CovarianceNoiseFloor(i, i);
        }
    }
};
