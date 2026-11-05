#include "Eigen/Core"
#include "Eigen/Geometry"
#include <ETL.hpp>

#include <core/mekf_core.hpp>
#include <utilities/eigenCSVLogging.hpp>

#include <cstdlib>
#include <iomanip>
#include <string>

using std::cout, std::endl;

// int main(int argc, char *argv[])
int main()
{
    // Arg1  dataset path
    // Arg2  delimeter
    // Arg3  wether or not there is a head

    // ETL etl(argv[1], argv[2], argv[3]);
    std::string path = "C:"
                       "\\Users\\emmat\\Desktop\\PERSONAL\\RocketDevelopment\\MEKF\\src\\"
                       "datasets\\Log_1209_221705.csv";

    ETL etl(path, ",", 0);

    // Data format:
    //  [MeasuredBodyAngularRates MeasuredBodyAccelerations MeasuredBodyMagMeasurements
    //  MeasuredPosition MeasuredVelocity]
    MatrixXd dataset = etl.readCSV();

    // CSVLogger statelogger("../logs/stateLog.csv");
    // CSVLogger covlogger("../logs/covLog.csv");

    // CSVLogger statelogger("../logs/GravUpdatestateLog.csv");
    // CSVLogger covlogger("../logs/GravUpdatecovLog.csv");

    // CSVLogger statelogger("../logs/GravBaroUpdatestateLog.csv");
    // CSVLogger covlogger("../logs/GravBaroUpdatecovLog.csv");

    // CSVLogger statelogger("../logs/propagation_grav_baro_mag_state.csv");
    // CSVLogger covlogger("../logs/propagation_grav_baro_mag_cov.csv");

    CSVLogger statelogger("../logs/propagation_grav_baro_mag_gps_state.csv");
    CSVLogger covlogger("../logs/propagation_grav_baro_mag_gps_cov.csv");

    Matrix<float, 3, 1> gyroMeas;
    Matrix<float, 3, 1> accMeas;
    Matrix<float, 3, 1> magMeas;

    Matrix<float, 3, 1> posMeas;
    Matrix<float, 3, 1> velMeas;

    float dt = .0025;
    IMUSensorDefinition nativeSuite = defaults::DefaultIMU();
    MEKF nav(nativeSuite);


    for (int i = 0; i < dataset.rows(); i++)
    {
        statelogger.writeRow(nav.getFilterState().transpose());
        covlogger.writeRow(nav.getFilterCovariance().transpose());

        gyroMeas = dataset.block<1, 3>(i, 0).cast<float>();
        accMeas = dataset.block<1, 3>(i, 3).cast<float>();
        magMeas = dataset.block<1, 3>(i, 6).cast<float>();

        posMeas = dataset.block<1, 3>(i, 9).cast<float>();
        velMeas = dataset.block<1, 3>(i, 12).cast<float>();

        nav.propagate(gyroMeas, accMeas, dt);

        nav.updateWithGravity(accMeas);

        if ((i + 1) % 4 == 0)
        {
            nav.updateWithMagnetometer(magMeas);
        }

        if ((i + 1) % 15 == 0)
        {
            nav.updateWithBarometer(-posMeas(2));
        }

        if ((i + 1) % 80 == 0)
        {
            nav.updateWithGPS(posMeas, velMeas);
        }
    }

    cout << "Sensor sim complete" << endl;

    return EXIT_SUCCESS;
}
