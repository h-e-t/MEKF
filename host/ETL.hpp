#ifndef ETL_H
#define ETL_H

#include "Eigen/Core"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <vector>

using std::cout, std::endl;

class ETL
{
    std::string dataset;
    std::string delimiter;
    bool header;

  public:
    ETL(std::string data, std::string separator, bool head)
        : dataset(data), delimiter(separator), header(head)
    {
    }

    Eigen::MatrixXd readCSV();

    // Eigen::MatrixXd CSVtoEigen(std::vector<std::vector<std::string>> dataset);

    auto Mean(Eigen::MatrixXd data) -> decltype(data.colwise().mean().eval());

    auto Std(Eigen::MatrixXd data)
        -> decltype(((data.array().square().colwise().sum()) / (data.rows() - 1))
                        .sqrt()
                        .eval());

    Eigen::MatrixXd Normalize(Eigen::MatrixXd data, bool normalizeTarget);
};

#endif
