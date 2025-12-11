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

class ETL {
  std::string dataset;
  std::string delimiter;
  bool header;

public:
  ETL(std::string data, std::string separator, bool head)
      : dataset(data), delimiter(separator), header(head) {}

  std::vector<std::vector<std::string>> readCSV();

  Eigen::MatrixXd CSVtoEigen(std::vector<std::vector<std::string>> dataset,
                             int rows, int cols);
};

#endif
