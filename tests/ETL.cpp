#include "ETL.hpp"
#include "Eigen/Core"

std::vector<std::vector<std::string>> ETL::readCSV() {
  using std::cout, std::endl;
  std::ifstream file(dataset);
  std::vector<std::vector<std::string>> dataString;

  std::string line = "";

  while (getline(file, line)) {
    std::vector<std::string> vec;
    std::stringstream ss(line);
    std::string field;
    std::string temp;

    while (std::getline(ss, temp, delimiter.front())) {
      vec.push_back(temp);
    }

    dataString.push_back(vec);
  }

  return dataString;
}

Eigen::MatrixXd ETL::CSVtoEigen(std::vector<std::vector<std::string>> dataset,
                                int rows, int cols) {
  if (header) {
    rows = rows - 1;
  }

  Eigen::MatrixXd mat(cols, rows);

  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; ++j) {
      mat(j, i) = atof(dataset[i][j].c_str());
    }
  }

  return mat.transpose();
}
