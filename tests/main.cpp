#include "ETL.hpp"
#include "Eigen/Core"
#include <cstdlib>
#include <vector>

using std::cout, std::endl;

int main(int argc, char *argv[]) {
  // Arg1  dataset path
  // Arg2  delimeter
  // Arg3  wether or not there is a head
  ETL etl(argv[1], argv[2], argv[3]);

  std::vector<std::vector<std::string>> dataset = etl.readCSV();

  Eigen::Vector3d v()

  
  int rows = dataset.size();
  int columns = dataset[0].size();

  Eigen::MatrixXd dataMat = etl.CSVtoEigen(dataset, rows, columns);

  cout << dataMat;

  return EXIT_SUCCESS;
}
