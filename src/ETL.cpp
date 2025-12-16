#include "ETL.hpp"
#include "Eigen/Core"
#include <ostream>
#include <string>

Eigen::MatrixXd ETL::readCSV()
{
    std::ifstream file(dataset);
    std::vector<std::vector<std::string>> dataString;

    std::string line = "";

    while (getline(file, line))
    {
        std::vector<std::string> vec;
        std::stringstream ss(line);
        std::string field;
        std::string temp;

        while (std::getline(ss, temp, delimiter.front()))
        {
            vec.push_back(temp);
        }

        dataString.push_back(vec);
    }

    int rows = dataString.size();
    int cols = dataString[0].size();

    if (header == true)
    {
        rows = rows - 1;
    }

    Eigen::MatrixXd mat(cols, rows);

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; ++j)
        {
            mat(j, i) = atof(dataString[i][j].c_str());
        }
    }

    return mat.transpose();
}

auto ETL::Mean(Eigen::MatrixXd data) -> decltype(data.colwise().mean().eval())
{
    return data.colwise().mean().eval();
}

auto ETL::Std(Eigen::MatrixXd data)
    -> decltype(((data.array().square().colwise().sum()) / (data.rows() - 1))
                    .sqrt()
                    .eval())
{
    cout << "calculating standard deviation" << endl;
    return ((data.array().square().colwise().sum()) / (data.rows() - 1)).sqrt().eval();
}

Eigen::MatrixXd ETL::Normalize(Eigen::MatrixXd data, bool normalizeTarget)
{
    cout << "Normalizing" << endl;
    Eigen::MatrixXd dataNorm;
    if (normalizeTarget == true)
    {
        dataNorm = data;
    }
    else
    {
        dataNorm = data.leftCols(data.cols() - 1);
    }

    auto mean = Mean(dataNorm);
    Eigen::MatrixXd scaled_data = dataNorm.rowwise() - mean;
    auto std = Std(scaled_data);

    Eigen::MatrixXd norm = scaled_data.array().rowwise() / std;

    if (normalizeTarget == false)
    {
        norm.conservativeResize(norm.rows(), norm.cols() + 1);
        norm.col(norm.cols() - 1) = data.rightCols(1);
    }

    return norm;
}
