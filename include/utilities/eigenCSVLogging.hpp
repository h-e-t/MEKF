#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <string>

class CSVLogger
{
  public:
    CSVLogger(const std::string &filename)
    {
        file_.open(filename);
        if (!file_.is_open())
            throw std::runtime_error("Could not open CSV file");
    }

    ~CSVLogger()
    {
        if (file_.is_open())
            file_.close();
    }

    template <typename Derived> void writeRow(const Eigen::MatrixBase<Derived> &vec)
    {
        const int N = vec.size();
        for (int i = 0; i < N; ++i)
        {
            file_ << vec(i);
            if (i + 1 != N)
                file_ << ",";
        }
        file_ << "\n";
    }

    void writeHeader(const std::vector<std::string> &labels)
    {
        for (size_t i = 0; i < labels.size(); ++i)
        {
            file_ << labels[i];
            if (i + 1 != labels.size())
                file_ << ",";
        }
        file_ << "\n";
    }

  private:
    std::ofstream file_;
};
