#pragma once

#include <Eigen/Dense>
#include <core/mekf_core.hpp>

template <typename Derived> Derived inline rad2deg(const Eigen::MatrixBase<Derived> &v)
{
    return v * (180.0 / PI);
};

template <typename Derived> Derived inline deg2rad(const Eigen::MatrixBase<Derived> &v)
{
    return v * (PI / 180.0);
};
