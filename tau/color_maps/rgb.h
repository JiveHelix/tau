#pragma once


#include "Eigen/Dense"


namespace tau
{


template<typename T>
using RgbMatrix = Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>;


} // end namespace tau
