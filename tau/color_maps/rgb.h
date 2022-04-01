#pragma once


#include "Eigen/Dense"


namespace tau
{


template<typename T>
using RgbMatrix = Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>;


template<typename T, typename MakeFloat>
RgbMatrix<T> MakeRgb(size_t count, MakeFloat makeFloat)
{
    RgbMatrix<double> asFloat = makeFloat(count);

    T maximum = std::numeric_limits<T>::max();

    RgbMatrix<T> converted{
        (asFloat * maximum).array().round().template cast<T>()};

    return converted;
}


} // end namespace tau
