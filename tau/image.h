#pragma once


#include <tau/eigen.h>


namespace tau
{


template<typename T>
using ImageMatrix = Eigen::Matrix
<
    T,
    Eigen::Dynamic,
    Eigen::Dynamic,
    Eigen::RowMajor
>;


using ImageMatrixInt = ImageMatrix<int32_t>;
using ImageMatrixFloat = ImageMatrix<float>;



} // end namespace tau
