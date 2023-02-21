#pragma once


#include "tau/eigen_shim.h"


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

template<typename T>
struct RgbPixels
{
    using Data = RgbMatrix<T>;

    Data data;

    // The PixelMatrix is stored as a count x 3 matrix, where each row contains
    // an RGB triplet.
    // height and width store the dimensions the image.
    // Expect height * width == data.rows()
    Eigen::Index height;
    Eigen::Index width;
};


} // end namespace tau
