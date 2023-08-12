#pragma once


#include "tau/eigen_shim.h"
#include "tau/size.h"


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
    using Index = Eigen::Index;

    Data data;

    // The PixelMatrix is stored as a count x 3 matrix, where each row contains
    // an RGB triplet.
    // height and width store the dimensions the image.
    // Expect height * width == data.rows()
    Size<Index> size;

    static RgbPixels<T> Create(const Size<Index> &size)
    {
        RgbPixels<T> result;
        result.size = size;
        result.data = Data::Zero(size.height * size.width, 3);

        return result;
    }

    static std::shared_ptr<RgbPixels<T>> CreateShared(const Size<Index> &size)
    {
        auto result = std::make_shared<RgbPixels<T>>();
        result->size = size;
        result->data = Data::Zero(size.height * size.width, 3);

        return result;
    }
};


} // end namespace tau
