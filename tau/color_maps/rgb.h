#pragma once


#include "tau/eigen_shim.h"
#include "tau/size.h"
#include "tau/color.h"


namespace tau
{


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
