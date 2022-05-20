/**
  * @file gray.h
  * 
  * @brief Maps values to grayscale.
  * 
  * @author Jive Helix (jivehelix@gmail.com)
  * @date 07 Feb 2022
  * @copyright Jive Helix
  * Licensed under the MIT license. See LICENSE file.
**/


#pragma once


#include "tau/eigen.h"
#include "tau/power_series.h"
#include "tau/color_maps/rgb.h"


namespace tau
{

namespace gray
{


inline
RgbMatrix<double> MakeRgbFloat(size_t count)
{
    assert(count <= std::numeric_limits<Eigen::Index>::max());

    using VectorType = Eigen::Vector<double, Eigen::Dynamic>;

    VectorType x = VectorType::LinSpaced(
        static_cast<Eigen::Index>(count),
        0.0,
        1.0);

    RgbMatrix<double> result(count, 3);
    result << x, x, x;
    
    return result;
}


inline
RgbMatrix<uint8_t> MakeRgb8(size_t count)
{
    return MakeRgb<uint8_t>(count, MakeRgbFloat);
}


} // end namespace gray


} // end namespace tau
