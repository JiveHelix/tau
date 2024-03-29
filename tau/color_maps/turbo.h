/**
  * @file turbo.h
  *
  * @brief Uses polynomial coefficients to create the Turbo colormap with an
  * arbitrary number of steps.
  *
  * @author Jive Helix (jivehelix@gmail.com)
  * @date 07 Feb 2022
  * @copyright Jive Helix
  * Licensed under the MIT license. See LICENSE file.
**/


#pragma once


#include "tau/eigen.h"
#include "tau/planar.h"
#include "tau/color.h"
#include "tau/horner.h"
#include "tau/color_maps/rgb.h"


namespace tau
{

namespace turbo
{

namespace factors
{

    static const auto red = Vector(
        0.638095,
        1.50192,
        -1.95788,
        1.94068,
        13.3149,
        -52.036,
        -35.0841,
        218.445,
        39.6986,
        -446.202,
        -14.08,
        495.862,
        -7.0707,
        -287.713,
        4.87735,
        68.3481);

    static const auto green = Vector(
        0.990193,
        -0.275077,
        -1.96421,
        0.412521,
        5.73317,
        -2.29317,
        -30.8389,
        7.7321,
        89.4262,
        -11.3136,
        -130.427,
        7.75726,
        93.1419,
        -2.0479,
        -26.0202);

    static const auto blue = Vector(
        0.236484,
        -0.758137,
        4.44364,
        0.255715,
        -34.9887,
        3.5434,
        168.725,
        -72.6929,
        -488.495,
        359.208,
        836.368,
        -798.283,
        -833.524,
        922.711,
        447.258,
        -543.061,
        -99.9034,
        128.968);

    static const auto linearMap = LinearMap<double>{{0.0, 1.0}, {-1.0, 1.0}};

} // end namespace factors


inline
RgbMatrix<double> MakeRgbFloat(size_t count)
{
    assert(count
        <= static_cast<size_t>(std::numeric_limits<Eigen::Index>::max()));

    using VectorType = Eigen::Vector<double, Eigen::Dynamic>;

    VectorType x = VectorType::LinSpaced(
        static_cast<Eigen::Index>(count),
        0.0,
        1.0);

    auto planar = Planar<3, double, Eigen::Dynamic, 1>(
        static_cast<Eigen::Index>(count),
        1);

    GetRed(planar) = Horner(x, factors::red, factors::linearMap);
    GetGreen(planar) = Horner(x, factors::green, factors::linearMap);
    GetBlue(planar) = Horner(x, factors::blue, factors::linearMap);

    return planar.GetInterleaved<Eigen::RowMajor>();
}


inline
RgbMatrix<uint8_t> MakeRgb8(size_t count)
{
    return MakeRgb<uint8_t>(count, MakeRgbFloat);
}


} // end namespace turbo


} // end namespace tau
