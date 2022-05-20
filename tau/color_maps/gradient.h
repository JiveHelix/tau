#pragma once


#include "tau/eigen.h"
#include "tau/planar.h"
#include "tau/color.h"
#include "tau/color_maps/rgb.h"


namespace tau
{


namespace gradient
{


template<typename F>
using Hsv = Eigen::Vector<F, 3>;


template<typename F>
using Planes = Planar<3, F, Eigen::Dynamic, 1>;


template<typename F>
Planes<F> MakeHsvGradient(
    size_t count_,
    const Hsv<F> &first,
    const Hsv<F> &last)
{
    auto count = static_cast<Eigen::Index>(count_);

    using VectorType = Eigen::Vector<double, Eigen::Dynamic>;

    auto hsv = Planes<F>(count, 1);

    using namespace index;

    GetHue(hsv) = VectorType::LinSpaced(count, first(hue), last(hue));

    GetSaturation(hsv) =
        VectorType::LinSpaced(count, first(saturation), last(saturation));

    GetValue(hsv) = VectorType::LinSpaced(count, first(value), last(value));

    return hsv;
}


template<typename I, typename F>
Planes<I> MakeRgbGradientFromHsv(
    size_t count,
    const Hsv<F> &first,
    const Hsv<F> &last)
{
    return HsvToRgb<I>(MakeHsvGradient(count, first, last));
}


template<typename I, typename F>
RgbMatrix<I> MakeColormap(
    size_t count,
    const Hsv<F> &first,
    const Hsv<F> &last)
{
    Planes<I> planes = MakeRgbGradientFromHsv<I>(count, first, last);
    return planes.template GetInterleaved<Eigen::RowMajor>();
}


} // end namespace gradient


} // end namespace tau
