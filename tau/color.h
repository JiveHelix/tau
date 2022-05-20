#pragma once

#include "tau/eigen.h"
#include "tau/planar.h"
#include "tau/angles.h"


namespace tau
{


namespace index
{
    static constexpr size_t hue = 0;
    static constexpr size_t saturation = 1;
    static constexpr size_t value = 2;

    static constexpr size_t red = 0;
    static constexpr size_t green = 1;
    static constexpr size_t blue = 2;
};


/** 
 ** GetType returns by const reference or non-const reference depending on the
 ** const-ness of Planes.
 **/
template<typename T>
struct GetType_;


template<typename T>
struct GetType_<const T &>
{
    using type = const typename T::Matrix &;
};


template<typename T>
struct GetType_<T &>
{
    using type = typename T::Matrix &;
};


template<typename T>
using GetType = typename GetType_<T>::type;


template<typename Planes>
auto GetHue(Planes &&planes) -> GetType<Planes>
{
    return std::get<index::hue>(std::forward<Planes>(planes).planes);
}

template<typename Planes>
auto GetSaturation(Planes &&planes) -> GetType<Planes>
{
    return std::get<index::saturation>(std::forward<Planes>(planes).planes);
}

template<typename Planes>
auto GetValue(Planes &&planes) -> GetType<Planes>
{
    return std::get<index::value>(std::forward<Planes>(planes).planes);
}

template<typename Planes, typename Matrix>
void SetHue(Planes &planes, const Matrix &hue)
{
    std::get<index::hue>(planes.planes) = hue;
}

template<typename Planes, typename Matrix>
void SetSaturation(Planes &planes, const Matrix &saturation)
{
    std::get<index::saturation>(planes.planes) = saturation;
}

template<typename Planes, typename Matrix>
void SetValue(Planes &planes, const Matrix &value)
{
    std::get<index::value>(planes.planes) = value;
}

template<typename Planes>
auto GetRed(Planes &&planes) -> GetType<Planes>
{
    return std::get<index::red>(std::forward<Planes>(planes).planes);
}

template<typename Planes>
auto GetGreen(Planes &&planes) -> GetType<Planes>
{
    return std::get<index::green>(std::forward<Planes>(planes).planes);
}

template<typename Planes>
auto GetBlue(Planes &&planes) -> GetType<Planes>
{
    return std::get<index::blue>(std::forward<Planes>(planes).planes);
}

template<typename Planes, typename Matrix>
void SetRed(Planes &planes, const Matrix &red)
{
    std::get<index::red>(planes.planes) = red;
}

template<typename Planes, typename Matrix>
void SetGreen(Planes &planes, const Matrix &green)
{
    std::get<index::green>(planes.planes) = green;
}

template<typename Planes, typename Matrix>
void SetBlue(Planes &planes, const Matrix &blue)
{
    std::get<index::blue>(planes.planes) = blue;
}


template<typename F, typename I>
Eigen::Vector3<F> RgbToHsv(const Eigen::Vector3<I> &rgb)
{
    static_assert(std::is_floating_point_v<F>);

    Eigen::Vector3<F> rgbFloat = rgb.template cast<F>();
    
    if constexpr (std::is_integral_v<I>)
    {
        rgbFloat.array() /= std::numeric_limits<I>::max();
    }
    
    Eigen::Index maxIndex;
    auto Cmax = rgbFloat.maxCoeff(&maxIndex);
    auto Cmin = rgbFloat.minCoeff();
    auto delta = Cmax - Cmin;
    
    Eigen::Vector3<F> hsv;

    using namespace index;

    if (delta == 0.0)
    {
        hsv(0) = 0.0;    
    }
    else
    {
        switch (maxIndex)
        {
            case 0:
                hsv(hue) = ((rgbFloat(green) - rgbFloat(blue)) / delta);
                break;

            case 1:
                hsv(hue) = 2 + ((rgbFloat(blue) - rgbFloat(red)) / delta);
                break;

            case 2:
                hsv(hue) = 4 + ((rgbFloat(red) - rgbFloat(green)) / delta);
                break;

            default:
                throw std::logic_error("We only have 3 colors");
        }
    }

    hsv(hue) = 60.0 * (std::fmod(hsv(hue) + 6.0, 6.0));

    if (Cmax == 0.0)
    {
        hsv(saturation) = 0.0;
    }
    else
    {
        hsv(saturation) = delta / Cmax;
    }

    hsv(value) = Cmax;

    if constexpr (std::is_integral_v<I>)
    {
        F rounder;

        if constexpr (sizeof(I) == 1)
        {
            // Round to 3 decimals.
            rounder = static_cast<F>(1000);

        }
        else if constexpr (sizeof(I) == 2)
        {
            // Round to 6 decimals
            rounder = static_cast<F>(1000000);
        }

        hsv = (hsv.array() * rounder).round() / rounder;
    }

    return hsv;
}


template<typename F, typename I, int rows, int columns, int options>
auto RgbToHsv(const Planar<3, I, rows, columns, options> &rgb)
{
    static_assert(std::is_floating_point_v<F>);

    using PlanarT = Planar<3, F, rows, columns, options>;
    using Matrix = typename PlanarT::Matrix;
    using Array = ArrayLike<Matrix>;

    PlanarT hsv(rgb.GetRowCount(), rgb.GetColumnCount());

    PlanarT rgbFloat = rgb.template Cast<F>();
    
    if constexpr (std::is_integral_v<I>)
    {
        GetRed(rgbFloat).array() /= std::numeric_limits<I>::max();
        GetGreen(rgbFloat).array() /= std::numeric_limits<I>::max();
        GetBlue(rgbFloat).array() /= std::numeric_limits<I>::max();
    }

    typename PlanarT::ExtremaIndices indices(
        rgb.GetRowCount(),
        rgb.GetColumnCount());

    auto extrema = rgbFloat.GetExtrema(&indices);

    Array delta = extrema.planes[1] - extrema.planes[0];

    auto mask0 = (indices.planes[1].array() == 0);
    auto mask1 = (indices.planes[1].array() == 1);

    GetHue(hsv) = (delta == 0.0).select(
        0.0,
        mask0.select(
            (GetGreen(rgbFloat) - GetBlue(rgbFloat)).array() / delta,
            mask1.select(
                2 + (GetBlue(rgbFloat) - GetRed(rgbFloat)).array() / delta,
                4 + (GetRed(rgbFloat) - GetGreen(rgbFloat)).array() / delta)));

    GetHue(hsv) = GetHue(hsv).array() + 6.0;
    GetHue(hsv) = 60.0 * tau::Modulo(GetHue(hsv), 6.0).array();

    GetSaturation(hsv) =
        (extrema.planes[1].array() == 0.0).select(
            0.0,
            delta / extrema.planes[1].array());

    GetValue(hsv) = extrema.planes[1];

    if constexpr (std::is_integral_v<I>)
    {
        if constexpr (sizeof(I) == 1)
        {
            // Round to 3 decimals.
            hsv.template Round<3>();
        }
        else if constexpr (sizeof(I) == 2)
        {
            // Round to 6 decimals
            hsv.template Round<6>();
        }
    }

    return hsv;
}


template<typename I, typename F>
Eigen::Vector3<I> HsvToRgb(const Eigen::Vector3<F> &hsv)
{
    static_assert(std::is_floating_point_v<F>);

    static constexpr auto oneSixth = Angles<F>::tauDegrees / F{6};

    auto hue = hsv(0) / oneSixth;
    auto saturation = hsv(1);
    auto value = hsv(2);

    auto C = value * saturation;
    auto X = C * (1 - std::abs(std::fmod(hue, 2.0) - 1));
    auto m = value - C;

    Eigen::Vector3<F> rgb;

    int hueFloor = static_cast<int>(hue);

    switch (hueFloor)
    {
        case 0:
            rgb << C, X, 0;
            break;

        case 1:
            rgb << X, C, 0;
            break;

        case 2:
            rgb << 0, C, X;
            break;

        case 3:
            rgb << 0, X, C;
            break;

        case 4:
            rgb << X, 0, C;
            break;

        case 5:
        case 6:
            rgb << C, 0, X;
            break;

        default:
            throw std::logic_error("Hue must be 360 or less");
    }

    rgb.array() += m;

    if constexpr (std::is_floating_point_v<I>)
    {
        return rgb.template cast<I>();
    }
    else
    {
        rgb.array() *= std::numeric_limits<I>::max();
        rgb = rgb.array().round();

        return rgb.template cast<I>();
    }
}


template<typename T, int rows, int columns, int options>
auto HsvToRgbFloat(const Planar<3, T, rows, columns, options> &hsv)
{
    static_assert(
        std::is_floating_point_v<T>,
        "Data must be floating point.");

    using PlanarT = Planar<3, T, rows, columns, options>;
    using Matrix = typename PlanarT::Matrix;
    using Array = ArrayLike<Matrix>;

    static constexpr auto oneSixth = Angles<T>::tauDegrees / T{6};
    static constexpr auto two = T{2};

    PlanarT rgb(hsv.GetRowCount(), hsv.GetColumnCount());

    Matrix H = GetHue(hsv).array() / oneSixth;
    const Array &S = GetSaturation(hsv);
    const Array &V = GetValue(hsv);

    auto mod = tau::Modulo(H, two);

    Array C = V * S;
    Array X = C * (1 - (mod.array() - 1).abs());
    Array m = V - C;

    auto hue = H.array();

    auto mask0 = hue >= T{0} && hue < T{1};
    auto mask1 = hue >= T{1} && hue < T{2};
    auto mask2 = hue >= T{2} && hue < T{3};
    auto mask3 = hue >= T{3} && hue < T{4};
    auto mask4 = hue >= T{4} && hue < T{5};
    auto mask5 = hue >= T{5} && hue <= T{6};

    /*

    if (mask0 || mask5)
        red = C
    else
        if (mask1 || mask4)
            red = X
        else // (mask2 || mask3)
            red = 0
    */

    GetRed(rgb) = (mask0 || mask5).select(
        C,
        (mask1 || mask4).select(
            X,
            T{0}));

    GetGreen(rgb) = (mask1 || mask2).select(
        C,
        (mask0 || mask3).select(
            X,
            T{0}));

    GetBlue(rgb) = (mask3 || mask4).select(
        C,
        (mask2 || mask5).select(
            X,
            T{0}));

    GetRed(rgb).array() += m;
    GetGreen(rgb).array() += m;
    GetBlue(rgb).array() += m;

    return rgb;
}


template
<
    typename Target,
    typename T,
    int rows,
    int columns,
    int options
>
auto HsvToRgb(const Planar<3, T, rows, columns, options> &hsv)
{
    static_assert(std::is_integral_v<Target>);

    auto rgb = HsvToRgbFloat(hsv);
    
    GetRed(rgb).array() *= std::numeric_limits<Target>::max();
    GetGreen(rgb).array() *= std::numeric_limits<Target>::max();
    GetBlue(rgb).array() *= std::numeric_limits<Target>::max();
    
    GetRed(rgb) = GetRed(rgb).array().round();
    GetGreen(rgb) = GetGreen(rgb).array().round();
    GetBlue(rgb) = GetBlue(rgb).array().round();

    return rgb.template Cast<Target>();
}


} // end namespace tau
