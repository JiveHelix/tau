#pragma once

#include <fields/fields.h>
#include <pex/group.h>

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

    static constexpr size_t alpha = 3;
};


template<typename T>
using ColorVector = Eigen::Vector3<T>;


template<typename T>
using AlphaVector = Eigen::Vector<T, 4>;


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

template<typename Planes>
auto GetAlpha(Planes &&planes) -> GetType<Planes>
{
    return std::get<index::alpha>(std::forward<Planes>(planes).planes);
}

template<typename T, int count>
auto GetHue(const Eigen::Vector<T, count> &vector)
{
    return vector(index::hue);
}

template<typename T, int count>
auto GetSaturation(const Eigen::Vector<T, count> &vector)
{
    return vector(index::saturation);
}

template<typename T, int count>
auto GetValue(const Eigen::Vector<T, count> &vector)
{
    return vector(index::value);
}

template<typename T, int count>
auto GetAlpha(const Eigen::Vector<T, count> &vector)
{
    static_assert(count == 4);
    return vector(index::alpha);
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

template<typename T, int count>
auto GetRed(const Eigen::Vector<T, count> &vector)
{
    return vector(index::red);
}

template<typename T, int count>
auto GetGreen(const Eigen::Vector<T, count> &vector)
{
    return vector(index::green);
}

template<typename T, int count>
auto GetBlue(const Eigen::Vector<T, count> &vector)
{
    return vector(index::blue);
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


template<typename F, typename I, int count>
Eigen::Vector<F, count> RgbToHsv(const Eigen::Vector<I, count> &rgb)
{
    static_assert(std::is_floating_point_v<F>);

    Eigen::Vector<F, count> rgbFloat = rgb.template cast<F>();

    if constexpr (std::is_integral_v<I>)
    {
        rgbFloat.array() /= std::numeric_limits<I>::max();
    }

    Eigen::Index maxIndex;
    auto Cmax = rgbFloat.template head<3>().maxCoeff(&maxIndex);
    auto Cmin = rgbFloat.template head<3>().minCoeff();
    auto delta = Cmax - Cmin;

    Eigen::Vector<F, count> hsv;

    using namespace index;

    if (delta == 0.0)
    {
        hsv(index::hue) = 0.0;
    }
    else
    {
        switch (maxIndex)
        {
            case 0:
                hsv(index::hue) = ((rgbFloat(green) - rgbFloat(blue)) / delta);
                break;

            case 1:
                hsv(index::hue)
                    = 2 + ((rgbFloat(blue) - rgbFloat(red)) / delta);
                break;

            case 2:
                hsv(index::hue) =
                    4 + ((rgbFloat(red) - rgbFloat(green)) / delta);
                break;

            default:
                throw std::logic_error("We only have 3 colors");
        }
    }

    hsv(index::hue) = 60.0 * (std::fmod(hsv(index::hue) + 6.0, 6.0));

    if (Cmax == 0.0)
    {
        hsv(index::saturation) = 0.0;
    }
    else
    {
        hsv(index::saturation) = delta / Cmax;
    }

    hsv(index::value) = Cmax;

    if constexpr (count == 4)
    {
        hsv(index::alpha) = rgbFloat(index::alpha);
    }

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


template
<
    typename F,
    size_t count,
    typename I,
    int rows,
    int columns,
    int options
>
auto RgbToHsv(const Planar<count, I, rows, columns, options> &rgb)
{
    static_assert(std::is_floating_point_v<F>);

    using PlanarT = Planar<count, F, rows, columns, options>;
    using Matrix = typename PlanarT::Matrix;
    using Array = ArrayLike<Matrix>;

    PlanarT hsv(rgb.GetRowCount(), rgb.GetColumnCount());

    PlanarT rgbFloat = rgb.template Cast<F>();

    if constexpr (std::is_integral_v<I>)
    {
        GetRed(rgbFloat).array() /= std::numeric_limits<I>::max();
        GetGreen(rgbFloat).array() /= std::numeric_limits<I>::max();
        GetBlue(rgbFloat).array() /= std::numeric_limits<I>::max();

        if constexpr (count == 4)
        {
            GetAlpha(rgbFloat).array() /= std::numeric_limits<I>::max();
        }
    }

    typename PlanarT::ExtremaIndices indices(
        rgb.GetRowCount(),
        rgb.GetColumnCount());

    typename PlanarT::Extrema extrema;

    if constexpr (count == 4)
    {
        // Only the first three planes (red, green, and blue) are used in the
        // conversion.
        extrema = rgbFloat.GetExtrema(std::index_sequence<0, 1, 2>(), &indices);
    }
    else
    {
        extrema = rgbFloat.GetExtrema(&indices);
    }

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

    if constexpr (count == 4)
    {
        GetAlpha(hsv) = GetAlpha(rgbFloat);
    }

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


template<typename I, typename F, int count>
Eigen::Vector<I, count> HsvToRgb(const Eigen::Vector<F, count> &hsv)
{
    static_assert(std::is_floating_point_v<F>);

    static constexpr auto oneSixth = Angles<F>::tauDegrees / F{6};

    auto hue = static_cast<F>(hsv(0) / oneSixth);
    auto saturation = hsv(1);
    auto value = hsv(2);

    auto C = static_cast<F>(value * saturation);
    auto X = static_cast<F>(C * (1 - std::abs(std::fmod(hue, 2.0) - 1)));
    auto m = static_cast<F>(value - C);

    Eigen::Vector<F, count> rgb;

    int hueFloor = static_cast<int>(hue);

    switch (hueFloor)
    {
        case 0:
            rgb.template head<3>() << C, X, 0;
            break;

        case 1:
            rgb.template head<3>() << X, C, 0;
            break;

        case 2:
            rgb.template head<3>() << 0, C, X;
            break;

        case 3:
            rgb.template head<3>() << 0, X, C;
            break;

        case 4:
            rgb.template head<3>() << X, 0, C;
            break;

        case 5:
        case 6:
            rgb.template head<3>() << C, 0, X;
            break;

        default:
            throw std::logic_error("Hue must be 360 or less");
    }

    rgb.template head<3>().array() += m;

    if constexpr (count == 4)
    {
        rgb(index::alpha) = hsv(index::alpha);
    }

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


template<size_t count, typename T, int rows, int columns, int options>
auto HsvToRgbFloat(const Planar<count, T, rows, columns, options> &hsv)
{
    static_assert(
        std::is_floating_point_v<T>,
        "Data must be floating point.");

    using PlanarT = Planar<count, T, rows, columns, options>;
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

    if constexpr (count == 4)
    {
        GetAlpha(rgb) = GetAlpha(hsv);
    }

    return rgb;
}


template
<
    typename Target,
    size_t count,
    typename T,
    int rows,
    int columns,
    int options
>
auto HsvToRgb(const Planar<count, T, rows, columns, options> &hsv)
{
    static_assert(std::is_integral_v<Target>);

    auto rgb = HsvToRgbFloat(hsv);

    GetRed(rgb).array() *= std::numeric_limits<Target>::max();
    GetGreen(rgb).array() *= std::numeric_limits<Target>::max();
    GetBlue(rgb).array() *= std::numeric_limits<Target>::max();

    GetRed(rgb) = GetRed(rgb).array().round();
    GetGreen(rgb) = GetGreen(rgb).array().round();
    GetBlue(rgb) = GetBlue(rgb).array().round();

    if constexpr (count == 4)
    {
        GetAlpha(rgb).array() *= std::numeric_limits<Target>::max();
        GetAlpha(rgb) = GetAlpha(rgb).array().round();
    }

    return rgb.template Cast<Target>();
}


template<typename T>
struct HsvFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::hue, "hue"),
        fields::Field(&T::saturation, "saturation"),
        fields::Field(&T::value, "value"));
};

template<typename U>
struct HsvTemplate
{
    template<template<typename> typename V>
    struct MemberTemplate
    {
        V<pex::MakeRange<U, pex::Limit<0>, pex::Limit<360>>> hue;
        V<pex::MakeRange<U, pex::Limit<0>, pex::Limit<1>>> saturation;
        V<pex::MakeRange<U, pex::Limit<0>, pex::Limit<1>>> value;

        static constexpr auto fields = HsvFields<MemberTemplate>::fields;
    };
};


template<typename T>
struct Hsv: public HsvTemplate<T>::template MemberTemplate<pex::Identity>
{
    template<typename U>
    static Hsv FromVector(const ColorVector<U> &hsv)
    {
        return {
            static_cast<T>(GetHue(hsv)),
            static_cast<T>(GetSaturation(hsv)),
            static_cast<T>(GetValue(hsv))};
    }

    ColorVector<T> ToVector() const
    {
        return {this->hue, this->saturation, this->value};
    }
};


DECLARE_OUTPUT_STREAM_OPERATOR(Hsv<float>)
DECLARE_OUTPUT_STREAM_OPERATOR(Hsv<double>)
DECLARE_COMPARISON_OPERATORS(Hsv<float>)
DECLARE_COMPARISON_OPERATORS(Hsv<double>)


template<typename T>
struct HsvaFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::hue, "hue"),
        fields::Field(&T::saturation, "saturation"),
        fields::Field(&T::value, "value"),
        fields::Field(&T::alpha, "alpha"));
};


template<typename U>
struct HsvaTemplate
{
    template<template<typename> typename V>
    struct MemberTemplate
    {
        V<pex::MakeRange<U, pex::Limit<0>, pex::Limit<360>>> hue;
        V<pex::MakeRange<U, pex::Limit<0>, pex::Limit<1>>> saturation;
        V<pex::MakeRange<U, pex::Limit<0>, pex::Limit<1>>> value;
        V<pex::MakeRange<U, pex::Limit<0>, pex::Limit<1>>> alpha;

        static constexpr auto fields = HsvaFields<MemberTemplate>::fields;
    };
};


template<typename T>
struct Hsva: public HsvaTemplate<T>::template MemberTemplate<pex::Identity>
{
    template<typename U>
    static Hsva FromVector(const AlphaVector<U> &hsva)
    {
        return {
            static_cast<T>(GetHue(hsva)),
            static_cast<T>(GetSaturation(hsva)),
            static_cast<T>(GetValue(hsva)),
            static_cast<T>(GetAlpha(hsva))};
    }

    AlphaVector<T> ToVector() const
    {
        return {this->hue, this->saturation, this->value, this->alpha};
    }
};


DECLARE_OUTPUT_STREAM_OPERATOR(Hsva<float>)
DECLARE_OUTPUT_STREAM_OPERATOR(Hsva<double>)
DECLARE_COMPARISON_OPERATORS(Hsva<float>)
DECLARE_COMPARISON_OPERATORS(Hsva<double>)


template<typename T>
struct RgbFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::red, "red"),
        fields::Field(&T::green, "green"),
        fields::Field(&T::blue, "blue"));
};


template<typename U>
struct RgbTemplate
{
    template<template<typename> typename V>
    struct MemberTemplate
    {
        V<pex::MakeRange<U>> red;
        V<pex::MakeRange<U>> green;
        V<pex::MakeRange<U>> blue;

        static constexpr auto fields = RgbFields<MemberTemplate>::fields;
        static constexpr auto fieldsTypeName = "Rgb";
    };
};


template<typename T>
struct Rgb: public RgbTemplate<T>::template MemberTemplate<pex::Identity>
{
    template<typename U>
    static Rgb FromVector(const ColorVector<U> &rgb)
    {
        return {
            static_cast<T>(GetRed(rgb)),
            static_cast<T>(GetGreen(rgb)),
            static_cast<T>(GetBlue(rgb))};
    }

    ColorVector<T> ToVector() const
    {
        return {this->red, this->green, this->blue};
    }
};


DECLARE_OUTPUT_STREAM_OPERATOR(Rgb<uint8_t>)
DECLARE_OUTPUT_STREAM_OPERATOR(Rgb<uint16_t>)
DECLARE_COMPARISON_OPERATORS(Rgb<uint8_t>)
DECLARE_COMPARISON_OPERATORS(Rgb<uint16_t>)


template<typename T>
struct RgbaFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::red, "red"),
        fields::Field(&T::green, "green"),
        fields::Field(&T::blue, "blue"),
        fields::Field(&T::alpha, "alpha"));
};


template<typename U>
struct RgbaTemplate
{
    template<template<typename> typename V>
    struct MemberTemplate
    {
        V<pex::MakeRange<U>> red;
        V<pex::MakeRange<U>> green;
        V<pex::MakeRange<U>> blue;
        V<pex::MakeRange<U>> alpha;

        static constexpr auto fields = RgbaFields<MemberTemplate>::fields;
        static constexpr auto fieldsTypeName = "Rgba";
    };
};


template<typename T>
struct Rgba: public RgbaTemplate<T>::template MemberTemplate<pex::Identity>
{
    template<typename U>
    static Rgba FromVector(const AlphaVector<U> &rgba)
    {
        return {
            static_cast<T>(GetRed(rgba)),
            static_cast<T>(GetGreen(rgba)),
            static_cast<T>(GetBlue(rgba)),
            static_cast<T>(GetAlpha(rgba))};
    }

    AlphaVector<T> ToVector() const
    {
        return {this->red, this->green, this->blue, this->alpha};
    }
};


DECLARE_OUTPUT_STREAM_OPERATOR(Rgba<uint8_t>)
DECLARE_OUTPUT_STREAM_OPERATOR(Rgba<uint16_t>)
DECLARE_COMPARISON_OPERATORS(Rgba<uint8_t>)
DECLARE_COMPARISON_OPERATORS(Rgba<uint16_t>)


template<typename R, typename H>
Rgb<R> HsvToRgb(const Hsv<H> &hsv)
{
    ColorVector<R> rgbVector = HsvToRgb<R>(hsv.ToVector());

    return Rgb<R>::FromVector(rgbVector);
}


template<typename R, typename H>
Rgba<R> HsvToRgb(const Hsva<H> &hsva)
{
    AlphaVector<R> rgbVector = HsvToRgb<R>(hsva.ToVector());

    return Rgba<R>::FromVector(rgbVector);
}


template<typename H, typename R>
Hsv<H> RgbToHsv(const Rgb<R> &rgb)
{
    ColorVector<H> hsvVector = RgbToHsv<H>(rgb.ToVector());

    return Hsv<H>::FromVector(hsvVector);
}


template<typename H, typename R>
Hsva<H> RgbToHsv(const Rgba<R> &rgba)
{
    AlphaVector<H> hsvVector = RgbToHsv<H>(rgba.ToVector());

    return Hsva<H>::FromVector(hsvVector);
}


template<typename Color, typename = int>
struct HasAlpha_: std::false_type {};

// For any type Color that does not declare alpha, the comma operator falls
// back to 0.
template<typename Color>
struct HasAlpha_<Color, decltype((void)Color::alpha, 0)>: std::true_type {};


template<typename Color>
inline constexpr bool HasAlpha = HasAlpha_<Color>::value;


template<typename T>
using HsvGroup =
    pex::Group
    <
        HsvFields,
        HsvTemplate<T>::template MemberTemplate,
        Hsv<T>
    >;


template<typename T>
using HsvaGroup =
    pex::Group
    <
        HsvaFields,
        HsvaTemplate<T>::template MemberTemplate,
        Hsva<T>
    >;


template<typename T>
using RgbGroup =
    pex::Group
    <
        RgbFields,
        RgbTemplate<T>::template MemberTemplate,
        Rgb<T>
    >;

template<typename T>
using RgbaGroup =
    pex::Group
    <
        RgbaFields,
        RgbaTemplate<T>::template MemberTemplate,
        Rgba<T>
    >;


template
<
    typename T,
    typename Enable = std::enable_if_t<std::is_floating_point_v<T>>
>
using HsvPlanes =
    tau::Planar<3, T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;


template<typename T>
using RgbPlanes =
    tau::Planar<3, T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;


} // end namespace tau
