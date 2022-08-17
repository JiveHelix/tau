#pragma once

#include <fields/fields.h>
#include <pex/group.h>

#include "tau/arithmetic.h"


namespace tau
{


template<typename T>
struct ScaleFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::vertical, "vertical"),
        fields::Field(&T::horizontal, "horizontal"));
};


template<typename T>
struct ScaleTemplate
{
    template<template<typename> typename V>
    struct Template
    {
        V<T> vertical;
        V<T> horizontal;
    };
};


template<typename T = double>
struct Scale
    :
    public ScaleTemplate<T>::template Template<fields::Identity>,
    public tau::Arithmetic<T, ScaleFields, Scale>
{
    using This = Scale<T>;
    using Type = T;

    static constexpr auto fields = ScaleFields<This>::fields;

    static Scale<T> Default()
    {
        return {{1.0, 1.0}};
    }
};


template<typename T, typename Enable = void>
struct HasHorizontal_: std::false_type {};

template<typename T>
struct HasHorizontal_
<
    T,
    std::void_t<decltype(std::declval<T>().Horizontal())>
>: std::true_type {};

template<typename T>
inline constexpr bool HasHorizontal = HasHorizontal_<T>::value;


template<typename T, typename Enable = void>
struct HasVertical_: std::false_type {};

template<typename T>
struct HasVertical_
<
    T,
    std::void_t<decltype(std::declval<T>().Vertical())>
>: std::true_type {};

template<typename T>
inline constexpr bool HasVertical = HasVertical_<T>::value;


template<typename T, typename Enable = void>
struct SupportsScaleOperators_: std::false_type {};

template<typename T>
struct SupportsScaleOperators_
<
    T,
    std::enable_if_t<HasHorizontal<T> && HasVertical<T>>
>: std::true_type {};

template<typename T>
inline constexpr bool SupportsScaleOperators =
    SupportsScaleOperators_<T>::value;


template
<
    typename T,
    template<typename> typename Fields,
    template<typename> typename Derived,
    typename U
>
Derived<T> & operator*=(
    Arithmetic<T, Fields, Derived> &left,
    const Scale<U> &scale)
{
    static_assert(
        SupportsScaleOperators<Derived<T>>,
        "Missing required functions Horizontal() and Vertical()");

    auto & derived = left.Upcast();

    auto result = derived.template Convert<U>();

    result.Horizontal() *= scale.horizontal;
    result.Vertical() *= scale.vertical;

    // Floor is only used when T is integral
    // No rounding is applied to floating-point conversions.
    return derived = result.template Convert<T, tau::Floor>();
}


template
<
    typename T,
    template<typename> typename Fields,
    template<typename> typename Derived,
    typename U
>
Derived<T> & operator/=(
    Arithmetic<T, Fields, Derived> &left,
    const Scale<U> &scale)
{
    static_assert(
        SupportsScaleOperators<Derived<T>>,
        "Missing required functions Horizontal() and Vertical()");

    auto & derived = left.Upcast();

    auto result = derived.template Convert<U>();

    result.Horizontal() /= scale.horizontal;
    result.Vertical() /= scale.vertical;

    // Floor is only used when T is integral
    // No rounding is applied to floating-point conversions.
    return derived = result.template Convert<T, tau::Floor>();
}


template
<
    typename T,
    template<typename> typename Fields,
    template<typename> typename Derived,
    typename U
>
Derived<T> operator*(
    const Arithmetic<T, Fields, Derived> &left,
    const Scale<U> &scale)
{
    static_assert(
        SupportsScaleOperators<Derived<T>>,
        "Missing required functions Horizontal() and Vertical()");

    // Make a copy
    auto derived = left.Upcast();

    // Call the in-place operator
    derived *= scale;

    return derived;
}


template
<
    typename T,
    template<typename> typename Fields,
    template<typename> typename Derived,
    typename U
>
Derived<T> operator/(
    const Arithmetic<T, Fields, Derived> &left,
    const Scale<U> &scale)
{
    static_assert(
        SupportsScaleOperators<Derived<T>>,
        "Missing required functions Horizontal() and Vertical()");

    // Make a copy
    auto derived = left.Upcast();

    // Call the in-place operator
    derived /= scale;

    return derived;
}


template<typename T>
std::ostream & operator<<(std::ostream &output, const Scale<T> &scale)
{
    return output << fields::DescribeCompact(scale);
}


template<typename T>
using ScaleGroup =
    pex::Group<ScaleFields, ScaleTemplate<T>::template Template, Scale<T>>;



} // end namespace tau
