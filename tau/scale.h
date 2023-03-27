#pragma once

#include <cassert>
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


template<typename T>
using ScaleBase =
    typename ScaleTemplate<T>::template Template<fields::Identity>;


template<typename T = double>
struct Scale
    :
    public ScaleBase<T>,
    public tau::Arithmetic<T, ScaleFields, Scale>
{
    using This = Scale<T>;
    using Type = T;

    static constexpr auto fields = ScaleFields<This>::fields;

    Scale(): ScaleBase<T>{1.0, 1.0}
    {

    }

    Scale(T vertical_, T horizontal_)
        :
        ScaleBase<T>{vertical_, horizontal_}
    {

    }
};


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
        HasOrthogonals<Derived<T>>,
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
        HasOrthogonals<Derived<T>>,
        "Missing required functions Horizontal() and Vertical()");

    assert(scale.vertical != 0);
    assert(scale.horizontal != 0);

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
        HasOrthogonals<Derived<T>>,
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
        HasOrthogonals<Derived<T>>,
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


template<typename T, typename Minimum, typename Maximum>
using ScaleGroup =
    pex::Group
    <
        ScaleFields,
        ScaleTemplate<pex::MakeRange<T, Minimum, Maximum>>::template Template,
        Scale<T>
    >;



} // end namespace tau
