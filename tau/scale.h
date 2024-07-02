#pragma once

#include <cassert>
#include <fields/fields.h>
#include <pex/identity.h>
#include <pex/selectors.h>
#include "tau/arithmetic.h"
#include "tau/orthogonal.h"


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

        static constexpr auto fields = ScaleFields<Template>::fields;
        static constexpr auto fieldsTypeName = "Scale";
    };
};


template<typename T>
using ScaleBase =
    typename ScaleTemplate<T>::template Template<pex::Identity>;


template<typename T>
struct Scale
    :
    public ScaleBase<T>,
    public Arithmetic<T, Scale>
{
    Scale(): ScaleBase<T>{static_cast<T>(1.0), static_cast<T>(1.0)}
    {

    }

    static Scale Default()
    {
        return {};
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
    template<typename> typename Derived,
    typename U
>
Derived<T> & operator*=(
    Arithmetic<T, Derived> &left,
    const Scale<U> &scale)
{
    static_assert(
        HasOrthogonals<Derived<T>>,
        "Missing required functions Horizontal() and Vertical()");

    auto & derived = left.Upcast();

    auto result = derived.template Cast<U>();

    result.Horizontal() *= scale.horizontal;
    result.Vertical() *= scale.vertical;

    // Floor is only used when T is integral
    // No rounding is applied to floating-point conversions.
    return derived = result.template Cast<T, Floor>();
}


template
<
    typename T,
    template<typename> typename Derived,
    typename U
>
Derived<T> & operator/=(
    Arithmetic<T, Derived> &left,
    const Scale<U> &scale)
{
    static_assert(
        HasOrthogonals<Derived<T>>,
        "Missing required functions Horizontal() and Vertical()");

    assert(scale.vertical != 0);
    assert(scale.horizontal != 0);

    auto & derived = left.Upcast();

    auto result = derived.template Cast<U>();

    result.Horizontal() /= scale.horizontal;
    result.Vertical() /= scale.vertical;

    // Floor is only used when T is integral
    // No rounding is applied to floating-point conversions.
    return derived = result.template Cast<T, Floor>();
}


template
<
    typename T,
    template<typename> typename Derived,
    typename U
>
Derived<T> operator*(
    const Arithmetic<T, Derived> &left,
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
    template<typename> typename Derived,
    typename U
>
Derived<T> operator/(
    const Arithmetic<T, Derived> &left,
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


extern template struct Scale<float>;
extern template struct Scale<double>;


} // end namespace tau
