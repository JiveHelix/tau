#pragma once

#include <fields/fields.h>
#include <pex/group.h>

#include "tau/arithmetic.h"
#include "tau/scale.h"


namespace tau
{


template<typename T>
struct PointFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::y, "y"),
        fields::Field(&T::x, "x"));
};


template<typename T>
struct PointTemplate
{
    template<template<typename> typename V>
    struct Template
    {
        V<T> y;
        V<T> x;
    };
};


template<typename T>
struct Point
    : public PointTemplate<T>::template Template<fields::Identity>,
      public tau::Arithmetic<T, PointFields, Point>
{
    static constexpr auto fields = PointFields<Point>::fields;

    using Type = T;

    using Base = typename PointTemplate<T>::template Template<fields::Identity>;

    Point()
        :
        Base{0, 0}
    {

    }

    Point(T x_, T y_)
        :
        Base{x_, y_}
    {

    }

    Point(const Point &) = default;
    Point & operator=(const Point &) = default;
    Point(Point &&) = default;
    Point & operator=(Point &&) = default;

    template<typename U>
    Point(const Point<U> &point)
        :
        Point(point.template Convert<Type>())
    {

    }

    auto GetAngle()
    {
        if constexpr (std::is_integral_v<T>)
        {
            // Convert to double for the computation.
            auto point = this->template Convert<double>();
            return std::atan2(point.y, point.x);
        }
        else
        {
            // Already a floint-point type.
            // Do the calculation with the current precision.
            return std::atan2(this->y, this->x);
        }
    }

    T & Horizontal() { return this->x; }
    T Horizontal() const { return this->x; }

    T & Vertical() { return this->y; }
    T Vertical() const { return this->y; }
};


template<typename T>
std::ostream & operator<<(std::ostream &output, const Point<T> &point)
{
    return output << fields::DescribeCompact(point);
}


template<typename T>
using PointGroup =
    pex::Group<PointFields, PointTemplate<T>::template Template, Point<T>>;


} // end namespace tau
