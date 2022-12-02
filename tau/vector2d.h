#pragma once

#include <fields/fields.h>
#include <pex/group.h>

#include "tau/arithmetic.h"
#include "tau/angles.h"


namespace tau
{


template<typename T>
struct Vector2dFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::x, "x"),
        fields::Field(&T::y, "y"));
};


template<typename T>
struct Vector2dTemplate
{
    template<template<typename> typename V>
    struct Template
    {
        V<T> x;
        V<T> y;

        static constexpr auto fields = Vector2dFields<Template>::fields;

    };
};


template<typename T>
using Vector2dBase =
    typename Vector2dTemplate<T>::template Template<pex::Identity>;


template<typename T, template<typename> typename Derived>
struct Base2d
    : public Vector2dBase<T>,
      public tau::Arithmetic<T, Vector2dFields, Derived>
{
    using Type = T;

    Base2d()
        :
        Vector2dBase<T>{0, 0}
    {

    }

    Base2d(T x_, T y_)
        :
        Vector2dBase<T>{x_, y_}
    {

    }

    Base2d(const Base2d &) = default;
    Base2d & operator=(const Base2d &) = default;
    Base2d(Base2d &&) = default;
    Base2d & operator=(Base2d &&) = default;

    template<typename U>
    Base2d(const Base2d<U, Derived> &point)
        :
        Base2d(point.template Convert<Type>())
    {

    }

    T & Horizontal() { return this->x; }
    T Horizontal() const { return this->x; }

    T & Vertical() { return this->y; }
    T Vertical() const { return this->y; }

    auto GetAngle()
    {
        if constexpr (std::is_integral_v<T>)
        {
            // Convert to double for the computation.
            auto asDouble = this->template Convert<double>();
            return tau::ToDegrees(std::atan2(asDouble.y, asDouble.x));
        }
        else
        {
            // Already a floating-point type.
            // Do the calculation with the current precision.
            return tau::ToDegrees(std::atan2(this->y, this->x));
        }
    }
};


template<typename T>
struct Vector2d: public Base2d<T, Vector2d>
{
    using Base2d<T, Vector2d>::Base2d;
    static constexpr auto fieldsTypeName = "Vector2d";

    Vector2d<T> Rotate(T rotation_deg)
    {
        auto rotation_rad = tau::ToRadians(rotation_deg);
        auto sine = std::sin(rotation_rad);
        auto cosine = std::cos(rotation_rad);

        T x = cosine * this->x - sine * this->y;
        T y = sine * this->x + cosine * this->y;

        return {x, y};
    }
};


template<typename T>
struct Point2d: public Base2d<T, Point2d>
{
    using Base2d<T, Point2d>::Base2d;
    static constexpr auto fieldsTypeName = "Point2d";

    Point2d(const Vector2d<T> &vector2d)
        :
        Base2d<T, Point2d>(vector2d.x, vector2d.y)
    {

    }

    Vector2d<T> ToVector() const
    {
        return {this->x, this->y};
    }
};


template<typename T>
std::ostream & operator<<(std::ostream &output, const Vector2d<T> &vector2d)
{
    return output << fields::DescribeCompact(vector2d);
}


template<typename T>
std::ostream & operator<<(std::ostream &output, const Point2d<T> &point2d)
{
    return output << fields::DescribeCompact(point2d);
}


template<typename T>
using Vector2dGroup =
    pex::Group
    <
        Vector2dFields,
        Vector2dTemplate<T>::template Template,
        Vector2d<T>
    >;


template<typename T>
using Point2dGroup =
    pex::Group
    <
        Vector2dFields,
        Vector2dTemplate<T>::template Template,
        Point2d<T>
    >;


} // end namespace tau
