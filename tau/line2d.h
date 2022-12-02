#pragma once

#include "tau/vector2d.h"


namespace tau
{


template<typename T>
struct Line2dFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::point, "point"),
        fields::Field(&T::vector, "vector"));
};


template<typename T>
struct Line2dTemplate
{
    template<template<typename> typename V>
    struct Template
    {
        V<Point2d<T>> point;
        V<Vector2d<T>> vector;

        static constexpr auto fields = Line2dFields<Template>::fields;

    };
};


template<typename T>
using Line2dBase =
    typename Line2dTemplate<T>::template Template<pex::Identity>;


template<typename T>
struct Line2d: public Line2dBase<T>
{
    using Type = T;

    Line2d()
        :
        Line2dBase<T>{{0, 0}, {1, 0}}
    {

    }

    Line2d(const Point2d<T> &point, const Vector2d<T> &vector)
        :
        Line2dBase<T>{point, vector}
    {

    }

    Line2d(const Point2d<T> &first, const Point2d<T> &second)
        :
        Line2dBase<T>{first, (second - first).ToVector().Normalize()}
    {

    }

    Line2d(const Line2d &) = default;
    Line2d & operator=(const Line2d &) = default;
    Line2d(Line2d &&) = default;
    Line2d & operator=(Line2d &&) = default;

    template<typename U>
    Line2d(const Line2d<U> &other)
        :
        Line2d(
            other.point.template Convert<Type>(),
            other.vector.template Convert<Type>())
    {

    }

#if 0
    T DistanceToPoint(const Point2d<T> &point)
    {
        auto perpendicular = this->vector.Rotate(90);
    }
#endif
};


} // end namespace tau
