#include "tau/arithmetic.h"

template<typename T>
using Identity = T;

template<typename T>
struct Point2dFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::x, "x"),
        fields::Field(&T::y, "y"));
};


template<typename T>
struct Point2dTemplate
{
    template<template<typename> typename V>
    struct Template
    {
        V<T> x;
        V<T> y;
    };
};


template<typename T>
using Point2dBase = typename Point2dTemplate<T>::template Template<Identity>;


template<typename T>
struct Point2d:
    public Point2dBase<T>,
    public tau::Arithmetic<T, Point2dFields, Point2d>
{
    static constexpr auto fields = Point2dFields<Point2d>::fields;

    Point2d() = default;

    Point2d(T x_, T y_): Point2dBase<T>{x_, y_} {}
};


template<typename T>
struct Point3dFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::x, "x"),
        fields::Field(&T::y, "y"),
        fields::Field(&T::z, "z"));
};


template<typename T>
struct Point3dTemplate
{
    template<template<typename> typename V>
    struct Template
    {
        V<T> x;
        V<T> y;
        V<T> z;
    };
};


template<typename T>
using Point3dBase = typename Point3dTemplate<T>::template Template<Identity>;


template<typename T>
struct Point3d:
    public Point3dBase<T>,
    public tau::Arithmetic<T, Point3dFields, Point3d>
{
    static constexpr auto fields = Point3dFields<Point3d>::fields;

    Point3d() = default;

    Point3d(T x_, T y_, T z_)
        :
        Point3dBase<T>{x_, y_, z_}
    {

    }
};
