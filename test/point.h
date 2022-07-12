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
struct Point2d:
    public Point2dTemplate<T>::template Template<Identity>,
    public tau::Arithmetic<T, Point2dFields, Point2d>
{
    static constexpr auto fields = Point2dFields<Point2d>::fields;
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
struct Point3d:
    public Point3dTemplate<T>::template Template<Identity>,
    public tau::Arithmetic<T, Point3dFields, Point3d>
{
    static constexpr auto fields = Point3dFields<Point3d>::fields;
};
