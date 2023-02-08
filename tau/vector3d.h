#pragma once

#include <fmt/core.h>
#include <fields/fields.h>
#include <pex/group.h>

#include "tau/arithmetic.h"


namespace tau
{


template<typename T>
struct Vector3dFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::x, "x"),
        fields::Field(&T::y, "y"),
        fields::Field(&T::z, "z"));
};


template<typename T>
struct Vector3dTemplate
{
    template<template<typename> typename V>
    struct Template
    {
        V<T> x;
        V<T> y;
        V<T> z;

        static constexpr auto fields = Vector3dFields<Template>::fields;

    };
};


template<typename T>
using Vector3dBase =
    typename Vector3dTemplate<T>::template Template<pex::Identity>;


template<typename T, template<typename> typename Derived>
struct Base3d
    : public Vector3dBase<T>,
      public tau::Arithmetic<T, Vector3dFields, Derived>
{
    using Type = T;

    Base3d()
        :
        Vector3dBase<T>{0, 0, 0}
    {

    }

    Base3d(T x_, T y_, T z_)
        :
        Vector3dBase<T>{x_, y_, z_}
    {

    }

    Base3d(const Base3d &) = default;
    Base3d & operator=(const Base3d &) = default;
    Base3d(Base3d &&) = default;
    Base3d & operator=(Base3d &&) = default;

    template<typename U>
    Base3d(const Base3d<U, Derived> &point)
        :
        Base3d(point.template Convert<Type>())
    {

    }

    std::string GetAsString() const
    {
        return fmt::format("({}, {}, {})", this->x, this->y, this->z);
    }
};


template<typename T>
struct Vector3d;

template<typename T>
struct Point3d;


template<typename T>
struct Point3d: public Base3d<T, Point3d>
{
    using Base3d<T, Point3d>::Base3d;
    static constexpr auto fieldsTypeName = "Point3d";

    Point3d(const Vector3d<T> &vector3d)
        :
        Base3d<T, Point3d>(vector3d.x, vector3d.y)
    {

    }

    Vector3d<T> ToVector() const
    {
        return {this->x, this->y, this->z};
    }

    T Distance(const Point3d<T> &point) const
    {
        return (point - *this).Magnitude();
    }
};


template<typename T>
struct Vector3d: public Base3d<T, Vector3d>
{
    using Base3d<T, Vector3d>::Base3d;
    static constexpr auto fieldsTypeName = "Vector3d";
};


template<typename T>
std::ostream & operator<<(std::ostream &output, const Vector3d<T> &vector3d)
{
    return output << fields::DescribeCompact(vector3d);
}


template<typename T>
std::ostream & operator<<(std::ostream &output, const Point3d<T> &point3d)
{
    return output << fields::DescribeCompact(point3d);
}


template<typename T>
using Vector3dGroup =
    pex::Group
    <
        Vector3dFields,
        Vector3dTemplate<T>::template Template,
        Vector3d<T>
    >;


template<typename T>
using Point3dGroup =
    pex::Group
    <
        Vector3dFields,
        Vector3dTemplate<T>::template Template,
        Point3d<T>
    >;


template<typename T>
using Point3dCollection = std::vector<tau::Point3d<T>>;


template<typename T>
using Vector3dCollection = std::vector<tau::Vector3d<T>>;


} // end namespace tau
