#pragma once

#include <tau/eigen.h>
#include <fmt/core.h>
#include <fields/fields.h>
#include <pex/group.h>

#include "tau/arithmetic.h"
#include "tau/angles.h"


namespace tau
{


template<typename T>
using Vector3 = Eigen::Vector<T, 3>;

template<typename T>
using Vector4 = Eigen::Vector<T, 4>;


template<typename T>
bool IsScaled(const Vector3<T> &first, const Vector3<T> &second)
{
    return first.normalized().isApprox(second.normalized());
}


template<typename T>
bool IsSameDirection(
    const Vector3<T> &first,
    const Vector3<T> &second,
    T threshold = static_cast<T>(0.9))
{
    return first.normalized().transpose().dot(second.normalized()) > threshold;
}


template<typename T>
bool IsLinear(const Vector3<T> first, const Vector3<T> second)
{
    if (first.isApprox(second))
    {
        return true;
    }

    Vector3<T> reversed = second.array() * -1;

    return (first.isApprox(reversed));
}


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
      public tau::Arithmetic<T, Derived>
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
        Base3d(point.template Cast<Type>())
    {

    }

    std::string GetAsString() const
    {
        return fmt::format("({}, {}, {})", this->x, this->y, this->z);
    }

    T operator()(Eigen::Index index) const
    {
        if (index == 0)
        {
            return this->x;
        }
        else if (index == 1)
        {
            return this->y;
        }
        else if (index == 2)
        {
            return this->z;
        }
        else
        {
            throw std::out_of_range("Index not valid for Point3d");
        }
    }

    T & operator()(Eigen::Index index)
    {
        if (index == 0)
        {
            return this->x;
        }
        else if (index == 1)
        {
            return this->y;
        }
        else if (index == 2)
        {
            return this->z;
        }
        else
        {
            throw std::out_of_range("Index not valid for Point3d");
        }
    }

    Eigen::Vector<T, 3> ToEigen() const
    {
        return Eigen::Vector<T, 3>(this->x, this->y, this->z);
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

    // Compare equal to 6 decimal places.
    static constexpr ssize_t precision = 6;

    Point3d(const Vector3<T> &vector_)
        :
        Point3d(vector_(0), vector_(1), vector_(2))
    {

    }

    Point3d(const Vector3d<T> &vector3d)
        :
        Base3d<T, Point3d>(vector3d.x, vector3d.y, vector3d.z)
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

    Eigen::Vector<T, 4> GetHomogeneous() const
    {
        Eigen::Vector<T, 4> result;
        result.template head<3>() = this->ToEigen();
        result(3) = static_cast<T>(1);
        return result;
    }
};


template<typename T>
struct Vector3d: public Base3d<T, Vector3d>
{
    using Base3d<T, Vector3d>::Base3d;
    static constexpr auto fieldsTypeName = "Vector3d";

    Vector3d(const Vector3<T> &vector_)
        :
        Base3d<T, Vector3d>::Base3d(vector_(0), vector_(1), vector_(2))
    {

    }

    T GetAngle_rad(const Vector3d<T> &other)
    {
        return ::tau::GetAngle_rad(this->ToEigen(), other.ToEigen());
    }

    T GetAngle_deg(const Vector3d<T> &other)
    {
        return ToDegrees(this->GetAngle_rad(other));
    }
};


TEMPLATE_OUTPUT_STREAM(Vector3d)
TEMPLATE_OUTPUT_STREAM(Point3d)


template<typename T>
using Vector3dGroup =
    pex::Group
    <
        Vector3dFields,
        Vector3dTemplate<T>::template Template,
        pex::PlainT<Vector3d<T>>
    >;


template<typename T>
using Point3dGroup =
    pex::Group
    <
        Vector3dFields,
        Vector3dTemplate<T>::template Template,
        pex::PlainT<Point3d<T>>
    >;


template<typename T>
using Point3dCollection = std::vector<tau::Point3d<T>>;


template<typename T>
using Vector3dCollection = std::vector<tau::Vector3d<T>>;


} // end namespace tau
