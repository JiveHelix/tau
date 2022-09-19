#pragma once

#include <fields/fields.h>
#include <pex/group.h>

#include "tau/eigen.h"
#include "tau/arithmetic.h"
#include "tau/point.h"


namespace tau
{


template<typename T>
struct SizeFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::height, "height"),
        fields::Field(&T::width, "width"));
};


template<typename T>
struct SizeTemplate
{
    template<template<typename> typename V>
    struct Template
    {
        V<T> height;
        V<T> width;
    };
};


template<typename T>
struct Size
    : public SizeTemplate<T>::template Template<fields::Identity>,
      public tau::Arithmetic<T, SizeFields, Size>
{
    static constexpr auto fields = SizeFields<Size>::fields;
    
    using Type = T;

    using Base = typename SizeTemplate<T>::template Template<fields::Identity>;

    constexpr Size()
        :
        Base{0, 0}
    {

    }

    constexpr Size(Type height_, Type width_)
        :
        Base{height_, width_}
    {

    }

    Size(const Size &) = default;
    Size & operator=(const Size &) = default;
    Size(Size &&) = default;
    Size & operator=(Size &&) = default;

    template<typename U>
    Size(const Size<U> &size)
        :
        Size(size.template Convert<Type>())
    {

    }

    constexpr Size(const Point<Type> &point)
        :
        Base{point.y, point.x}
    {
        
    }

    template<typename U>
    constexpr Size(const Point<U> &point)
        :
        Size(point.template Convert<Type>())
    {
         
    }

    template<typename U>
    Size & operator=(const Size<U> &size)
    {
        *this = Size(size);
    }

    template<typename U>
    Size & operator=(const Point<U> &point)
    {
        *this = Size(std::move(point));
    }

    // Create a Size from an Eigen matrix.
    template<typename Derived>
    Size(const Eigen::DenseBase<Derived> &matrix)
        :
        Size(
            Size<typename Eigen::DenseBase<Derived>::Index>(
                matrix.rows(),
                matrix.cols()))
    {

    }

    template<typename U>
    Size(const Point<U> &first, const Point<U> &second)
    {
        Point<U> topLeft;
        Point<U> bottomRight;
        
        if (first < second)
        {
            topLeft = first;
            bottomRight = second;
        }
        else
        {
            topLeft = second;
            bottomRight = first;
        }

        Point<U> point = bottomRight - topLeft;

        *this = Size<U>(point.y, point.x);
    }

    Point<Type> ToPoint() const
    {
        return {this->height, this->width};
    }

    auto GetAngle()
    {
        if constexpr (std::is_integral_v<Type>)
        {
            auto size = this->template Convert<double>();
            return std::atan2(size.height, size.width);
        }
        else
        {
            return std::atan2(this->height, this->width);
        }
    }

    T GetArea() const
    {
        return this->height * this->width;
    }

    bool HasArea() const
    {
        return this->GetArea() > 0;
    }

    T & Horizontal() { return this->width; }
    T Horizontal() const { return this->width; }

    T & Vertical() { return this->height; }
    T Vertical() const { return this->height; }
};


template<typename T>
std::ostream & operator<<(std::ostream &output, const Size<T> &size)
{
    return output << fields::DescribeCompact(size);
}


template<typename T>
using SizeGroup =
    pex::Group<SizeFields, SizeTemplate<T>::template Template, Size<T>>;


} // end namespace tau
