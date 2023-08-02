#pragma once

#include <fields/fields.h>
#include <pex/group.h>

#include "tau/eigen.h"
#include "tau/arithmetic.h"
#include "tau/vector2d.h"
#include "tau/orthogonal.h"


namespace tau
{


template<typename T>
struct SizeFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::width, "width"),
        fields::Field(&T::height, "height"));
};


template<typename T>
struct SizeTemplate
{
    template<template<typename> typename V>
    struct Template
    {
        V<T> width;
        V<T> height;

        static constexpr auto fields = SizeFields<Template>::fields;
        static constexpr auto fieldsTypeName = "Size";
    };

};

template<typename T>
using SizeBase = typename SizeTemplate<T>::template Template<fields::Identity>;


template<typename T>
struct Size
    : public SizeBase<T>,
      public tau::Arithmetic<T, SizeFields, Size>
{
    using Type = T;

    constexpr Size()
        :
        SizeBase<T>{0, 0}
    {

    }

    constexpr Size(Type width_, Type height_)
        :
        SizeBase<T>{width_, height_}
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

    constexpr Size(const Point2d<Type> &point)
        :
        SizeBase<T>{point.x, point.y}
    {

    }

    template<typename U>
    constexpr Size(const Point2d<U> &point)
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
    Size & operator=(const Point2d<U> &point)
    {
        *this = Size(std::move(point));
    }

    // Create a Size from an Eigen matrix.
    template<typename Derived>
    Size(const Eigen::DenseBase<Derived> &matrix)
        :
        Size(
            Size<typename Eigen::DenseBase<Derived>::Index>(
                matrix.cols(),
                matrix.rows()))
    {

    }

    template<typename U>
    Size(const Point2d<U> &first, const Point2d<U> &second)
    {
        Point2d<U> topLeft;
        Point2d<U> bottomRight;

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

        Point2d<U> point = bottomRight - topLeft;
        *this = Size<U>(point.x, point.y);
    }

    Point2d<Type> ToPoint2d() const
    {
        return {this->width, this->height};
    }

    bool Contains(const Point2d<Type> &point)
    {
        return point.AndGreaterEqual(Point2d<Type>(0, 0))
        && point.AndLess(this->ToPoint2d());
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


extern template struct Size<int8_t>;
extern template struct Size<int16_t>;
extern template struct Size<int32_t>;
extern template struct Size<int64_t>;

extern template struct Size<uint8_t>;
extern template struct Size<uint16_t>;
extern template struct Size<uint32_t>;
extern template struct Size<uint64_t>;

extern template struct Size<float>;
extern template struct Size<double>;


} // end namespace tau


namespace pex
{


extern template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<int8_t>::template Template,
        tau::Size<int8_t>
    >;

extern template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<int16_t>::template Template,
        tau::Size<int16_t>
    >;

extern template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<int32_t>::template Template,
        tau::Size<int32_t>
    >;

extern template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<int64_t>::template Template,
        tau::Size<int64_t>
    >;

extern template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<uint8_t>::template Template,
        tau::Size<uint8_t>
    >;

extern template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<uint16_t>::template Template,
        tau::Size<uint16_t>
    >;

extern template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<uint32_t>::template Template,
        tau::Size<uint32_t>
    >;

extern template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<uint64_t>::template Template,
        tau::Size<uint64_t>
    >;

extern template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<float>::template Template,
        tau::Size<float>
    >;

extern template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<double>::template Template,
        tau::Size<double>
    >;


} // end namespace pex
