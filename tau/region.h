#pragma once


#include <fields/fields.h>
#include <pex/group.h>
#include "tau/point.h"
#include "tau/size.h"
#include "tau/scale.h"


namespace tau
{


template<typename T>
struct RegionFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::y, "topLeft"),
        fields::Field(&T::x, "size"));
};


template<typename T>
struct RegionTemplate
{
    template<template<typename> typename V>
    struct Template
    {
        V<pex::MakeGroup<PointGroup<T>>> topLeft;
        V<pex::MakeGroup<SizeGroup<T>>> size;
    };
};


template<typename T>
struct Region
    :
    public RegionTemplate<T>::template Template<pex::Identity>
{
    Point<T> GetBottomRight() const
    {
        return this->topLeft + this->size.ToPoint();
    }

    static constexpr auto fields = std::make_tuple(
        fields::Field(&Region::topLeft, "topLeft"),
        fields::Field(&Region::size, "size"));

    bool Intersects(const Region &other) const
    {
        /*
        Returns true if any part of the two rectangles overlap.

        A helpful visualization:
        https://silentmatt.com/rectangle-intersection/
        */

        auto thisBottomRight = this->GetBottomRight();
        auto otherBottomRight = other.GetBottomRight();

        // If any of the following are true, there is no intersection
        bool otherIsToTheLeft = otherBottomRight.x <= this->topLeft.x;
        bool otherIsBelow = other.topLeft.y >= thisBottomRight.y;
        bool otherIsToTheRight = other.topLeft.x >= thisBottomRight.x;
        bool otherIsAbove = otherBottomRight.y <= this->topLeft.y;

        bool noIntersection =
            otherIsToTheLeft
            || otherIsBelow
            || otherIsToTheRight
            || otherIsAbove;

        return !noIntersection;
    }

    Region Intersect(const Region &other) const
    {
        if (!this->Intersects(other))
        {
            return {this->topLeft, {0, 0}};
        }

        auto thisBottomRight = this->GetBottomRight();
        auto otherBottomRight = other.GetBottomRight();

        Point<T> resultTopLeft = this->topLeft;
        Point<T> resultBottomRight = thisBottomRight;

        if (other.topLeft.x > this->topLeft.x) 
        {
            resultTopLeft.x = other.topLeft.x;
        }

        if (other.topLeft.y > this->topLeft.y)
        {
            resultTopLeft.y = other.topLeft.y;
        }

        if (otherBottomRight.x < thisBottomRight.x)
        {
            resultBottomRight.x = otherBottomRight.x;
        }

        if (otherBottomRight.y < thisBottomRight.y)
        {
            resultBottomRight.y = otherBottomRight.y;
        }
        
        return {
            resultTopLeft,
            Size<T>(resultTopLeft, resultBottomRight)};
    }

    template<typename U, typename Style = Round>
    Region<U> Convert() const
    {
        Region<U> result;
        result.topLeft = this->topLeft.template Convert<U, Style>();
        result.size = this->size.template Convert<U, Style>();

        return result;
    }
};


template<typename T, typename U>
Region<T> & operator*=(Region<T> &left, const Scale<U> &scale)
{
    auto scaledRegion = left.template Convert<U>();

    scaledRegion.topLeft *= scale;
    scaledRegion.size *= scale;
    left = scaledRegion.template Convert<T>();

    return left;
}

template<typename T, typename U>
Region<T> & operator/=(Region<T> &left, const Scale<U> &scale)
{
    auto scaledRegion = left.template Convert<U>();

    scaledRegion.topLeft /= scale;
    scaledRegion.size /= scale;
    left = scaledRegion.template Convert<T>();

    return left;
}

template<typename T, typename U>
Region<T> operator*(const Region<T> &left, const Scale<U> &scale)
{
    auto result = left;
    result *= scale;
    return result;
}

template<typename T, typename U>
Region<T> operator/(const Region<T> &left, const Scale<U> &scale)
{
    auto result = left;
    result /= scale;
    return result;
}


template<typename T>
std::ostream & operator<<(std::ostream &outputStream, const Region<T> &region)
{
    return outputStream << fields::DescribeCompact(region);
}


template<typename T>
using RegionGroup =
    pex::Group<RegionFields, RegionTemplate<T>::template Template, Region<T>>;


} // end namespace tau
