#pragma once


#include <fields/fields.h>
#include <pex/group.h>
#include "tau/vector2d.h"
#include "tau/scale.h"
#include "tau/size.h"


namespace tau
{


template<typename T>
struct RegionFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::topLeft, "topLeft"),
        fields::Field(&T::size, "size"));
};


template<typename T>
struct RegionTemplate
{
    template<template<typename> typename V>
    struct Template
    {
        V<Point2dGroup<T>> topLeft;
        V<SizeGroup<T>> size;

        static constexpr auto fields = RegionFields<Template>::fields;
        static constexpr auto fieldsTypeName = "Region";
    };
};


template<typename T>
struct Region: public RegionTemplate<T>::template Template<pex::Identity>
{
    Point2d<T> GetBottomRight() const
    {
        return this->topLeft + this->size.ToPoint2d();
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
            return {{this->topLeft, {0, 0}}};
        }

        auto thisBottomRight = this->GetBottomRight();
        auto otherBottomRight = other.GetBottomRight();

        Point2d<T> resultTopLeft = this->topLeft;
        Point2d<T> resultBottomRight = thisBottomRight;

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

        return {{
            resultTopLeft,
            Size<T>(resultTopLeft, resultBottomRight)}};
    }

    template<typename U, typename Style = Round>
    Region<U> Cast() const
    {
        return CastFields<Region<U>, U, Style>(*this);
    }

    T GetArea() const
    {
        return this->size.GetArea();
    }

    bool HasArea() const
    {
        return this->size.HasArea();
    }
};


template<typename T>
using RegionGroup =
    pex::Group
    <
        RegionFields,
        RegionTemplate<T>::template Template,
        pex::PlainT<Region<T>>
    >;


template<typename T>
Region<T> & operator*=(
    Region<T> &left,
    const Scale<T> &scale)
{
    left.topLeft *= scale;
    left.size *= scale;

    return left;
}

template<typename T>
Region<T> & operator/=(
    Region<T> &left,
    const Scale<T> &scale)
{
    left.topLeft /= scale;
    left.size /= scale;

    return left;
}

template<typename T>
Region<T> operator*(
    const Region<T> &left,
    const Scale<T> &scale)
{
    auto result = left;
    result *= scale;
    return result;
}

template<typename T>
Region<T> operator/(
    const Region<T> &left,
    const Scale<T> &scale)
{
    auto result = left;
    result /= scale;
    return result;
}


template<typename T>
std::ostream & operator<<(
    std::ostream &outputStream,
    const Region<T> &region)
{
    return outputStream << fields::DescribeCompact(region);
}




using IntRegionGroup = RegionGroup<int>;
using IntRegion = typename IntRegionGroup::Plain;


} // end namespace tau
