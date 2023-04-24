#pragma once

#include <cmath>
#include "tau/eigen.h"
#include "tau/vector2d.h"
#include "tau/angular.h"


namespace tau
{


CREATE_EXCEPTION(NoIntersection, TauError);


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

    Line2d(const Point2d<T> &point_, const Vector2d<T> &vector_)
        :
        Line2dBase<T>{point_, vector_}
    {

    }

    Line2d(const Point2d<T> &first, const Point2d<T> &second)
        :
        Line2dBase<T>{first, (second - first).ToVector().Normalize()}
    {
        if (first == second)
        {
            throw std::runtime_error("Line is undefined");
        }
    }

    Line2d(const std::vector<Point2d<T>> &points)
    {
        static_assert(std::is_floating_point_v<T>);

        // Find the line that best fits the set of points.
        std::vector<Line2d<T>> lines;
        T sumX = 0;
        T sumY = 0;

        for (const auto &point_: points)
        {
            sumX += point_.x;
            sumY += point_.y;
        }

        auto pointCount = static_cast<T>(points.size());

        Point2d<T> averagePoint{{sumX / pointCount, sumY / pointCount}};
        this->point = averagePoint;

        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            const auto &start = points.at(i);

            for (size_t j = i + 1; j < points.size(); ++j)
            {
                lines.emplace_back(start, points.at(j));
            }
        }

        T angle = GetAverageAngleRadians<T>(lines);

        this->vector.x = std::cos(angle);
        this->vector.y = std::sin(angle);
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

    template<typename U>
    Line2d<U> Convert() const
    {
        return Line2d<U>(*this);
    }

    /**
    let R have direction u = [a, b] and Q have direction v = [c, d]
    let r0 be a known point on R and q0 be a known point on Q

    R = r0 + tu
    Q = q0 + sv

    If the lines intersect, it will be when R == Q

    R = Q
    r0 + tu = q0 + sv

    tu - sv = q0 - r0

    at + bt - (sc + sd) = q0 - r0

    at - sc = q0_x - r0_x
    bt - sd = q0_y - r0_y

    a  -c     t     q0_x - r0_x
           @     =
    b  -d     s     q0_y - r0_y

    Two equations, two unknowns */

    T DistanceToIntersection(const Line2d<T> &other) const
    {
        if (other.vector == this->vector)
        {
            // Parallel lines have no intersection.
            throw NoIntersection("Parallel lines");
        }

        Eigen::Matrix<T, 2, 2> directions{
            {this->vector.x, -other.vector.x},
            {this->vector.y, -other.vector.y}};

        Eigen::Vector<T, 2> points
        {
            other.point.x - this->point.x,
            other.point.y - this->point.y
        };

        Eigen::Vector<T, 2> parameters = directions.inverse() * points;

        return parameters(0);
    }

    Point2d<T> Intersect(const Line2d<T> &other) const
    {
        return this->point
            + this->DistanceToIntersection(other) * this->vector;
    }

    T DistanceToPoint(const Point2d<T> &point_) const
    {
        auto perpendicular = Line2d<T>(point_, this->vector.Rotate(90));
        return std::abs(perpendicular.DistanceToIntersection(*this));
    }

    T DistanceToLine(const Line2d<T> &other) const
    {
        return std::min(
            this->DistanceToPoint(other.point),
            other.DistanceToPoint(this->point));
    }

    Line2d<T> GetRotated(T angleDegrees) const
    {
        return Line2d<T>(this->point, this->vector.Rotate(angleDegrees));
    }

    Point2d<T> GetEndPoint(T scale) const
    {
        return this->point + this->vector * scale;
    }

    T GetAngleRadians() const
    {
        return std::atan2(this->vector.y, this->vector.x);
    }

    T GetAngleDegrees() const
    {
        return ToDegrees(this->GetAngleRadians());
    }

    bool IsColinear(
        const Line2d<T> &other,
        T toleranceDegrees,
        T toleranceOffset) const
    {
        T thisAngle = this->GetAngleDegrees();
        T otherAngle = other.GetAngleDegrees();

        if (std::isnan(thisAngle))
        {
            std::cout << "thisAngle is nan: "
                << fields::DescribeColorized(*this) << std::endl;

            return false;
        }

        if (std::isnan(otherAngle))
        {
            std::cout << "otherAngle is nan: "
                << fields::DescribeColorized(other) << std::endl;

            return false;
        }

        if (!CompareLineAngles(thisAngle, otherAngle, toleranceDegrees))
        {
            return false;
        }

        return this->DistanceToLine(other) <= toleranceOffset;
    }

    bool LessThan(const Line2d<T> &other, T toleranceDegrees) const
    {
        auto thisAngle = this->GetAngleDegrees();
        auto otherAngle = other.GetAngleDegrees();

        if (!CompareLineAngles(thisAngle, otherAngle, toleranceDegrees))
        {
            // The line angles are sufficiently different to be considered not
            // equal.
            return thisAngle < otherAngle;
        }

        // Line angles are equal.
        // Create a perpendicular line through the origin to sort this line
        // against other.

        double perpendicularAngle = thisAngle + 90;
        auto perpendicularAngle_rad = tau::ToRadians(perpendicularAngle);

        auto perpendicular = Line2d<double>(
            Point2d<double>(0, 0),
            Vector2d<double>(
                std::cos(perpendicularAngle_rad),
                std::sin(perpendicularAngle_rad)));

        // Sort the lines by their position along the intersecting
        // line.
        return perpendicular.DistanceToIntersection(*this)
            < perpendicular.DistanceToIntersection(other);
    }
};


template<typename T>
using Line2dCollection = std::vector<tau::Line2d<T>>;


} // end namespace tau
