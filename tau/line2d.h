#pragma once

#include <cmath>
#include "tau/eigen.h"
#include "tau/vector2d.h"
#include "tau/angular.h"
#include "tau/region.h"


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

    // Construct a line from the Hesse Normal.
    Line2d(T distance, T theta_deg)
    {
        // First find a point on the line.
        using Point = Point2d<T>;
        using Vector = Vector2d<T>;
        auto origin = Point(0, 0);
        auto vector_ = Vector(1, 0).Rotate(theta_deg);
        this->point = origin + (distance * vector_);
        this->vector = vector_.Rotate(90);

        if (theta_deg > 90 || theta_deg < -90)
        {
            // The resulting line angle will be outside of the range [0, 180].
            this->vector = this->vector.Rotate(180);
        }
    }

    Line2d(const Line2d &) = default;
    Line2d & operator=(const Line2d &) = default;
    Line2d(Line2d &&) = default;
    Line2d & operator=(Line2d &&) = default;

    template<typename U>
    Line2d(const Line2d<U> &other)
        :
        Line2d(other.template Cast<Type>())
    {

    }

    template<typename U, typename Style = Round>
    Line2d<U> Cast() const
    {
        return CastFields<Line2d<U>, U, Style>(*this);
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

    ta + tb - (sc + sd) = q0 - r0

    ta - sc = q0_x - r0_x
    tb - sd = q0_y - r0_y

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

        auto inverse = directions.inverse();

        // We only need the scalar 't', so we can skip half of the operations.
        // Eigen::Vector<T, 2> scalars = directions.inverse() * points;

        return (inverse.topRows(1) * points)(0);
    }

    bool HasIntersection(const Line2d<T> &other) const
    {
        return (other.vector != this->vector);
    }

    Point2d<T> Intersect(const Line2d<T> &other) const
    {
        return this->point
            + this->DistanceToIntersection(other) * this->vector;
    }

    template<typename U>
    std::optional<std::pair<Point2d<T>, Point2d<T>>>
    Intersect(const Region<U> &region_) const
    {
        using Point = Point2d<T>;
        using Vector = Vector2d<T>;

        auto region = region_.template Cast<T>();
        Line2d leftEdge(region.topLeft, Vector(0, 1));
        Line2d topEdge(region.topLeft, Vector(1, 0));
        Line2d rightEdge(region.GetBottomRight(), Vector(0, -1));
        Line2d bottomEdge(region.GetBottomRight(), Vector(-1, 0));

        if (!this->HasIntersection(leftEdge))
        {
            // this must intersect with topEdge and bottomEdge.
            if (this->point.x < leftEdge.point.x
                    || (this->point.x > rightEdge.point.x))
            {
                // There is no intersection with this region.
                return {};
            }

            return std::make_pair(
                this->Intersect(topEdge),
                this->Intersect(bottomEdge));
        }

        if (!this->HasIntersection(topEdge))
        {
            if (this->point.y < topEdge.point.y
                    || (this->point.y > bottomEdge.point.y))
            {
                // There is no intersection with this region.
                return {};
            }

            return std::make_pair(
                this->Intersect(leftEdge),
                this->Intersect(rightEdge));
        }

        std::vector<Point> points;

        auto left = this->Intersect(leftEdge);

        if (left.y >= topEdge.point.y && left.y <= bottomEdge.point.y)
        {
            points.push_back(left);
        }

        auto right = this->Intersect(rightEdge);

        if (right.y >= topEdge.point.y && right.y <= bottomEdge.point.y)
        {
            points.push_back(right);
        }

        if (points.size() == 2)
        {
            return std::make_pair(points.front(), points.back());
        }

        auto top = this->Intersect(topEdge);

        if (top.x >= leftEdge.point.x && top.x <= rightEdge.point.x)
        {
            points.push_back(top);
        }

        if (points.size() == 2)
        {
            return std::make_pair(points.front(), points.back());
        }

        auto bottom = this->Intersect(bottomEdge);

        if (bottom.x >= leftEdge.point.x && bottom.x <= rightEdge.point.x)
        {
            if (points.size() == 1)
            {
                return std::make_pair(points.front(), bottom);
            }
        }

        // There is no intersection with this region.
        return {};
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
        auto perpendicularAngle_rad = ToRadians(perpendicularAngle);

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


TEMPLATE_OUTPUT_STREAM(Line2d)


template<typename T>
using Line2dCollection = std::vector<Line2d<T>>;


extern template struct Line2d<float>;
extern template struct Line2d<double>;


} // end namespace tau
