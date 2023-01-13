#pragma once

#include <cmath>
#include "tau/eigen.h"
#include "tau/vector2d.h"


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
T Factorial(T value)
{
    if (value <= 0)
    {
        return 1;
    }

    return value * Factorial(value - 1);
}


template<typename T, typename Lines>
static T GetAverageAngleRadians(const Lines &lines)
{
    auto lineCount = static_cast<Eigen::Index>(lines.size());
    Eigen::VectorX<T> angles(lineCount);

    for (Eigen::Index i = 0; i < lineCount; ++i)
    {
        angles(i) = lines[static_cast<size_t>(i)].GetAngleRadians();
    }

    // Lines with angles ±180 degrees are the same line.
    static constexpr T pi = tau::Angles<T>::pi;
    static constexpr T quarterTurn = pi / 2;
    angles.array() += pi;

    Eigen::VectorX<T> shiftedAngles =
        tau::Modulo(angles, tau::Angles<T>::pi);

    if (angles.size() != lineCount)
    {
        throw std::logic_error("Unexpected value");
    }

    T angle;

    // All points are expected to be close to a line.
    // If that line is close to 0/180, averaging 179 with 1 will not yield
    // the expected result.
    T first = shiftedAngles(0);

    if (first < (pi / 4) || first > (3 * pi / 4))
    {
        // The first angle is within ±45 degrees of 0/180.
        // Shift the values by 90 degrees.
        shiftedAngles.array() += quarterTurn;

        Eigen::VectorX<T> reshifted = tau::Modulo(shiftedAngles, pi);

        T reshiftedAngle = reshifted.sum() / static_cast<T>(lineCount);

        // Now shift the result back to the correct value.
        angle = std::fmod(reshiftedAngle + quarterTurn, pi);
    }
    else
    {
        angle = shiftedAngles.sum() / static_cast<T>(lineCount);
    }

    return angle;
}


template<typename T>
static T LineAngleDifference(T firstDegrees, T secondDegrees)
{
    static constexpr T quarterTurn = 90;

    if (firstDegrees < 45 || firstDegrees > 135)
    {
        // This angle is within ±45 degrees of 0/180.
        // Shift the values by 90 degrees.
        firstDegrees = std::fmod(firstDegrees + quarterTurn, 180);
        secondDegrees = std::fmod(secondDegrees + quarterTurn, 180);
    }

    return firstDegrees - secondDegrees;
}


template<typename T>
static bool CompareLineAngles(
    T firstDegrees,
    T secondDegrees,
    T toleranceDegrees)
{
    auto angleError =
        std::abs(LineAngleDifference(firstDegrees, secondDegrees));

    if (angleError > toleranceDegrees)
    {
        return false;
    }

    return true;
}


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

    }

    Line2d(const std::vector<Point2d<T>> &points)
    {
        static_assert(std::is_floating_point_v<T>);

        // Find the line that best fits the set of points.
        std::vector<Line2d<T>> lines;
        T sumX = 0;
        T sumY = 0;

        for (const auto &point: points)
        {
            sumX += point.x;
            sumY += point.y;
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

        if (!CompareLineAngles(thisAngle, otherAngle, toleranceDegrees))
        {
            return false;
        }

        if (this->DistanceToPoint(other.point) > toleranceOffset)
        {
            return false;
        }

        return true;
    }
};


template<typename T>
using Line2dCollection = std::vector<tau::Line2d<T>>;


} // end namespace tau
