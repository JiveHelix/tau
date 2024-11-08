#pragma once

#include <jive/create_exception.h>
#include <fields/fields.h>
#include "tau/eigen_shim.h"

EIGEN_SHIM_PUSH_IGNORES
#include <Eigen/QR>
EIGEN_SHIM_POP_IGNORES

#include "tau/stack.h"
#include "tau/vector3d.h"
#include "tau/angles.h"
#include "tau/error.h"
#include "tau/vector3d.h"


namespace tau
{


template<typename T>
struct Line3dFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::point, "point"),
        fields::Field(&T::direction, "direction"));
};


template<typename T>
struct Line3dTemplate
{
    template<template<typename> typename V>
    struct Template
    {
        V<Point3d<T>> point;
        V<Vector3<T>> direction;

        static constexpr auto fields = Line3dFields<Template>::fields;
        static constexpr auto fieldsTypeName = "Line3d";
    };
};


template<typename T>
using Line3dBase = typename Line3dTemplate<T>::template Template<pex::Identity>;

template<typename T>
struct Line3d
    :
    public Line3dBase<T>,
    public BasicArithmetic<T, Line3d<T>>
{
    static constexpr auto zero = static_cast<T>(0);
    static constexpr auto one = static_cast<T>(1);

    Line3d()
        :
        Line3dBase<T>()
    {

    }

    Line3d(const Point3d<T> &point_, const Vector3<T> &direction_)
        :
        Line3dBase<T>{point_, direction_}
    {

    }

    static Line3d FromPoints(const Point3d<T> &begin, const Point3d<T> &end)
    {
        return Line3d(begin, (end - begin).ToEigen().normalized());
    }

    // Implement our own implementation of structure so we can recover the
    // direction, which is an Eigen::Vector.
    template<typename Json>
    static Line3d Structure(const Json &unstructured)
    {
        Line3d result{};

        if (1 == unstructured.count("point"))
        {
            result.point =
                fields::Structure<Point3d<T>>(unstructured["point"]);
        }

        if (1 == unstructured.count("direction"))
        {
            const auto &values = unstructured["direction"];

            if (values.size() != 3)
            {
                std::cerr << "Expected 3 values for direction" << std::endl;
                return result;
            }

            result.direction(0) = values[0];
            result.direction(1) = values[1];
            result.direction(2) = values[2];
        }

        return result;
    }

    /**
    let R have direction u = [a, b, c] and Q have direction v = [d, e, f]
    let r0 be a known point on R and q0 be a known point on Q

    R = r0 + tu
    Q = q0 + sv

    If the lines intersect, it will be when R == Q

    R = Q
    r0 + tu = q0 + sv

    tu - sv = q0 - r0

    at + bt + ct - (sd + se + sf) =

    at - sd = q0x - r0x
    bt - se = q0y - r0y
    ct - sf = q0z - r0z

    a  -d     t     q0_x - r0_x
    b  -e  @     =  q0_y - r0_y
    c  -f     s     q0_z - r0_z

    Three equations, two unknowns, we can use the pseudo-inverse */

    T DistanceToIntersection(const Line3d &otherLine) const
    {
        if (!this->HasIntersect(otherLine))
        {
            // Parallel lines have no intersection.
            throw NoIntersection("Parallel lines");
        }

        Eigen::Matrix<T, 3, 2> normals = tau::HorizontalStack(
            this->direction,
            (-1 * otherLine.direction.array()).eval());

        Vector3<T> points =
            otherLine.point.ToEigen().transpose()
            - this->point.ToEigen().transpose();

        Eigen::Matrix<T, 1, 2> parameters =
            normals.colPivHouseholderQr().solve(points).transpose();

        return parameters(0, 0);
    }

    bool HasIntersect(const Line3d &otherLine) const
    {
        return !otherLine.direction.isApprox(this->direction);
    }

    Point3d<T> Intersect(const Line3d &otherLine) const
    {
        return this->ScaleToPoint(this->DistanceToIntersection(otherLine));
    }

    Point3d<T> ScaleToPoint(T scale) const
    {
        return {
            this->point.ToEigen().array() + scale * this->direction.array()};
    }

    Line3d<T> GetPerpendicularThroughPoint(const Point3d<T> &point_) const
    {
        Vector3<T> toPoint = point_.ToEigen() - this->point.ToEigen();
        Vector3<T> normal = this->direction.cross(toPoint);
        Vector3<T> perpendicular = this->direction.cross(normal);

        return {point_, perpendicular};
    }

    T DistanceToPoint(const Point3d<T> &point_) const
    {
        Vector3<T> toPoint = point_.ToEigen() - this->point.ToEigen();
        Vector3<T> normal = this->direction.cross(toPoint);
        Vector3<T> perpendicular = this->direction.cross(normal);

        Line3d<T> lineFromPoint{point_, perpendicular};

        return std::abs(
            this->GetPerpendicularThroughPoint(point_)
                .DistanceToIntersection(*this));
    }

    T GetCoplanarValue(const Line3d &other) const
    {
        if (other.point == this->point)
        {
            // The line origins are the same, so they must be coplanar.
            return 0;
        }

        Vector3<T> connecting =
            (other.point.ToEigen() - this->point.ToEigen()).normalized();

        Vector3<T> plane = this->direction.cross(other.direction).normalized();

        // For the lines to be coplanar, the line connecting their end points
        // must lie in their plane.
        // So, the dot product of the connecting lines with the plane must be
        // zero (or close to zero).

        return std::abs(connecting.transpose().dot(plane));
    }

    T GetAngle_rad(const Line3d other)
    {
        return Vector3d<T>(this->direction).GetAngle_rad(
            Vector3d<T>(other.direction));
    }

    T GetAngle_deg(const Line3d other)
    {
        return ToDegrees(this->GetAngle_rad(other));
    }

    bool IsCoplanar(
            const Line3d &other, T threshold = static_cast<T>(1e-6)) const
    {
        return this->GetCoplanarValue(other) < threshold;
    }

    bool IsColinear(const Line3d &other) const
    {
        if (!IsLinear(this->direction, other.direction))
        {
            return false;
        }

        if (this->point == other.point)
        {
            // Directions and points match.
            return true;
        }

        // Directions are the same. Check their starting points.
        Vector3<T> connecting =
            Line3d::FromPoints(this->point, other.point).direction;

        return IsLinear(this->direction, connecting);
    }


    Vector3<T> Project(const Vector3<T> &planeNormal) const
    {
        return planeNormal.cross(this->direction.cross(planeNormal));
    }

    T GetAngleAboutZ() const
    {
        Vector3<T> projection = this->Project({zero, zero, one});
        return tau::ToDegrees(std::atan2(projection(1), projection(0)));
    }

    T GetAngleAboutY() const
    {
        Vector3<T> projection = this->Project({zero, one, zero});
        return tau::ToDegrees(std::atan2(-projection(2), projection(0)));
    }

    template<typename U, typename Style = Round>
    Line3d<U> Cast() const
    {
        return CastFields<Line3d<U>, U, Style>(*this);
    }
};


TEMPLATE_OUTPUT_STREAM(Line3d)
TEMPLATE_EQUALITY_OPERATORS(Line3d)


template<typename T>
using Line3dGroup =
    pex::Group
    <
        Line3dFields,
        Line3dTemplate<T>::template Template,
        pex::PlainT<Line3d<T>>
    >;


} // namespace tau
