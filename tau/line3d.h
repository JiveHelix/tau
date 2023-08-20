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


template <typename T>
struct Line3dFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::point, "point"),
        fields::Field(&T::direction, "direction"));
};


template<typename T>
struct Line3d
{
    Point3d<T> point;
    Vector3<T> direction;

    static constexpr auto fields = Line3dFields<Line3d>::fields;
    static constexpr auto fieldsTypeName = "Line3d";
    static constexpr auto zero = static_cast<T>(0);
    static constexpr auto one = static_cast<T>(1);

    Line3d()
        :
        point{},
        direction{}
    {

    }

    Line3d(const Point3d<T> &point_, const Vector3<T> &direction_)
        :
        point(point_),
        direction(direction_)
    {

    }

    static Line3d FromPoints(const Point3d<T> &begin, const Point3d<T> &end)
    {
        return Line3d(begin, (end - begin).ToEigen().normalized());
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

    Point3d<T> Intersect(const Line3d &otherLine) const
    {
        Eigen::Matrix<T, 3, 2> normals = tau::HorizontalStack(
            this->direction,
            (-1 * otherLine.direction.array()).eval());

        Vector3<T> points =
            otherLine.point.ToEigen().transpose()
            - this->point.ToEigen().transpose();

        Eigen::Matrix<T, 1, 2> parameters =
            normals.colPivHouseholderQr().solve(points).transpose();

        return this->ScaleToPoint(parameters(0, 0));
    }

    Point3d<T> ScaleToPoint(T scale) const
    {
        return {
            this->point.ToEigen().array() + scale * this->direction.array()};
    }

    T GetCoplanarValue(const Line3d &other)
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

    bool IsCoplanar(const Line3d &other, T threshold = static_cast<T>(1e-6))
    {
        return this->GetCoplanarValue(other) < threshold;
    }

    bool IsColinear(const Line3d &other)
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
};


TEMPLATE_OUTPUT_STREAM(Line3d)


} // namespace tau
