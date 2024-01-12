#pragma once

#include <fields/fields.h>

#include "tau/eigen_shim.h"
#include "tau/stack.h"
#include "tau/vector2d.h"
#include "tau/line3d.h"
#include "tau/intrinsics.h"
#include "tau/pose.h"


namespace tau
{


// Project vector onto normal
template<typename T>
Eigen::Vector<T, 3> Project(
    const Eigen::Vector<T, 3> &normal,
    const Eigen::Vector<T, 3> &vector)
{
    return normal.transpose().dot(vector) * normal;
}


// Rejection of vector onto normal
// This is the projection of the vector onto the plane defined by the normal.
template<typename T>
Eigen::Vector<T, 3> Reject(
    const Eigen::Vector<T, 3> &normal,
    const Eigen::Vector<T, 3> &vector)
{
    return vector - Project(normal, vector);
}


template<typename T>
class Projection
{
public:
    Projection(
        const Intrinsics<T> &intrinsics,
        const Pose<T> &pose)
        :
        intrinsics_(intrinsics),
        pose_(pose),
        cameraPosition_pixels_(this->GetCameraPosition_pixels())
    {
        Eigen::Matrix<T, 4, 4> extrinsics_m = this->pose_.GetExtrinsics_m();

        Eigen::Matrix<T, 3, 4> topThreeRows =
            extrinsics_m(Eigen::seq(0, 2), Eigen::all);

        this->worldToCamera_ =
            tau::VerticalStack(
                (this->intrinsics_.GetArray_m() * topThreeRows).eval(),
                tau::RowVector<4, T>(T(0), T(0), T(0), T(1)));

        this->worldToCamera_.normalize();

        auto intrinsicsInverse = this->intrinsics_.GetInverse_pixels();

        this->cameraToWorld_ =
            (this->pose_.GetRotation() * intrinsicsInverse);

        this->cameraToWorld_.normalize();
    }

    Vector3<T> WorldToCamera(const Point3d<T> &world) const
    {
        Vector3<T> scaled =
            (this->worldToCamera_ * world.GetHomogeneous())(Eigen::seq(0, 2));

        return scaled.array() / scaled(2);
    }

    // The camera only knows the direction to the world point.
    // Use GetLine to get a line that passes through the world point.
    Vector3<T> CameraToWorld(const Vector3<T> &camera) const
    {
        return (this->cameraToWorld_ * camera);
    }

    Point3d<T> GetCameraPosition_pixels() const
    {
        return this->pose_.GetPosition_pixels(this->intrinsics_);
    }

    Line3d<T> GetLine(const tau::Point2d<T> &pixel) const
    {
        return Line3d<T>(
            this->cameraPosition_pixels_,
            this->CameraToWorld(pixel.GetHomogeneous()).normalized());
    }

    Line3d<T> GetLine_m(const tau::Point2d<T> &pixel) const
    {
        return Line3d<T>(
            this->pose_.GetPosition_m(),
            this->CameraToWorld(pixel.GetHomogeneous()).normalized());
    }

private:
    Intrinsics<T> intrinsics_;
    Pose<T> pose_;
    Point3d<T> cameraPosition_pixels_;
    Eigen::Matrix<T, 4, 4> worldToCamera_;
    Eigen::Matrix<T, 3, 3> cameraToWorld_;
};


} // end namespace tau
