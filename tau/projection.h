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
    Projection()
        :
        intrinsics_{},
        pose_{},
        cameraPosition_pixels_{},
        worldToCamera_{},
        cameraToWorld_{}
    {

    }

    Projection(
        const Intrinsics<T> &intrinsics,
        const Pose<T> &pose)
        :
        intrinsics_(intrinsics),
        pose_(pose),
        cameraPosition_pixels_(this->GetCameraPosition_pixels())
    {
        Extrinsic<T> extrinsics_m = this->pose_.GetExtrinsic_m();

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

    Extrinsic<T> GetExtrinsic_m() const
    {
        return this->pose_.GetExtrinsic_m();
    }

    Vector3<T> WorldToCamera(const Point3d<T> &world) const
    {
        Vector3<T> scaled =
            (this->worldToCamera_ * world.GetHomogeneous())(Eigen::seq(0, 2));

        return scaled.array() / scaled(2);
    }

    Vector3<T> VectorWorldToCamera(const Eigen::Vector<T, 4> &world) const
    {
        Vector3<T> scaled =
            (this->worldToCamera_ * world)(Eigen::seq(0, 2));

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

    template<typename>
    friend class Projection;

    template<typename Result, typename, typename, typename Source>
    friend Result CastFields(const Source &);

    template<typename U, typename Style = Round>
    Projection<U> Cast() const
    {
        return CastFields<Projection<U>, U, Style>(*this);
    }

    const Intrinsics<T> & GetIntrinsics() const
    {
        return this->intrinsics_;
    }

    const Pose<T> & GetPose() const
    {
        return this->pose_;
    }

private:
    Intrinsics<T> intrinsics_;
    Pose<T> pose_;
    Point3d<T> cameraPosition_pixels_;
    Eigen::Matrix<T, 4, 4> worldToCamera_;
    Eigen::Matrix<T, 3, 3> cameraToWorld_;

    static constexpr auto fields = std::make_tuple(
        fields::Field(&Projection::intrinsics_, "intrinsics"),
        fields::Field(&Projection::pose_, "pose"),
        fields::Field(&Projection::worldToCamera_, "worldToCamera"),
        fields::Field(&Projection::cameraToWorld_, "cameraToWorld"));


};


} // end namespace tau
