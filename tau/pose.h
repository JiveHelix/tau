#pragma once

#include <fields/fields.h>
#include <nlohmann/json.hpp>

#include "tau/eigen_shim.h"
#include "tau/stack.h"
#include "tau/vector3d.h"
#include "tau/intrinsics.h"
#include "tau/rotation.h"


namespace tau
{


template<typename T>
struct PoseFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::rotation, "rotation"),
        fields::Field(&T::x_m, "x_m"),
        fields::Field(&T::y_m, "y_m"),
        fields::Field(&T::z_m, "z_m"));
};


template<typename Float>
struct PoseTemplate
{
    template<template<typename> typename T>
    struct Template
    {
        T<RotationAnglesGroup<Float>> rotation;
        T<Float> x_m;
        T<Float> y_m;
        T<Float> z_m;

        static constexpr auto fields = PoseFields<Template>::fields;
        static constexpr auto fieldsTypeName = "Pose";
    };
};


/**
 **
 ** World Coordinate System
 ** X points straight ahead
 ** Y points towards left
 ** Z points up
 **
 ** The system is right-handed.
 **
 ** Right-handed rotations
 ** Yaw: positive rotation about the Z axis is counter-clockwise when
 ** looking down.
 **
 ** Pitch: positive rotation about the Y axis is counter-clockwise looking
 ** at the origin from the positive Y axis. Yaw is applied before pitch,
 ** meaning that the pitch axis changes with Yaw, and positive rotation is
 ** "down".
 **
 ** Roll: positive rotation about the X axis is counter-clockwise looking
 ** at the origin from the positive X axis.
 **
 **/
template<typename T>
struct Pose: public PoseTemplate<T>::template Template<pex::Identity>
{
    static constexpr auto version = jive::Version<uint8_t>(1, 0, 0);
    using Base = typename PoseTemplate<T>::template Template<pex::Identity>;

    Pose()
        :
        Base{
            RotationAngles<T>(),
            {},
            {},
            {}}
    {

    }

    Pose(const RotationAngles<T> &rotation_, T x_m_, T y_m_, T z_m_)
        :
        Base{
            rotation_,
            x_m_,
            y_m_,
            z_m_}
    {

    }

    static Pose Default()
    {
        return Pose();
    }

    Vector3<T> GetTranslation_m() const
    {
        return Vector3<T>{
            {this->x_m, this->y_m, this->z_m}};
    }

    Vector3<T> GetTranslation_pixels(const Intrinsics<T> &intrinsics) const
    {
        return intrinsics.MetersToPixels(this->GetTranslation_m());
    }

    RotationMatrix<T> GetRotation() const
    {
        return this->rotation.GetRotation()
            * SensorRelativeToWorld<T>();
    }

    auto GetArray_pixels(const Intrinsics<T> &intrinsics) const
    {
        return tau::HorizontalStack(
            this->GetRotation(),
            this->GetTranslation_pixels(intrinsics));
    }

    auto GetArray_m() const
    {
        return tau::HorizontalStack(
            this->GetRotation(),
            this->GetTranslation_m());
    }

    Eigen::Matrix<T, 4, 4> GetExtrinsics_pixels(
        const Intrinsics<T> &intrinsics) const
    {
        Eigen::Matrix<T, 3, 4> pose = this->GetArray_pixels(intrinsics);

        Eigen::Matrix<T, 4, 4> verticalStack = tau::VerticalStack(
            pose,
            Eigen::Matrix<T, 1, 4>{{0, 0, 0, 1}});

        return verticalStack.inverse();
    }

    Eigen::Matrix<T, 4, 4> GetExtrinsics_m() const
    {
        Eigen::Matrix<T, 3, 4> pose = this->GetArray_m();
        Eigen::Matrix<T, 4, 4> vStackPose = tau::VerticalStack(
            pose,
            Eigen::Matrix<T, 1, 4>{{0, 0, 0, 1}});

        return vStackPose.inverse();
    }

    Point3d<T> GetPosition_m() const
    {
        return {this->x_m, this->y_m, this->z_m};
    }

    Point3d<T> GetPosition_pixels(const Intrinsics<T> &intrinsics) const
    {
        auto position = this->GetPosition_m();
        return intrinsics.MetersToPixels(position);
    }

    static Pose Deserialize(const std::string &asString)
    {
        auto unstructured = nlohmann::json::parse(asString);
        auto fileVersion = jive::Version<uint8_t>(unstructured["version"]);
        auto minimumVersion = jive::Version<uint8_t>(1, 0, 0);

        if (fileVersion < minimumVersion)
        {
            throw std::runtime_error("Incompatible file version");
        }

        return fields::Structure<Pose>(unstructured);
    }

    std::string Serialize() const
    {
        auto unstructured = fields::Unstructure<nlohmann::json>(*this);
        unstructured["version"] = Pose::version.ToString();
        return unstructured.dump(4);
    }

    template<typename U>
    Pose<U> Convert() const
    {
        if constexpr (std::is_same_v<U, T>)
        {
            return *this;
        }

        Pose<U> result;
        result.rotation.axisOrder = this->rotation.axisOrder;
        result.rotation.yaw = static_cast<U>(this->rotation.yaw);
        result.rotation.pitch = static_cast<U>(this->rotation.pitch);
        result.rotation.roll = static_cast<U>(this->rotation.roll);

        result.x_m = static_cast<U>(this->x_m);
        result.y_m = static_cast<U>(this->y_m);
        result.z_m = static_cast<U>(this->z_m);

        return result;
    }
};


TEMPLATE_OUTPUT_STREAM(Pose)
TEMPLATE_EQUALITY_OPERATORS(Pose)


template<typename T>
using PoseGroup =
    pex::Group
    <
        PoseFields,
        PoseTemplate<T>::template Template,
        pex::PlainT<Pose<T>>
    >;

template<typename T>
using PoseModel = typename PoseGroup<T>::Model;

template<typename T>
using PoseControl = typename PoseGroup<T>::Control;


extern template struct Pose<float>;
extern template struct Pose<double>;


} // end namespace tau


extern template struct pex::Group
    <
        tau::PoseFields,
        tau::PoseTemplate<float>::template Template,
        pex::PlainT<tau::Pose<float>>
    >;


extern template struct pex::Group
    <
        tau::PoseFields,
        tau::PoseTemplate<double>::template Template,
        pex::PlainT<tau::Pose<double>>
    >;
