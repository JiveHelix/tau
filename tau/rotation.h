#pragma once

#include <iterator>
#include <fields/fields.h>
#include <pex/group.h>
#include <pex/interface.h>
#include <Eigen/Dense>
#include <cmath>
#include <tau/angles.h>
#include "tau/error.h"


namespace tau
{


CREATE_EXCEPTION(RotationError, TauError);


template<typename T>
using RotationMatrix = Eigen::Matrix<T, 3, 3>;


template<size_t axis, typename T>
RotationMatrix<T> MakeAxial(T rotation_deg)
{
    static_assert(axis < 3, "Designed for 3-D only");

    auto rotation_rad = tau::ToRadians(rotation_deg);
    auto sine = std::sin(rotation_rad);
    auto cosine = std::cos(rotation_rad);

    if constexpr (axis == 0)
    {
        return RotationMatrix<T>{
            {1.0, 0.0, 0.0},
            {0.0, cosine, -sine},
            {0.0, sine, cosine}};
    }
    else if constexpr (axis == 1)
    {
        return RotationMatrix<T>{
            {cosine, 0.0, sine},
            {0.0, 1.0, 0.0},
            {-sine, 0.0, cosine}};
    }
    else
    {
        // axis == 2
        return RotationMatrix<T>{
            {cosine, -sine, 0.0},
            {sine, cosine, 0.0},
            {0.0, 0.0, 1.0}};
    }
}


template<typename T>
RotationMatrix<T> MakeAxial(size_t axis, T rotation_deg)
{
    switch (axis)
    {
        case 0:
            return MakeAxial<0, T>(rotation_deg);
        case 1:
            return MakeAxial<1, T>(rotation_deg);
        case 2:
            return MakeAxial<2, T>(rotation_deg);
        default:
            throw RotationError("Unsupported axis");
    }
}


template<size_t first, size_t second, size_t third, typename T>
RotationMatrix<T> MakeIntrinsic(
    T first_deg,
    T second_deg,
    T third_deg)
{
    return MakeAxial<first>(first_deg)
        * MakeAxial<second>(second_deg)
        * MakeAxial<third>(third_deg);
}


struct AxisOrder;


struct AxisOrderConverter
{
    static std::string ToString(const AxisOrder &axisOrder);

    static AxisOrder ToValue(const std::string &asString);
};


struct AxisOrder
{
    static const std::array<std::string, 3> axisNames;

    size_t first;
    size_t second;
    size_t third;

    std::ostream & ToStream(std::ostream &outputStream) const;

    template<typename Json>
    Json Unstructure() const
    {
        return AxisOrderConverter::ToString(*this);
    }

    template<typename Json>
    static AxisOrder Structure(const Json &json)
    {
        return AxisOrderConverter::ToValue(json.template get<std::string>());
    }

    bool operator==(const AxisOrder &other) const
    {
        return (
            this->first == other.first
            && this->second == other.second
            && this->third == other.third);
    }
};


std::ostream & operator<<(
    std::ostream &outputStream,
    const AxisOrder &axisOrder);


template<typename T>
RotationMatrix<T> MakeIntrinsic(
    const AxisOrder &axisOrder,
    T first_deg,
    T second_deg,
    T third_deg)
{
    return MakeAxial(axisOrder.first, first_deg)
        * MakeAxial(axisOrder.second, second_deg)
        * MakeAxial(axisOrder.third, third_deg);
}


template<typename T>
struct RotationAnglesFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::yaw, "yaw"),
        fields::Field(&T::pitch, "pitch"),
        fields::Field(&T::roll, "roll"),
        fields::Field(&T::axisOrder, "axisOrder"));
};


template<typename U>
struct RotationAnglesTemplate
{
    using AngleRange = pex::MakeRange<U, pex::Limit<-180>, pex::Limit<180>>;

    template<template<typename> typename T>
    struct Template
    {
        T<AngleRange> yaw;
        T<AngleRange> pitch;
        T<AngleRange> roll;
        T<pex::MakeSelect<AxisOrder>> axisOrder;

        static constexpr auto fields = RotationAnglesFields<Template>::fields;
        static constexpr auto fieldsTypeName = "RotationAngles";
    };
};


template<typename T>
struct RotationAngles:
    public RotationAnglesTemplate<T>::template Template<pex::Identity>
{
    using Base =
        typename RotationAnglesTemplate<T>::template Template<pex::Identity>;

    // 0: roll (about x)
    // 1: pitch (about y)
    // 2: yaw (about z)
    // default is yaw-pitch-roll
    static constexpr auto defaultAxisOrder = AxisOrder{2, 1, 0};

    RotationAngles()
        :
        Base{0, 0, 0, defaultAxisOrder}
    {

    }

    RotationAngles(
        T first,
        T second,
        T third,
        const AxisOrder &axisOrder_ = defaultAxisOrder)
    {
        this->axisOrder = axisOrder_;
        (*this)(axisOrder_.first) = first;
        (*this)(axisOrder_.second) = second;
        (*this)(axisOrder_.third) = third;
    }

    static RotationAngles Default()
    {
        return RotationAngles();
    }

    RotationAngles(
        const RotationMatrix<T> &rotation,
        const AxisOrder &axisOrder_ = defaultAxisOrder)
    {
        using Vector = Eigen::Vector<T, 3>;

        using Eigen::Index;

        Vector angles = tau::ToDegrees(
            rotation.eulerAngles(
                static_cast<Index>(axisOrder_.first),
                static_cast<Index>(axisOrder_.second),
                static_cast<Index>(axisOrder_.third)));

        (*this)(axisOrder_.first) = angles(0);
        (*this)(axisOrder_.second) = angles(1);
        (*this)(axisOrder_.third) = angles(2);
        this->axisOrder = axisOrder_;
    }

    T & operator()(size_t axis)
    {
        switch (axis)
        {
            case 0:
                return this->roll;

            case 1:
                return this->pitch;

            case 2:
                return this->yaw;

            default:
                throw RotationError("out of bounds index");
        }
    }

    T operator()(size_t axis) const
    {
        return const_cast<RotationAngles<T> *>(this)->operator()(axis);
    }

    RotationMatrix<T> GetRotation() const
    {
        return MakeIntrinsic(
            this->axisOrder,
            (*this)(this->axisOrder.first),
            (*this)(this->axisOrder.second),
            (*this)(this->axisOrder.third));
    }
};


TEMPLATE_OUTPUT_STREAM(RotationAngles)


template<typename T>
using RotationAnglesGroup =
    pex::Group
    <
        RotationAnglesFields,
        RotationAnglesTemplate<T>::template Template,
        RotationAngles<T>
    >;

template<typename T>
struct RotationAnglesModel: RotationAnglesGroup<T>::Model
{
    using Base = typename RotationAnglesGroup<T>::Model;

    RotationAnglesModel()
        :
        Base(RotationAngles<T>::Default())
    {
        this->axisOrder.SetChoices(
            {
                {2, 1, 0},
                {2, 0, 1},
                {1, 2, 0},
                {1, 0, 2},
                {0, 2, 1},
                {0, 1, 2}});
    }
};


template<typename T>
using AnglesControl =
    typename RotationAnglesGroup<T>::Control;

template<typename T>
using AnglesGroupMaker =
    pex::MakeGroup<RotationAnglesGroup<T>, RotationAnglesModel<T>>;


template<typename T>
RotationMatrix<T> AboutX(T rotation_deg)
{
    return MakeAxial<0>(rotation_deg);
}


template<typename T>
RotationMatrix<T> AboutY(T rotation_deg)
{
    return MakeAxial<1>(rotation_deg);
}


template<typename T>
RotationMatrix<T> AboutZ(T rotation_deg)
{
    return MakeAxial<2>(rotation_deg);
}


/***** Intrinsic Rotation Matrices *****/

/* Note: Yaw-Pitch-Roll intrinsic rotation is computed as Roll-Pitch-Yaw */

/* Yaw-Pitch-Roll using camera coordinate system
 *      Y positive down (row of image sensor)
 *      X positive left to right (column of image sensor (pitch axis)
 *      Z perpendicular to image plane (roll axis)
 */
template<typename T>
RotationMatrix<T> MakeYxz(T y_deg, T x_deg, T z_deg)
{
    return MakeIntrinsic<1, 0, 2>(y_deg, x_deg, z_deg);
}


/* Yaw-Pitch-Roll using world coordinate system
 *      X positive Forward (roll axis)
 *      Y positive to left (pitch axis)
 *      Z up (yaw axis)
 */
template<typename T>
RotationMatrix<T> MakeYawPitchRoll(T yaw, T pitch, T roll)
{
    return MakeIntrinsic<2, 1, 0>(yaw, pitch, roll);
}


/* Pitch-Yaw-Roll using world coordinate system
 *      X positive Forward (roll axis)
 *      Y positive to left (pitch axis)
 *      Z up (yaw axis)
 */
template<typename T>
RotationMatrix<T> MakePitchYawRoll(T pitch, T yaw, T roll)
{
    return MakeIntrinsic<1, 2, 0>(pitch, yaw, roll);
}


/* The rotation of the sensor axes relative to the world axes */
template<typename T>
RotationMatrix<T> SensorRelativeToWorld()
{
    // To move from world coordinates to sensor coordinates:
    // Yaw is -90
    // Pitch is 0
    // Roll is -90
    return MakeYawPitchRoll(
        static_cast<T>(-90),
        static_cast<T>(0),
        static_cast<T>(-90));
}

/* The rotation of the world axes relative to the sensor axes */
template<typename T>
RotationMatrix<T> WorldRelativeToSensor()
{
    // To move from sensor coordinates to world coordinates:
    // Yaw is 90
    // Pitch is -90
    // Roll is 0
    return MakeYawPitchRoll(
        static_cast<T>(90),
        static_cast<T>(-90),
        static_cast<T>(0));
}


extern template struct RotationAnglesModel<float>;
extern template struct RotationAnglesModel<double>;


} // namespace tau


extern template struct pex::Group
    <
        tau::RotationAnglesFields,
        tau::RotationAnglesTemplate<float>::template Template,
        tau::RotationAngles<float>
    >;

extern template struct pex::Group
    <
        tau::RotationAnglesFields,
        tau::RotationAnglesTemplate<double>::template Template,
        tau::RotationAngles<double>
    >;
