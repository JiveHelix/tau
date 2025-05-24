#pragma once

#include <iterator>
#include <fields/fields.h>
#include <pex/group.h>
#include <pex/interface.h>
#include <cmath>

#include "tau/eigen_shim.h"
#include "tau/angles.h"
#include "tau/error.h"
#include "tau/arithmetic.h"


namespace tau
{


CREATE_EXCEPTION(RotationError, TauError);


template<typename T>
using RotationMatrix = Eigen::Matrix<T, 3, 3>;


template<size_t axis, typename T>
RotationMatrix<T> MakeAxial_rad(T rotation_rad)
{
    static_assert(axis < 3, "Designed for 3-D only");

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


template<size_t axis, typename T>
RotationMatrix<T> MakeAxial(T rotation_deg)
{
    return MakeAxial_rad<axis, T>(tau::ToRadians(rotation_deg));
}


template<size_t axis, typename T>
RotationMatrix<T> MakeAxial_deg(T rotation_deg)
{
    return MakeAxial_rad<axis, T>(tau::ToRadians(rotation_deg));
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


struct AxisOrder: public BasicArithmetic<size_t, AxisOrder>
{
    static const std::array<std::string, 3> axisNames;

    size_t first;
    size_t second;
    size_t third;

    static constexpr auto fields = std::make_tuple(
        fields::Field(&AxisOrder::first, "first"),
        fields::Field(&AxisOrder::second, "second"),
        fields::Field(&AxisOrder::third, "third"));

    constexpr AxisOrder()
        :
        first{},
        second{},
        third{}
    {

    }

    constexpr AxisOrder(size_t first_, size_t second_, size_t third_)
        :
        first(first_),
        second(second_),
        third(third_)
    {

    }

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


struct AxisOrderChoices
{
    using Type = AxisOrder;

    static std::vector<Type> GetChoices()
    {
        return {
            {2, 1, 0},
            {2, 0, 1},
            {1, 2, 0},
            {1, 0, 2},
            {0, 2, 1},
            {0, 1, 2}};
    }
};


template<typename T>
struct RotationAnglesFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::yaw, "yaw"),
        fields::Field(&T::pitch, "pitch"),
        fields::Field(&T::roll, "roll"),
        fields::Field(&T::axisOrder, "axisOrder"));
};


using AxisOrderSelect = pex::MakeSelect<AxisOrderChoices>;

static_assert(
    std::is_same_v
    <
        std::vector<typename AxisOrderChoices::Type>,
        decltype(AxisOrderChoices::GetChoices())
    >);

static_assert(pex::HasGetChoices<AxisOrderChoices>);


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
        T<pex::MakeSelect<AxisOrderChoices>> axisOrder;

        static constexpr auto fields = RotationAnglesFields<Template>::fields;
        static constexpr auto fieldsTypeName = "RotationAngles";
    };
};


template<typename Derived>
Eigen::Vector3<typename Derived::Scalar>
CanonicalEuler(
    const Eigen::MatrixBase<Derived>& rotationMatrix,
    Eigen::Index first,
    Eigen::Index second,
    Eigen::Index third)
{
    using T = typename Derived::Scalar;
    static constexpr auto tau = Angles<T>::tau;
    static constexpr auto pi = Angles<T>::pi;
    static constexpr auto halfPi = pi / 2;

    Eigen::Matrix<T, 3, 1> angles =
        rotationMatrix.eulerAngles(first, second, third);

    if (first == third)
    {
        // Proper euler angles are already in the expected range.
        return angles;
    }

    // Tait-Bryan angles

    if (std::abs(angles(1)) > halfPi)
    {
        // The middle rotation is outside of the range -90 to 90.
        // Correct angles to fix the range of the second rotation.
        // Get the sign of original middle angle
        T sign = (angles(1) > 0) ? +1 : -1;
        angles(0) += sign * pi;
        angles(1) = sign * (pi - std::abs(angles(1)));
        angles(2) += sign * pi;
    }

    // wrap all three back into (-pi, pi]
    for (int i = 0; i < 3; ++i)
    {
        angles(i) = std::remainder(angles(i), tau);
    }

    return angles;
}


template<typename T>
using RotationAnglesBase =
    typename RotationAnglesTemplate<T>::template Template<pex::Identity>;


template<typename T>
struct RotationAnglesTemplates_
{
    template<typename Base>
    struct Plain: public Base, public BasicArithmetic<T, Plain<Base>>
    {
        // 0: roll (about x)
        // 1: pitch (about y)
        // 2: yaw (about z)
        // default is yaw-pitch-roll
        static constexpr auto defaultAxisOrder = AxisOrder{2, 1, 0};

        Plain()
            :
            Base{0, 0, 0, defaultAxisOrder}
        {

        }

        Plain(
            T first,
            T second,
            T third,
            const AxisOrder &axisOrder_ = defaultAxisOrder)
        {
            this->axisOrder = axisOrder_;
            (*this)(axisOrder_.first) = first;
            (*this)(axisOrder_.second) = second;
            (*this)(axisOrder_.third) = third;

            // For AxisOrder{1, 2, 0}
            // (*this)(1) = pitch
            // (*this)(2) = yaw
            // (*this)(0) = roll
        }

        Plain(
            const RotationMatrix<T> &rotation,
            const AxisOrder &axisOrder_ = defaultAxisOrder)
        {
            using Vector = Eigen::Vector<T, 3>;

            using Eigen::Index;

            Vector angles = tau::ToDegrees(
                CanonicalEuler(
                    rotation,
                    static_cast<Index>(axisOrder_.first),
                    static_cast<Index>(axisOrder_.second),
                    static_cast<Index>(axisOrder_.third)));

            (*this)(axisOrder_.first) = angles(0);
            (*this)(axisOrder_.second) = angles(1);
            (*this)(axisOrder_.third) = angles(2);
            this->axisOrder = axisOrder_;
        }

        Plain ConvertToAxisOrder(const AxisOrder &axisOrder) const
        {
            return Plain(this->GetRotation(), axisOrder);
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
            return const_cast<Plain *>(this)->operator()(axis);
        }

        RotationMatrix<T> GetRotation() const
        {
            return MakeIntrinsic(
                this->axisOrder,
                (*this)(this->axisOrder.first),
                (*this)(this->axisOrder.second),
                (*this)(this->axisOrder.third));
        }

        template<typename U, typename Style = Round>
        auto Cast() const
        {
            using Result =
                typename RotationAnglesTemplates_<U>
                    ::template Plain<RotationAnglesBase<U>>;

            return CastFields<Result, U, Style>(*this);

        }
    };
};


template<typename T>
using RotationAnglesGroup =
    pex::Group
    <
        RotationAnglesFields,
        RotationAnglesTemplate<T>::template Template,
        RotationAnglesTemplates_<T>
    >;


template<typename T>
using RotationAngles = typename RotationAnglesGroup<T>::Plain;


DECLARE_OUTPUT_STREAM_OPERATOR(RotationAngles<float>)
DECLARE_OUTPUT_STREAM_OPERATOR(RotationAngles<double>)


template<typename T>
using RotationAnglesControl =
    typename RotationAnglesGroup<T>::Control;


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


} // namespace tau


extern template struct pex::Group
    <
        tau::RotationAnglesFields,
        tau::RotationAnglesTemplate<float>::template Template,
        tau::RotationAnglesTemplates_<float>
    >;

extern template struct pex::Group
    <
        tau::RotationAnglesFields,
        tau::RotationAnglesTemplate<double>::template Template,
        tau::RotationAnglesTemplates_<double>
    >;
