#pragma once

#include <jive/version.h>
#include <fields/fields.h>
#include <fields/compare.h>
#include <pex/group.h>
#include <nlohmann/json.hpp>

#include "tau/eigen_shim.h"
#include "tau/vector3d.h"


namespace tau
{


template<typename T>
class PixelConvert
{
    T pixelSize_um_;

public:
    static constexpr auto metersPerMicron = static_cast<T>(1e-6);

    PixelConvert()
        :
        pixelSize_um_(10)
    {

    }

    PixelConvert(T pixelSize_um)
        :
        pixelSize_um_(pixelSize_um)
    {
        if (pixelSize_um == static_cast<T>(0))
        {
            throw std::runtime_error("Invalid pixel size");
        }
    }

    template <typename Value>
    std::enable_if_t<std::is_floating_point_v<Value>, Value>
    PixelsToMeters(const Value &pixels) const
    {
        return pixels * (this->pixelSize_um_ * metersPerMicron);
    }

    template <typename Value>
    Value PixelsToMeters(const Eigen::MatrixBase<Value> &pixels) const
    {
        return pixels.array() * (this->pixelSize_um_ * metersPerMicron);
    }

    template <typename Value>
    tau::Point3d<Value> PixelsToMeters(
        const tau::Point3d<Value> &pixels) const
    {
        return pixels * (this->pixelSize_um_ * metersPerMicron);
    }

    template <typename Value>
    std::enable_if_t<std::is_floating_point_v<Value>, Value>
    MetersToPixels(const Value &meters) const
    {
        return meters / (this->pixelSize_um_ * metersPerMicron);
    }

    template<typename Value>
    Value MetersToPixels(const Eigen::MatrixBase<Value> &meters) const
    {
        return meters.array() / (this->pixelSize_um_ * metersPerMicron);
    }

    template<typename Value>
    tau::Point3d<Value>
    MetersToPixels(const tau::Point3d<Value> &meters) const
    {
        return meters / (this->pixelSize_um_ * metersPerMicron);
    }

    template<typename U>
    PixelConvert<U> Cast() const
    {
        if constexpr (std::is_same_v<U, T>)
        {
            return *this;
        }

        return PixelConvert<U>(static_cast<U>(this->pixelSize_um_));
    }
};


template<typename T>
struct IntrinsicsFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::pixelSize_um, "pixelSize_um"),
        fields::Field(&T::focalLengthX_mm, "focalLengthX_mm"),
        fields::Field(&T::focalLengthY_mm, "focalLengthY_mm"),
        fields::Field(&T::principalX_pixels, "principalX_pixels"),
        fields::Field(&T::principalY_pixels, "principalY_pixels"),
        fields::Field(&T::skew, "skew"));
};


template<typename Float>
struct IntrinsicsTemplate
{
    template<template<typename> typename T>
    struct Template
    {
        T<Float> pixelSize_um;
        T<Float> focalLengthX_mm;
        T<Float> focalLengthY_mm;
        T<Float> principalX_pixels;
        T<Float> principalY_pixels;
        T<Float> skew;

        static constexpr auto fields =
            IntrinsicsFields<Template>::fields;

        static constexpr auto fieldsTypeName = "Intrinsics";
    };
};


template<typename T>
struct Intrinsics:
    public IntrinsicsTemplate<T>::template Template<pex::Identity>
{
    using Base =
        typename IntrinsicsTemplate<T>::template Template<pex::Identity>;

    PixelConvert<T> pixelConvert;

    static constexpr auto version = jive::Version<uint8_t>(1, 0, 0);
    static constexpr auto millimetersPerMeter = static_cast<T>(1e3);

    using Matrix = Eigen::Matrix<T, 3, 3>;

    Intrinsics()
        :
        Base(
            {
                static_cast<T>(10),
                static_cast<T>(25),
                static_cast<T>(25),
                static_cast<T>(1920.0 / 2.0),
                static_cast<T>(1080.0 / 2.0),
                static_cast<T>(0)}),
        pixelConvert(this->pixelSize_um)
    {

    }

    Intrinsics(const Base &base)
        :
        Base(base),
        pixelConvert(this->pixelSize_um)
    {

    }

    template<typename Value>
    auto MetersToPixels(const Value &meters) const
    {
        return this->pixelConvert.MetersToPixels(meters);
    }

    template<typename Value>
    auto PixelsToMeters(const Value &pixels) const
    {
        return this->pixelConvert.PixelsToMeters(pixels);
    }

    static Intrinsics FromArray(
        T pixelSize_um_,
        const Matrix &array_pixels)
    {
        Intrinsics result;
        result.pixelSize_um = pixelSize_um_;
        T focalLengthX_m = result.PixelsToMeters(array_pixels(0, 0));
        T focalLengthY_m = result.PixelsToMeters(array_pixels(1, 1));
        result.focalLengthX_mm = focalLengthX_m * millimetersPerMeter;
        result.focalLengthY_mm = focalLengthY_m * millimetersPerMeter;
        result.skew = array_pixels(0, 1);
        result.principalX_pixels = array_pixels(0, 2);
        result.principalY_pixels = array_pixels(1, 2);

        return result;
    }

    T GetFocalLength_m() const
    {
        static constexpr auto half = static_cast<T>(0.5);
        return
            (this->focalLengthX_mm + this->focalLengthY_mm)
            * half
            / millimetersPerMeter;
    }

    T GetFocalLength_pixels() const
    {
        return this->MetersToPixels(this->GetFocalLength_mm());
    }

    Matrix GetArray_pixels() const
    {
        auto focalLengthX_pixels = this->MetersToPixels(
            this->focalLengthX_mm / millimetersPerMeter);

        auto focalLengthY_pixels = this->MetersToPixels(
            this->focalLengthY_mm / millimetersPerMeter);

        Matrix m{
            {focalLengthX_pixels, this->skew, this->principalX_pixels},
            {0.0, focalLengthY_pixels, this->principalY_pixels},
            {0.0, 0.0, 1.0}};

        return m;
    }

    Matrix GetArray_m() const
    {
        return this->PixelsToMeters(this->GetArray_pixels());
    }

    Matrix GetInverse_pixels() const
    {
        auto focalLengthX_pixels = this->MetersToPixels(
            this->focalLengthX_mm / millimetersPerMeter);

        auto focalLengthY_pixels = this->MetersToPixels(
            this->focalLengthY_mm / millimetersPerMeter);

        auto inversePrincipalX =
            (this->principalY_pixels * this->skew
             - this->principalX_pixels * focalLengthY_pixels);

        auto inversePrincipalY = -this->principalY_pixels * focalLengthY_pixels;

        auto focalProduct = focalLengthX_pixels * focalLengthY_pixels;

        Matrix m{
            {focalLengthY_pixels, -this->skew, inversePrincipalX},
            {0.0, focalLengthX_pixels, inversePrincipalY},
            {0.0, 0.0, focalProduct}};

        return m / focalProduct;
    }

    static Intrinsics Deserialize(const std::string &asString)
    {
        auto unstructured = nlohmann::json::parse(asString);
        auto fileVersion = jive::Version<uint8_t>(unstructured["version"]);
        auto minimumVersion = jive::Version<uint8_t>(1, 0, 0);

        if (fileVersion < minimumVersion)
        {
            throw std::runtime_error("Incompatible file version");
        }

        return fields::Structure<Intrinsics>(unstructured);
    }

    std::string Serialize() const
    {
        auto unstructured = fields::Unstructure<nlohmann::json>(*this);
        unstructured["version"] = Intrinsics::version.ToString();
        return unstructured.dump(4);
    }

    template<typename U, typename Style = Round>
    Intrinsics<U> Cast() const
    {
        return CastFields<Intrinsics<U>, U, Style>(*this);
    }
};


DECLARE_OUTPUT_STREAM_OPERATOR(Intrinsics<float>)
DECLARE_OUTPUT_STREAM_OPERATOR(Intrinsics<double>)
DECLARE_EQUALITY_OPERATORS(Intrinsics<float>)
DECLARE_EQUALITY_OPERATORS(Intrinsics<double>)


template<typename T>
using IntrinsicsGroup =
    pex::Group
    <
        IntrinsicsFields,
        IntrinsicsTemplate<T>::template Template,
        pex::PlainT<Intrinsics<T>>
    >;

template<typename T>
using IntrinsicsModel = typename IntrinsicsGroup<T>::Model;

template<typename T>
using IntrinsicsControl = typename IntrinsicsGroup<T>::Control;


} // end namespace tau
