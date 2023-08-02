#pragma once

#include <Eigen/Dense>
#include <jive/version.h>
#include <fields/fields.h>
#include <fields/compare.h>
#include <pex/group.h>
#include <nlohmann/json.hpp>

#include "tau/vector3d.h"


namespace tau
{


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
    static constexpr auto version = jive::Version<uint8_t>(1, 0, 0);
    static constexpr auto metersPerMicron = static_cast<T>(1e-6);
    static constexpr auto millimetersPerMeter = static_cast<T>(1e3);

    using Matrix = Eigen::Matrix<T, 3, 3>;

    static Intrinsics Default()
    {
        return {{
            static_cast<T>(10),
            static_cast<T>(25),
            static_cast<T>(25),
            static_cast<T>(1920.0 / 2.0),
            static_cast<T>(1080.0 / 2.0),
            static_cast<T>(0)}};
    }

    template <typename Value>
    std::enable_if_t<std::is_floating_point_v<Value>, Value>
    MetersToPixel(const Value &meters) const
    {
        return meters / (this->pixelSize_um * metersPerMicron);
    }

    template<typename Value>
    Value MetersToPixel(const Eigen::MatrixBase<Value> &meters) const
    {
        return meters.array() / (this->pixelSize_um * metersPerMicron);
    }

    template<typename Value>
    tau::Point3d<Value>
    MetersToPixel(const tau::Point3d<Value> &meters) const
    {
        return meters / (this->pixelSize_um * metersPerMicron);
    }

    template <typename Value>
    std::enable_if_t<std::is_floating_point_v<Value>, Value>
    PixelsToMeters(const Value &pixels) const
    {
        return pixels * (this->pixelSize_um * metersPerMicron);
    }

    template <typename Value>
    Value PixelsToMeters(const Eigen::MatrixBase<Value> &pixels) const
    {
        return pixels.array() * (this->pixelSize_um * metersPerMicron);
    }

    template <typename Value>
    tau::Point3d<Value> PixelsToMeters(
        const tau::Point3d<Value> &pixels) const
    {
        return pixels * (this->pixelSize_um * metersPerMicron);
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

    Matrix GetArray_pixels() const
    {
        auto focalLengthX_pixels = this->MetersToPixel(
            this->focalLengthX_mm / millimetersPerMeter);

        auto focalLengthY_pixels = this->MetersToPixel(
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
        auto focalLengthX_pixels = this->MetersToPixel(
            this->focalLengthX_mm / millimetersPerMeter);

        auto focalLengthY_pixels = this->MetersToPixel(
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
};


static_assert(pex::HasDefault<Intrinsics<float>>);


TEMPLATE_OUTPUT_STREAM(Intrinsics)


template<typename T>
using IntrinsicsGroup =
    pex::Group
    <
        IntrinsicsFields,
        IntrinsicsTemplate<T>::template Template,
        Intrinsics<T>
    >;

template<typename T>
using IntrinsicsModel = typename IntrinsicsGroup<T>::Model;

template<typename T>
using IntrinsicsControl = typename IntrinsicsGroup<T>::Control;


} // end namespace tau
