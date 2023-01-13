#pragma once

#include <fields/fields.h>
#include <pex/interface.h>
#include <tau/eigen.h>
#include <tau/gradient.h>
#include <tau/gaussian.h>
#include <tau/vector2d.h>


namespace tau
{


template<typename T>
struct HarrisFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::alpha, "alpha"),
        fields::Field(&T::sigma, "sigma"),
        fields::Field(&T::threshold, "threshold"));
};


struct HarrisRanges
{
    using AlphaLow = pex::Limit<0>;
    using AlphaHigh = pex::Limit<0, 25, 100>;

    using SigmaLow = pex::Limit<0, 25, 100>;
    using SigmaHigh = pex::Limit<4>;

    using ThresholdLow = pex::Limit<0>;
    using ThresholdHigh = pex::Limit<0, 25, 100>;
};


template<typename Float, typename Ranges = HarrisRanges>
struct HarrisTemplate
{
    static_assert(std::is_floating_point_v<Float>);

    template<template<typename> typename T>
    struct Template
    {
        T
        <
            pex::MakeRange
            <
                Float,
                typename Ranges::AlphaLow,
                typename Ranges::AlphaHigh
            >
        > alpha;

        T
        <
            pex::MakeRange
            <
                Float,
                typename Ranges::SigmaLow,
                typename Ranges::SigmaHigh
            >
        > sigma;

        T
        <
            pex::MakeRange
            <
                Float,
                typename Ranges::ThresholdLow,
                typename Ranges::ThresholdHigh
            >
        > threshold;

        static constexpr auto fields = HarrisFields<Template>::fields;
        static constexpr auto fieldsTypeName = "Harris";
    };
};


template<typename Float>
struct HarrisSettings
    :
    public HarrisTemplate<Float>::template Template<pex::Identity>
{
    static HarrisSettings Default()
    {
        static constexpr Float defaultAlpha = static_cast<float>(0.14);
        static constexpr Float defaultSigma = 1.5;
        static constexpr Float defaultThreshold = static_cast<Float>(0.01);

        return {{defaultAlpha, defaultSigma, defaultThreshold}};
    }
};


template<typename Float>
class Harris
{
public:
    Harris(const HarrisSettings<Float> &settings)
        :
        settings_(settings),
        gaussianKernel_(
            GaussianKernel<Float, 0>(
                settings.sigma,
                static_cast<Float>(0.01)).Normalize())
    {

    }

    template<typename Data>
    MatrixLike<Float, Data> Compute(const Gradient<Data> &gradient)
    {
        using FloatData = MatrixLike<Float, Data>;
        FloatData dx = gradient.dx.template cast<Float>();
        FloatData dy = gradient.dy.template cast<Float>();

        FloatData dxSquared = dx.array().square();
        FloatData dySquared = dy.array().square();
        FloatData dxdy = dx.array() * dy.array();

        FloatData windowedDxSquared =
            tau::DoConvolve(dxSquared, this->gaussianKernel_.rowKernel);

        FloatData windowedDySquared =
            tau::DoConvolve(dySquared, this->gaussianKernel_.columnKernel);

        FloatData windowedDxDy =
            tau::GaussianBlur(this->gaussianKernel_, dxdy);

        FloatData response =
            windowedDxSquared.array() * windowedDySquared.array()
            - windowedDxDy.array().square()
            - this->settings_.alpha
                * (windowedDxSquared.array() + windowedDySquared.array())
                    .square();

        return response;
    }

    template<typename Data>
    Data Threshold(Data response)
    {
        Float thresholdValue = this->settings_.threshold * response.maxCoeff();
        return (response.array() < thresholdValue).select(0, response);
    }

private:
    HarrisSettings<Float> settings_;
    GaussianKernel<Float, 0> gaussianKernel_;
};


} // end namespace tau
