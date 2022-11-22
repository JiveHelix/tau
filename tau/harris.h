#pragma once

#include <fields/fields.h>
#include <pex/interface.h>
#include <tau/gradient.h>
#include <tau/gaussian.h>


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


template<typename Float>
struct HarrisTemplate
{
    static_assert(std::is_floating_point_v<Float>);

    using AlphaLow = pex::Limit<0, 3, 100>;
    using AlphaHigh = pex::Limit<0, 7, 100>;

    using SigmaLow = pex::Limit<0, 25, 100>;
    using SigmaHigh = pex::Limit<4>;

    using ThresholdLow = pex::Limit<0>;
    using ThresholdHigh = pex::Limit<1>;

    template<template<typename> typename T>
    struct Template
    {
        T<pex::MakeRange<Float, AlphaLow, AlphaHigh>> alpha;
        T<pex::MakeRange<Float, SigmaLow, SigmaHigh>> sigma;
        T<pex::MakeRange<Float, ThresholdLow, ThresholdHigh>> threshold;

        static constexpr auto fields = HarrisFields<Template>::fields;
        static constexpr auto fieldsTypeName = "Harris";
    };
};


template<typename Float>
using HarrisBase =
    typename HarrisTemplate<Float>::template Template<pex::Identity>;


template<typename Float>
struct HarrisSettings: public HarrisBase<Float>
{
    static HarrisSettings Default()
    {
        static constexpr Float defaultAlpha = static_cast<float>(0.04);
        static constexpr Float defaultSigma = 1;
        static constexpr Float defaultThreshold = static_cast<Float>(0.5);

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

    using Matrix = Eigen::MatrixX<Float>;

    template<typename Data>
    Matrix Compute(const Gradient<Data> &gradient)
    {
        Matrix dx = gradient.dx.template cast<Float>();
        Matrix dy = gradient.dy.template cast<Float>();

        Matrix dxSquared = dx.array().square();
        Matrix dySquared = dy.array().square();
        Matrix dxdy = dx.array() * dy.array();

        Matrix windowedDxSquared =
            tau::DoConvolve(dxSquared, this->gaussianKernel_.rowKernel);

        Matrix windowedDySquared =
            tau::DoConvolve(dySquared, this->gaussianKernel_.columnKernel);

        Matrix windowedDxDy =
            tau::GaussianBlur(this->gaussianKernel_, dxdy);

        Matrix response =
            windowedDxSquared.array() * windowedDySquared.array()
            - windowedDxDy.array().square()
            - this->settings_.alpha
                * (windowedDxSquared.array() + windowedDySquared.array())
                    .square();

        return response;
    }

    Matrix Threshold(Matrix response)
    {
        Float thresholdValue = this->settings_.threshold * response.maxCoeff();
        return (response.array() < thresholdValue).select(0, response);
    }

private:
    HarrisSettings<Float> settings_;
    GaussianKernel<Float, 0> gaussianKernel_;
};




} // end namespace tau
