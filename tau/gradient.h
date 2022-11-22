#pragma once

#include <cmath>
#include <tau/eigen.h>
#include <tau/convolve.h>
#include <tau/planar.h>
#include <tau/color.h>
#include <tau/color_maps/rgb.h>
#include <pex/select.h>


namespace tau
{


struct DerivativeSize
{
    enum class Size
    {
        three,
        five,
        seven
    };

    using Index = Eigen::Index;

    using Select = pex::model::Select<Size>;

    using Control =
        pex::control::Select<void, Select>;

    static Index GetSize(Size size)
    {
        switch (size)
        {
            case Size::three:
                return 3;

            case Size::five:
                return 5;

            case Size::seven:
                return 7;

            default:
                throw tau::TauError("Not a supported size");
        }
    }

    struct SizeToString
    {
        static std::string ToString(Size size)
        {
            return std::to_string(GetSize(size));
        }
    };

    static std::vector<Size> GetValidSizes()
    {
        return {Size::three, Size::five, Size::seven};
    }

    template<typename T>
    static Eigen::VectorX<T> GetKernel(Index size, T scale)
    {
        switch (size)
        {
            case 3:
                return Eigen::VectorX<T>{{-scale, 0, scale}};

            case 5:
                return Eigen::VectorX<T>{
                    {-scale, -2 * scale, 0, 2 * scale, scale}};

            case 7:
                return Eigen::VectorX<T>{{
                    -scale,
                    -2 * scale,
                    -5 * scale,
                    0,
                    5 * scale,
                    2 * scale,
                    scale}};

            default:
                throw tau::TauError("Not a supported size");
        }
    }

    static signed GetWeight(Index size)
    {
        // Returns the sum of all (unscaled) positive terms of the kernel.

        switch (size)
        {
            case 3:
                return 1;

            case 5:
                return 3;

            case 7:
                return 7;

            default:
                throw tau::TauError("Not a supported size");
        }
    }
};


inline
std::ostream & operator<<(std::ostream &outputStream, DerivativeSize::Size size)
{
    return outputStream << DerivativeSize::SizeToString::ToString(size);
}


template<typename T>
struct Differentiate
{
    static_assert(sizeof(T) >= 2);
    static_assert(std::is_signed_v<T>);

    using RowVector = Eigen::Matrix<T, 1, Eigen::Dynamic>;
    using ColumnVector = Eigen::Matrix<T, Eigen::Dynamic, 1>;

    Differentiate(T maximumInput, T scale, DerivativeSize::Size size)
        :
        maximumInput_(maximumInput),
        scale_{scale},
        size_{DerivativeSize::GetSize(size)},
        horizontal(DerivativeSize::GetKernel<T>(this->size_, this->scale_)),
        vertical(this->horizontal.transpose())
    {
        if (maximumInput * DerivativeSize::GetWeight(this->size_)
                > std::numeric_limits<T>::max())
        {
            // Overflow may occur.
            throw tau::TauError(
                "Choose a larger data type to accomodate larger inputs.");
        }

        this->SetScale(scale);
    }

    T GetScale() const
    {
        return this->scale_;
    }

    T SetScale(T scale)
    {
        auto weight = DerivativeSize::GetWeight(this->size_);
        auto weightedScale = scale * weight;

        if (std::numeric_limits<T>::max() / weightedScale < this->maximumInput_)
        {
            weightedScale = std::numeric_limits<T>::max() / this->maximumInput_;
        }

        this->scale_ = weightedScale / weight;

        // Maximum weight is 7, and the minimum sizeof(T) is 2, and
        // maximumInput_ has been checked to be low enough.
        assert(this->scale_ >= 1);

        this->horizontal =
            DerivativeSize::GetKernel<T>(this->size_, this->scale_);

        this->vertical = this->horizontal.transpose();

        return this->scale_;
    }

    T GetMaximum() const
    {
        return this->maximumInput_
            * DerivativeSize::GetWeight(this->size_);
    }

    RowVector GetHorizontal() const
    {
        return this->horizontal;
    }

    ColumnVector GetVertical() const
    {
        return this->horizontal;
    }

    template<typename Data>
    Data X(const Eigen::MatrixBase<Data> &data) const
    {
        return tau::DoConvolve(data, this->horizontal);
    }

    template<typename Data>
    Data Y(const Eigen::MatrixBase<Data> &data) const
    {
        return tau::DoConvolve(data, this->vertical);
    }

private:
    T maximumInput_;
    T scale_;
    Eigen::Index size_;

public:
    RowVector horizontal;
    ColumnVector vertical;
};


template
<
    typename T,
    typename Enable = std::enable_if_t<std::is_floating_point_v<T>>
>
using HsvPlanes =
    tau::Planar<3, T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;


template<typename Data>
struct Gradient
{
    using Scalar = typename Data::Scalar;

    Gradient() = default;

    Gradient(
        const Differentiate<Scalar> &differentiate,
        const Eigen::MatrixBase<Data> &data)
        :
        maximum(differentiate.GetMaximum()),
        dx(differentiate.X(data)),
        dy(differentiate.Y(data))
    {

    }

    template<typename Float>
    static Eigen::MatrixX<Float> GetMagnitude(
        const Eigen::MatrixX<Float> &dx,
        const Eigen::MatrixX<Float> &dy)
    {
        static_assert(std::is_floating_point_v<Float>);

        Eigen::MatrixX<Float> result =
            (dx.array().square() + dy.array().square()).sqrt();

        // Clamp all magnitudes higher than one.
        return (result.array() > 1).select(1, result);
    }

    template<typename Float>
    static Eigen::MatrixX<Float> GetPhase(
        const Eigen::MatrixX<Float> &dx,
        const Eigen::MatrixX<Float> &dy)
    {
        static_assert(std::is_floating_point_v<Float>);

        Eigen::MatrixX<Float> asRadians = dy.array().binaryExpr(
            dx.array(),
            [](auto y, auto x) { return std::atan2(y, x); });

        return tau::ToDegrees(asRadians);
    }

    template<typename Pixel>
    RgbPixels<Pixel> Colorize() const
    {
        using Matrix = Eigen::MatrixX<float>;

        Matrix dxFloat = this->dx.template cast<float>();
        Matrix dyFloat = this->dy.template cast<float>();

        // Scale the derivates to -1 to 1.
        dxFloat.array() /= static_cast<float>(this->maximum);
        dyFloat.array() /= static_cast<float>(this->maximum);

        Matrix magnitude = GetMagnitude(dxFloat, dyFloat);
        Matrix phase = GetPhase(dxFloat, dyFloat);

        phase.array() += 360;
        phase = tau::Modulo(phase, 360);

        HsvPlanes<float> hsv(magnitude.rows(), magnitude.cols());

        GetSaturation(hsv).array() = 1.0;

        GetHue(hsv) = phase;
        GetValue(hsv) = magnitude;

        auto asRgb = tau::HsvToRgb<Pixel>(hsv);

        return {
            asRgb.template GetInterleaved<Eigen::RowMajor>(),
            magnitude.rows(),
            magnitude.cols()};
    }

    Scalar maximum;
    Data dx;
    Data dy;
};


} // end namespace tau
