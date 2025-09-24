/**
  * @file color_map.h
  *
  * @brief Apply color maps.
  *
  * @author Jive Helix (jivehelix@gmail.com)
  * @date 07 Feb 2022
  * @copyright Jive Helix
  * Licensed under the MIT license. See LICENSE file.
**/

#pragma once

#include <tau/eigen.h>
#include <tau/color_maps/rgb.h>
#include <tau/color_maps/turbo.h>
#include <tau/color_maps/gray.h>
#include <tau/mono_image.h>
#include <tau/color_map_settings.h>


namespace tau
{

template<typename ColorType>
class BasicColorMap
{
public:
    using Colors = ColorType;

    using ColorsTraits = MatrixTraits<Colors>;

    static_assert(ColorsTraits::columns != Eigen::Dynamic);

    static constexpr auto pixelSizeBytes =
        ColorsTraits::columns * sizeof(typename ColorsTraits::type);

    BasicColorMap(const Colors &map)
        :
        map_(map)
    {

    }

    template<typename Input, typename Output>
    void operator()(
        const Eigen::MatrixBase<Input> &input,
        Eigen::MatrixBase<Output> *output)
    {
        using InputTraits = MatrixTraits<Input>;

        static_assert(
            std::is_integral_v<typename InputTraits::type>,
            "Input must be integral");

        *output = this->map_(
            input.template reshaped<Eigen::AutoOrder>(),
            Eigen::all).eval();

        assert(output->rows() == input.size());
    }

protected:
    Colors map_;
};


template<typename Bound, typename Float = double>
class FloatRescale
{
public:
    FloatRescale()
        :
        minimum_(0),
        maximum_(255),
        factor_(1)
    {

    }

    FloatRescale(Eigen::Index count, Bound minimum, Bound maximum)
        :
        minimum_(minimum),
        maximum_(maximum),
        factor_()
    {
        assert(count > 0);
        assert(minimum < maximum);

        this->factor_ = static_cast<Float>(count - 1)
            / static_cast<Float>(this->maximum_ - this->minimum_);
    }

    template<typename Input>
    auto operator()(const Eigen::MatrixBase<Input> &input) const
    {
        // Convert to floating-point for the rescaling operations.
        MatrixLike<Float, Input> asFloat =
            input.template cast<Float>();

        Constrain(asFloat, this->minimum_, this->maximum_);

        asFloat.array() -= static_cast<Float>(this->minimum_);
        asFloat.array() *= this->factor_;

        // Cast back to integral values to use as indices.
        // eval is used to force evaluation before asFloat goes out of scope.
        return asFloat.array().round()
            .template cast<typename Input::Scalar>().eval();
    }

private:
    Bound minimum_;
    Bound maximum_;
    Float factor_;
};


template<typename Bound>
class Rescale
{
public:
    Rescale(Bound minimum, Bound maximum)
        :
        minimum_(minimum),
        maximum_(maximum)
    {
        assert(minimum < maximum);
    }

    template<typename Input>
    Input operator()(Eigen::MatrixBase<Input> &input) const
    {
        Constrain(input, this->minimum_, this->maximum_);
        input.array() -= static_cast<typename Input::Scalar>(this->minimum_);
        return input;
    }

private:
    Bound minimum_;
    Bound maximum_;
};


template<typename Input>
using IndexMatrix = MatrixLike<Eigen::Index, Input>;


template<
    typename ColorType,
    typename Bound,
    typename Float = double>
class ScaledColorMap: public BasicColorMap<ColorType>
{
public:
    using Base = BasicColorMap<ColorType>;

    ScaledColorMap(const ColorType &map, Bound minimum, Bound maximum)
        :
        Base(map),
        rescale_(this->map_.rows(), minimum, maximum)
    {

    }

    template<typename Input, typename Output>
    void operator()(
        const Eigen::MatrixBase<Input> &input,
        Eigen::MatrixBase<Output> *output) const
    {
        Input rescaled = input;

        *output = this->map_(
            this->rescale_(rescaled).template reshaped<Eigen::AutoOrder>(),
            Eigen::all).eval();

        assert(output->rows() == input.size());
    }

private:
    FloatRescale<Bound, Float> rescale_;
};


template<typename ColorType, typename Bound>
class LimitedColorMap: public BasicColorMap<ColorType>
{
public:
    using Base = BasicColorMap<ColorType>;

    LimitedColorMap(const ColorType &map, Bound minimum, Bound maximum)
        :
        Base(map),
        rescale_(minimum, maximum)
    {
        if ((maximum - minimum + 1) != map.rows())
        {
            throw std::invalid_argument(
                "color map must match the size of the range.");
        }
    }

    template<typename Input, typename Output>
    void operator()(
        const Eigen::MatrixBase<Input> &input,
        Eigen::MatrixBase<Output> *output) const
    {
        Input rescaled = input;

        *output = this->map_(
            this->rescale_(rescaled).template reshaped<Eigen::AutoOrder>(),
            Eigen::all).eval();

        assert(output->rows() == input.size());
    }

private:
    Rescale<Bound> rescale_;
};


template<typename Pixels, typename T>
auto MakeColorMap(const ColorMapSettings<T> &colorMapSettings)
{
    using PixelMatrix = typename Pixels::Data;
    auto low = colorMapSettings.range.low;
    auto high = colorMapSettings.range.high;

    static constexpr auto maximum =
        static_cast<decltype(low)>(std::numeric_limits<T>::max());

    high = std::min(maximum, high);
    low = std::min(high - 1, low);

    assert(low < high);

    size_t count = static_cast<size_t>(1 + high - low);

    if (colorMapSettings.turbo)
    {
        return tau::LimitedColorMap<PixelMatrix, T>(
            tau::turbo::MakeRgb8(count),
            static_cast<T>(low),
            static_cast<T>(high));
    }
    else
    {
        return tau::LimitedColorMap<PixelMatrix, T>(
            tau::gray::MakeRgb8(count),
            static_cast<T>(low),
            static_cast<T>(high));
    }
}


template<typename Value>
class ColorMap
{
public:
    using Matrix = MonoImage<Value>;
    using Pixels = RgbPixels<uint8_t>;

    ColorMap(const ColorMapSettings<Value> &colorMapSettings)
        :
        colorMap_(MakeColorMap<Pixels>(colorMapSettings))
    {

    }

    Pixels Filter(const Matrix &data) const
    {
        RgbPixels<uint8_t> result{{}, {data.cols(), data.rows()}};
        this->colorMap_(data, &result.data);

        return result;
    }

protected:
    LimitedColorMap<typename Pixels::Data, Value> colorMap_;
};


extern template class ColorMap<int32_t>;


} // end namespace tau
