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

#include "tau/eigen.h"
#include "tau/color_maps/rgb.h"

#include <iostream>

namespace tau
{

template<typename ColorType>
class ColorMap
{
public:
    using Colors = ColorType;
    
    using ColorsTraits = MatrixTraits<Colors>;

    static_assert(ColorsTraits::columns != Eigen::Dynamic);

    static constexpr auto pixelSizeBytes =
        ColorsTraits::columns * sizeof(typename ColorsTraits::type);

    ColorMap(const Colors &map)
        :
        map_(map)
    {

    }

    template<typename Input, typename Output>
    void operator()(const Input &input, Output *output)
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


template<typename Input, typename Bound>
void Constrain(Input &input, Bound minimum, Bound maximum)
{
    using traits = MatrixTraits<Input>;
    using type = typename traits::type;

    if constexpr (std::is_same_v<type, Bound>)
    {
        Select(input) < minimum = minimum;
        Select(input) > maximum = maximum;
    }
    else
    {
        auto typedMinimum = static_cast<type>(minimum);
        auto typedMaximum = static_cast<type>(maximum);
        Select(input) < typedMinimum = typedMinimum;
        Select(input) > typedMaximum = typedMaximum;
    }
}


template<typename Bound, typename Float = double>
class FloatRescale
{
public:
    FloatRescale(Bound minimum, Bound maximum)
        :
        minimum_(minimum),
        maximum_(maximum)
    {
        assert(minimum < maximum);
    }

    template<typename Input>
    auto operator()(Eigen::Index count, const Input &input) const
    {
        assert(count > 0);

        Float factor = static_cast<Float>(count - 1)
            / static_cast<Float>(this->maximum_ - this->minimum_);

        // Convert to floating-point for the rescaling operations.
        MatrixLike<Float, Input> asFloat = input.template cast<Float>();

        Constrain(asFloat, this->minimum_, this->maximum_);

        asFloat.array() -= static_cast<Float>(this->minimum_);
        asFloat.array() *= factor;

        // Cast back to integral values to use as indices.
        // eval is used tp force evaluation before asFloat goes out of scope.
        return asFloat.array().round().template cast<Eigen::Index>().eval();
    }

private:
    Bound minimum_;
    Bound maximum_;
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
    auto operator()(Input input) const
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
class ScaledColorMap: public ColorMap<ColorType>
{
public:
    using Base = ColorMap<ColorType>;

    ScaledColorMap(const ColorType &map, Bound minimum, Bound maximum)
        :
        Base(map),
        rescale_(minimum, maximum)
    {

    }

    template<typename Input, typename Output>
    void operator()(const Input &input, Output *output) const
    {
        *output = this->map_(
            this->rescale_(
                this->map_.rows(),
                input).template reshaped<Eigen::AutoOrder>(),
            Eigen::all).eval();

        assert(output->rows() == input.size());
    }

private:
    FloatRescale<Bound, Float> rescale_;
};


template<typename ColorType, typename Bound>
class LimitedColorMap: public ColorMap<ColorType>
{
public:
    using Base = ColorMap<ColorType>;

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
    void operator()(const Input &input, Output *output) const
    {
        *output = this->map_(
            this->rescale_(input).template reshaped<Eigen::AutoOrder>(),
            Eigen::all).eval();

        assert(output->rows() == input.size());
    }

private:
    Rescale<Bound> rescale_;
};



} // end namespace tau
