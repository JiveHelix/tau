#pragma once

#include "tau/eigen.h"


namespace tau
{


template<typename Derived>
auto Bilinear(
    const Eigen::DenseBase<Derived> &source,
    Eigen::Index height,
    Eigen::Index width)
{
    using Traits = tau::MatrixTraits<Derived>;

    using FloatingPointResult =
        Eigen::Matrix
        <
            double,
            Eigen::Dynamic,
            Eigen::Dynamic,
            Traits::options
        >;

    if (height <= 0 || width <= 0)
    {
        throw std::logic_error("Size must be greater than 0.");
    }

    auto sourceHeight = static_cast<double>(source.rows());
    auto sourceWidth = static_cast<double>(source.cols());
    auto targetHeight = static_cast<double>(height);
    auto targetWidth = static_cast<double>(width);

    // Ratio converts from target index back to source index.
    auto xRatio = (width > 1)
        ? (sourceWidth - 1) / (targetWidth - 1)
        : 0.0;

    auto yRatio = (height > 1)
        ? (sourceHeight - 1) / (targetHeight - 1)
        : 0.0;

    FloatingPointResult result(height, width);

    using IndexVector = Eigen::Vector<double, Eigen::Dynamic>;

    // Map the x and y indices of the source data to the target.
    auto xIndices =
        IndexVector::LinSpaced(width, 0.0, targetWidth - 1).eval();

    IndexVector xSources = xIndices.array() * xRatio;

    IndexVector xSourcesLowAsDoubles = xSources.array().floor();
    auto xSourcesLow = xSourcesLowAsDoubles.template cast<Eigen::Index>();

    auto xSourcesHigh =
        xSources.array().ceil().template cast<Eigen::Index>().eval();

    IndexVector xWeights = xSources - xSourcesLowAsDoubles;
    IndexVector xCounterWeights = 1.0 - xWeights.array();

    auto yIndices =
        IndexVector::LinSpaced(height, 0.0, targetHeight - 1).eval();

    IndexVector ySources = yIndices.array() * yRatio;

    IndexVector ySourcesLowAsDoubles = ySources.array().floor();
    auto ySourcesLow = ySourcesLowAsDoubles.template cast<Eigen::Index>();

    auto ySourcesHigh =
        ySources.array().ceil().template cast<Eigen::Index>().eval();

    IndexVector yWeights = ySources - ySourcesLowAsDoubles;
    IndexVector yCounterWeights = 1.0 - yWeights.array();

    // Assuming column-major for now
    for (Eigen::Index i = 0; i < width; ++i)
    {
        auto xLow = xSourcesLow[i];
        auto xHigh = xSourcesHigh[i];
        auto xWeight = xWeights[i];
        auto xCounterWeight = xCounterWeights[i];

        for (Eigen::Index j = 0; j < height; ++j)
        {
            auto yLow = ySourcesLow[j];
            auto yHigh = ySourcesHigh[j];

            // Get the four source points surrounding this target point.
            auto topLeft = source(yLow, xLow);
            auto topRight = source(yLow, xHigh);
            auto bottomLeft = source(yHigh, xLow);
            auto bottomRight = source(yHigh, xHigh);

            auto topResult =
                topLeft * xCounterWeight + topRight * xWeight;

            auto bottomResult =
                bottomLeft * xCounterWeight + bottomRight * xWeight;

            result(j, i) =
                topResult * yCounterWeights[j] + bottomResult * yWeights[j];
        }
    }

    return result.array().round().template cast<typename Traits::type>().eval();
}


} // end namespace tau
