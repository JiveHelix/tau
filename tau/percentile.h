#pragma once


#include <algorithm>
#include <tau/eigen.h>


namespace tau
{


template<typename Derived>
auto Percentile(
    const Eigen::MatrixBase<Derived> &data,
    double percentile)
{
    using Eigen::Index;

    // Flatten and sort the data.
    auto flattened = data.derived().reshaped().eval();
    std::sort(std::begin(flattened), std::end(flattened));

    // Get the appropriate index
    auto valueCount = static_cast<double>(flattened.size());
    auto index = static_cast<Index>(std::round(valueCount * percentile));

    return flattened(index);
}


template<typename Derived, typename PercentileDerived>
auto Percentile(
    const Eigen::MatrixBase<Derived> &data,
    const Eigen::DenseBase<PercentileDerived> &percentiles)
{
    using Eigen::Index;
    using Float = typename PercentileDerived::Scalar;

    using Scalar = typename Derived::Scalar;
    using Result = Eigen::MatrixX<typename Derived::Scalar>;
    Result result(percentiles.rows(), percentiles.cols());

    // Flatten and sort the data.
    Eigen::VectorX<Scalar> flattened = data.derived().reshaped().eval();
    std::sort(std::begin(flattened), std::end(flattened));

    // Get the appropriate index
    auto valueCount = static_cast<Float>(flattened.size());

    using Indices = tau::MatrixLike<Index, decltype(result)>;
    Indices indices(result.rows(), result.cols());

    indices = (percentiles.derived().array() * valueCount).round()
        .template cast<Index>();

    result = flattened(
        indices.template reshaped<Eigen::AutoOrder>(),
        Eigen::all);

    return result;
}


} // end namespace tau
