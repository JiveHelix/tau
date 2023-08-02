#pragma once


#include <algorithm>
#include <tau/eigen.h>


namespace tau
{


template<typename Derived>
auto Percentile(
    const Eigen::DenseBase<Derived> &data,
    double percentile,
    bool isSorted = false)
{
    using Eigen::Index;

    // Get the appropriate index
    auto valueCount = static_cast<double>(data.size());
    auto index = static_cast<Index>(std::floor(valueCount * percentile));

    if (!isSorted)
    {
        // Flatten and sort the data.
        auto flattened = data.derived().reshaped().eval();
        std::sort(std::begin(flattened), std::end(flattened));
        return flattened(index);
    }

    return data.derived()(index);
}


template<typename Derived>
Eigen::VectorX<typename Derived::Scalar> RemoveZeros(
    const Eigen::DenseBase<Derived> &data)
{
    using Scalar = typename Derived::Scalar;
    Eigen::VectorX<Scalar> flattened = data.derived().reshaped().eval();
    auto begin = std::begin(flattened);
    auto end = std::end(flattened);

    auto filteredEnd = std::remove_if(
        begin,
        end,
        [](Scalar value) { return value == 0; });

    if (filteredEnd != end)
    {
        flattened.conservativeResize(std::distance(begin, filteredEnd));
    }

    return flattened;
}


template<typename Derived>
Eigen::VectorX<typename Derived::Scalar> FilterLessThan(
    const Eigen::DenseBase<Derived> &data,
    typename Derived::Scalar threshold)
{
    using Scalar = typename Derived::Scalar;
    Eigen::VectorX<Scalar> flattened = data.derived().reshaped().eval();
    auto begin = std::begin(flattened);
    auto end = std::end(flattened);

    auto filteredEnd = std::remove_if(
        begin,
        end,
        [threshold](Scalar value) { return value < threshold; });

    if (filteredEnd != end)
    {
        flattened.conservativeResize(std::distance(begin, filteredEnd));
    }

    return flattened;
}


template<typename Derived, typename PercentileDerived>
auto Percentile(
    const Eigen::DenseBase<Derived> &data,
    const Eigen::DenseBase<PercentileDerived> &percentiles)
{
    using Eigen::Index;
    using Float = typename PercentileDerived::Scalar;
    using Scalar = typename Derived::Scalar;

    // Flatten and sort the data.
    Eigen::VectorX<Scalar> flattened = data.derived().reshaped().eval();
    std::sort(std::begin(flattened), std::end(flattened));

    using Result = Eigen::MatrixX<typename Derived::Scalar>;
    Result result(percentiles.rows(), percentiles.cols());

    // Get the appropriate index
    auto valueCount = static_cast<Float>(flattened.size());

    using Indices = tau::MatrixLike<Index, decltype(result)>;
    Indices indices(result.rows(), result.cols());

    indices = (percentiles.derived().array() * valueCount).floor()
        .template cast<Index>();

    return flattened(
        indices.template reshaped<Eigen::AutoOrder>(),
        Eigen::all).eval();
}


template<typename Derived, typename PercentileDerived>
auto PresortedPercentile(
    const Eigen::DenseBase<Derived> &data,
    const Eigen::DenseBase<PercentileDerived> &percentiles)
{
    using Eigen::Index;
    using Float = typename PercentileDerived::Scalar;

    using Result = Eigen::MatrixX<typename Derived::Scalar>;
    Result result(percentiles.rows(), percentiles.cols());

    // Get the appropriate index
    auto valueCount = static_cast<Float>(data.size());

    using Indices = tau::MatrixLike<Index, decltype(result)>;
    Indices indices(result.rows(), result.cols());

    indices = (percentiles.derived().array() * valueCount).floor()
        .template cast<Index>();

    return data.derived().reshaped()(
        indices.template reshaped<Eigen::AutoOrder>(),
        Eigen::all).eval();
}


template<typename Scalar>
struct Quartiles
{
    Scalar lower;
    Scalar median;
    Scalar upper;

    Scalar Range() const
    {
        return this->upper - this->lower;
    }

    Scalar LowerLimit(Scalar scale = 1.5)
    {
        return this->median - this->Range() * scale;
    }

    Scalar UpperLimit(Scalar scale = 1.5)
    {
        return this->median + this->Range() * scale;
    }
};


template<typename Scalar>
std::ostream & operator<<(
    std::ostream &outputStream,
    const Quartiles<Scalar> &quartiles)
{
    return outputStream << "Quartiles{"
        << quartiles.lower << ", "
        << quartiles.median << ", "
        << quartiles.upper << ", "
        << "range: " << quartiles.Range() << "}" << std::endl;
}


template<typename Derived>
Quartiles<typename Derived::Scalar> GetQuartiles(
    const Eigen::DenseBase<Derived> &values)
{
    auto quartiles = Percentile(
        values,
        Eigen::Vector3<double>(0.25, 0.5, 0.75));

    return {quartiles(0), quartiles(1), quartiles(2)};
}


} // end namespace tau
