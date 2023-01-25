#pragma once

#include <cassert>
#include <cmath>
#include <fields/fields.h>
#include <jive/overflow.h>

#include "tau/eigen.h"
#include "tau/convolve.h"
#include "tau/angles.h"



namespace tau
{


template<typename T>
struct GaussianFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::sigma, "sigma"),
        fields::Field(&T::threshold, "threshold"),
        fields::Field(&T::size, "size"),
        fields::Field(&T::rowKernel, "rowKernel"),
        fields::Field(&T::sum, "sum"));
};


template<typename T, size_t order>
Eigen::VectorX<T> Sample(T sigma, size_t size)
{
    using Vector = Eigen::VectorX<T>;

    T count = static_cast<T>(size);
    auto range = (count - 1) / 2;

    Vector x = Vector::LinSpaced(
        static_cast<Eigen::Index>(size),
        -range,
        range);

    Vector exponential = (x.array().pow(2.0) / (-2 * sigma * sigma)).exp();

    if constexpr (order == 0)
    {
        T divisor = sigma * std::sqrt(static_cast<T>(2.0) * tau::Angles<T>::pi);
        return exponential.array() / divisor;
    }
    else if constexpr (order == 1)
    {
        T divisor = sigma * sigma * sigma
            * std::sqrt(static_cast<T>(2.0) * tau::Angles<T>::pi);
        return -x.array() * exponential.array() / divisor;
    }
    else
    {
        static_assert(order < 2);
    }
}


template<typename T, size_t order, typename Enable = void>
struct GaussianKernel
{

};


template<typename T, size_t order>
struct GaussianKernel<T, order, std::enable_if_t<std::is_floating_point_v<T>>>
{
    using ColumnVector = Eigen::VectorX<T>;
    using RowVector = Eigen::RowVectorX<T>;
    using Matrix = Eigen::MatrixX<T>;

    static T GetRadius(T sigma, T threshold)
    {
        // Using the zeroth gaussian to estimate kernel size.
        T scale = sigma * std::sqrt(static_cast<T>(2.0) * tau::Angles<T>::pi);
        T centerValue = static_cast<T>(1.0) / scale;
        T edgeValue = centerValue * threshold;

        return std::sqrt(
            static_cast<T>(-2.0) * sigma * sigma * std::log(scale * edgeValue));
    }

    GaussianKernel(T sigma_, T threshold_ = 0.01)
        :
        sigma(sigma_),
        threshold(threshold_),
        size(
            static_cast<size_t>(
                1 + 2 * std::round(GetRadius(sigma, threshold))))
    {
        this->columnKernel = Sample<T, order>(sigma, this->size);
        this->rowKernel = this->columnKernel.transpose();

        assert(this->rowKernel.rows() == 1);
        assert(this->columnKernel.cols() == 1);

        this->sum = this->columnKernel.sum();
    }

    GaussianKernel<T, 0> Normalize() const
    {
        // Some truncation occurs depending on the threshold, and the sum will
        // be close to 1.0.
        // Scale the kernel to unity gain.
        Eigen::MatrixX<T> combined = this->columnKernel * this->rowKernel;
        T correction = std::sqrt(combined.sum());
        GaussianKernel<T, 0> result(*this);
        result.columnKernel.array() /= correction;
        result.rowKernel.array() /= correction;
        result.sum = result.columnKernel.sum();

        return result;
    }

    Matrix GetMatrix() const
    {
        return this->columnKernel * this->rowKernel;
    }

    T sigma;
    T threshold;
    size_t size;
    RowVector rowKernel;
    ColumnVector columnKernel;
    T sum;

    static constexpr auto fields = GaussianFields<GaussianKernel>::fields;
};


template<typename T, size_t order>
struct GaussianKernel<T, order, std::enable_if_t<std::is_integral_v<T>>>
{
    using ColumnVector = Eigen::VectorX<T>;
    using RowVector = Eigen::RowVectorX<T>;
    using Matrix = Eigen::MatrixX<T>;

    GaussianKernel(double sigma_, T maximumInput, double threshold_ = 0.01)
        :
        sigma(sigma_),
        threshold(threshold_),
        size()
    {
        // Use a floating-point kernel to design our integral kernel.
        GaussianKernel<double, order> designKernel(sigma_, threshold_);
        auto normalized = designKernel.Normalize();

        // A normalized floating-point kernel sums to 1.
        // Find the maximum scale
        double maximumScale = std::floor(
            static_cast<double>(
                std::numeric_limits<T>::max() / maximumInput));

        Eigen::Index startingIndex = 0;
        double scale = 1.0 / normalized.columnKernel(startingIndex);

        Eigen::Index midpoint = (normalized.size - 1) / 2;

        while (scale > maximumScale && startingIndex < midpoint)
        {
            scale = 1.0 / normalized.columnKernel(++startingIndex);
        }

        if (startingIndex >= midpoint)
        {
            throw TauError("Unable to create integral filter");
        }

        Eigen::Index taps = (midpoint - startingIndex) * 2 + 1;

        this->columnKernel =
            (normalized.columnKernel.array() * scale).round()
                .segment(startingIndex, taps).template cast<T>();

        this->rowKernel = this->columnKernel.transpose();

        assert(this->rowKernel.rows() == 1);
        assert(this->columnKernel.cols() == 1);

        this->size = static_cast<size_t>(taps);

        this->threshold =
            1.0 / static_cast<double>(this->columnKernel.maxCoeff());

        this->sum = this->columnKernel.sum();

        // Assert that data will not be lost to overflow.
        assert(
            std::numeric_limits<T>::max() / this->sum >= maximumInput);
    }

    Matrix GetMatrix() const
    {
        return this->columnKernel * this->rowKernel;
    }

    double sigma;
    double threshold;
    size_t size;
    RowVector rowKernel;
    ColumnVector columnKernel;
    T sum;

    static constexpr auto fields = GaussianFields<GaussianKernel>::fields;
};


template<typename Derived, typename T>
Derived GaussianBlur(
    const GaussianKernel<T, 0> &kernel,
    const Eigen::MatrixBase<Derived> &data)
{
    assert(kernel.rowKernel.rows() == 1);
    assert(kernel.columnKernel.cols() == 1);

    Derived partial = tau::DoConvolve(data, kernel.rowKernel);

    if constexpr (std::is_integral_v<T>)
    {
        partial.array() /= kernel.sum;
    }

    Derived result = tau::DoConvolve(partial, kernel.columnKernel);

    if constexpr (std::is_integral_v<T>)
    {
        result.array() /= kernel.sum;
    }

    return result;
}


} // end namespace tau
