/**
  * @file power_series.h
  * 
  * @brief Compute values of the power series from coefficients.
  * 
  * @author Jive Helix (jivehelix@gmail.com)
  * @date 09 Nov 2021
  * @copyright Jive Helix
  * Licensed under the MIT license. See LICENSE file.
**/

#pragma once


#include "jive/type_traits.h"
#include "tau/eigen.h"


namespace tau
{

template<typename T>
struct Domain
{
    T first;
    T last;

    Domain(T first_, T last_): first(first_), last(last_) {}

    T GetLength() const
    {
        return this->last - this->first;
    }
};


template<typename T>
class LinearMap
{
    static_assert(
        std::is_floating_point_v<T>,
        "Only supports floating-point types.");

public:
    LinearMap()
        : offset_(0),
        scale_(1)
    {

    }

    LinearMap(const Domain<T> &source, const Domain<T> &target)
    {
        this->offset_ = 
            (source.last * target.first - source.first * target.last)
            / source.GetLength();

        this->scale_ = target.GetLength() / source.GetLength();
    }

    template<typename U>
    auto operator()(U value) const
    {
        return value * this->scale_ + this->offset_;
#if 0
        value.array() *= this->scale_;
        value.array() += this->offset_;
#endif
    }

private:
    T offset_;
    T scale_;
};


template<typename F, typename Enable = void>
struct IndependentReplication;


template<typename F>
struct IndependentReplication<
    F,
    std::enable_if_t<MatrixTraits<F>::isRowVector>
>
{
    // Replicate as columns
    static constexpr int rowFactor = 1;
    static constexpr int columnFactor = MatrixTraits<F>::size;
};


template<typename F>
struct IndependentReplication<
    F,
    std::enable_if_t<MatrixTraits<F>::isColumnVector>
>
{
    // Replicate as rows
    static constexpr int rowFactor = MatrixTraits<F>::size;
    static constexpr int columnFactor = 1;
};


template<typename I, typename F, typename Enable = void>
struct PowersReplication;


template<typename I, typename F>
struct PowersReplication<
    I,
    F,
    std::enable_if_t<
        !MatrixTraits<I>::isDynamic
        && MatrixTraits<F>::isRowVector>
>
{
    // Replicate as columns
    static constexpr int rowFactor = MatrixTraits<I>::size;
    static constexpr int columnFactor = 1;
};


template<typename I, typename F>
struct PowersReplication<
    I,
    F,
    std::enable_if_t<
        !MatrixTraits<I>::isDynamic
        && MatrixTraits<F>::isColumnVector>
>
{
    // Replicate as rows
    static constexpr int rowFactor = 1;
    static constexpr int columnFactor = MatrixTraits<I>::size;
};





template<typename I, typename F, typename Enable = void>
struct Expansion;


template<typename I, typename F>
struct Expansion<
    I,
    F,
    std::enable_if_t<
        MatrixTraits<I>::isDynamic
        && MatrixTraits<F>::isRowVector>
>
{
    using iTraits = MatrixTraits<I>;

    using type = typename Eigen::Map<
        const Eigen::Vector<typename iTraits::type, Eigen::Dynamic>>;
};


template<typename I, typename F>
struct Expansion<
    I,
    F,
    std::enable_if_t<
        MatrixTraits<I>::isDynamic
        && MatrixTraits<F>::isColumnVector>
>
{
    using iTraits = MatrixTraits<I>;

    using type = typename Eigen::Map<
        const Eigen::RowVector<typename iTraits::type, Eigen::Dynamic>>;
};
            

template<typename I, typename F>
struct Expansion<
    I,
    F,
    std::enable_if_t<
        !MatrixTraits<I>::isDynamic
        && MatrixTraits<F>::isRowVector>
>
{
    using iTraits = MatrixTraits<I>;

    using type = typename Eigen::Map<
        const Eigen::Vector<typename iTraits::type, iTraits::size>>;
};


template<typename I, typename F>
struct Expansion<
    I,
    F,
    std::enable_if_t<
        !MatrixTraits<I>::isDynamic
        && MatrixTraits<F>::isColumnVector>
>
{
    using iTraits = MatrixTraits<I>;

    using type = typename Eigen::Map<
        const Eigen::RowVector<typename iTraits::type, iTraits::size>>;
};


template<typename I, typename F>
auto ExpandIndependent(I &&independent, const F &factors)
{
    static_assert(IsMatrix<F>::value);

    using iType = std::remove_cvref_t<I>;
    using fTraits = MatrixTraits<F>;

    static_assert(fTraits::isVector, "factors must be a row or column vector");

    static_assert(
        std::is_arithmetic_v<iType> || jive::IsValueContainer<iType>::value,
        "Unexpected independent type.");

    if constexpr (jive::IsValueContainer<iType>::value)
    {
        assert(
            static_cast<size_t>(independent.size())
                <= static_cast<size_t>(
                    std::numeric_limits<Eigen::Index>::max()));
    }

    if constexpr (fTraits::isDynamic)
    {
        Eigen::Index rowFactor;
        Eigen::Index columnFactor;

        if constexpr (fTraits::isRowVector)
        {
            rowFactor = 1;
            columnFactor = factors.innerSize();
        }
        else 
        {
            static_assert(fTraits::isColumnVector);
            rowFactor = factors.innerSize();
            columnFactor = 1;
        }

        if constexpr (std::is_arithmetic_v<iType>)
        {
            // Expand the independent variable to a row or column vector.
            using Expand = MatrixLike<iType, F>;

            return Expand::Constant(
                rowFactor,
                columnFactor,
                independent);
        }
        else 
        {
            static_assert(jive::IsValueContainer<iType>::value);

            using MapType = typename Expansion<I, F>::type;

            MapType flattened(
                independent.data(),
                static_cast<Eigen::Index>(independent.size()));

            if constexpr (std::is_rvalue_reference_v<I>)
            {
                // Return by copy.
                return flattened.replicate(rowFactor, columnFactor).eval();
            }
            else
            {
                return flattened.replicate(rowFactor, columnFactor);
            }
        }
    }
    else
    {
        using replication = IndependentReplication<F>;

        if constexpr (std::is_arithmetic_v<iType>)
        {
            using Expand = MatrixLike<iType, F>;
            return Expand::Constant(independent);
        }
        else
        {
            static_assert(jive::IsValueContainer<iType>::value);

            using MapType = typename Expansion<I, F>::type;

            MapType flattened(
                independent.data(),
                static_cast<Eigen::Index>(independent.size()));

            if constexpr (std::is_rvalue_reference_v<I>)
            {
                // Return by copy.
                return flattened.template replicate<
                    replication::rowFactor,
                    replication::columnFactor>().eval();
            }
            else
            {
                return flattened.template replicate<
                    replication::rowFactor,
                    replication::columnFactor>();
            }
        }
    }
}


template<typename F>
auto CreatePowers(const F &factors)
{
    using traits = MatrixTraits<F>;
    using type = MatrixLike<typename MatrixTraits<F>::type, F>;

    static_assert(
        IsMatrix<type>::value,
        "Deduced type must be an Eigen Matrix.");

    if constexpr (traits::isDynamic)
    {
        assert(factors.outerSize() == 1);
        
        if (IsRowVector(factors))
        {
            type result(1, factors.innerSize());
            auto flattened = result.reshaped();
            std::iota(flattened.begin(), flattened.end(), 0);
            return result;
        }
        else
        {
            type result(factors.innerSize(), 1);
            auto flattened = result.reshaped();
            std::iota(flattened.begin(), flattened.end(), 0);
            return result;
        }
    }
    else
    {
        type result{};
        std::iota(result.begin(), result.end(), 0);
        return result;
    }
}


template<typename F, typename P, typename I>
auto ExpandPowers(P &&powers, const I &independent)
{
    static_assert(IsMatrix<F>::value);

    using iTraits = MatrixTraits<I>;
    using fTraits = MatrixTraits<F>;

    static_assert(fTraits::isVector, "factors must be a row or column vector");

    static_assert(
        std::is_arithmetic_v<I> || jive::IsValueContainer<I>::value,
        "Unexpected independent type.");

    if constexpr (jive::IsValueContainer<I>::value)
    {
        assert(
            static_cast<size_t>(independent.size())
                <= static_cast<size_t>(
                    std::numeric_limits<Eigen::Index>::max()));
    }
   
    if constexpr (std::is_arithmetic_v<I>)
    {
        return powers;
    }
    else if constexpr (!iTraits::isDynamic)
    {
        using replication = PowersReplication<I, F>;

        return powers.template replicate<
            replication::rowFactor,
            replication::columnFactor>();
    }
    else
    {
        Eigen::Index rowFactor;
        Eigen::Index columnFactor;

        if constexpr (fTraits::isRowVector)
        {
            rowFactor = static_cast<Eigen::Index>(independent.size());
            columnFactor = 1;
        }
        else 
        {
            static_assert(fTraits::isColumnVector);
            rowFactor = 1;
            columnFactor = static_cast<Eigen::Index>(independent.size());
        }

        return powers.replicate(rowFactor, columnFactor);
    }
}


template<typename F, typename Enable = void>
struct Sum;

template<typename F>
struct Sum<
    F,
    std::enable_if_t<MatrixTraits<F>::isRowVector>
>
{
    template<typename Terms>
    auto operator()(const Terms &terms)
    {
        return terms.array().rowwise().sum(); 
    }
};

template<typename F>
struct Sum<
    F,
    std::enable_if_t<MatrixTraits<F>::isColumnVector>
>
{
    template<typename Terms>
    auto operator()(const Terms &terms)
    {
        return terms.array().colwise().sum(); 
    }
};


template<typename I, typename F>
auto PowerSeries(
    const I &independent,
    const F &factors,
    const LinearMap<typename MatrixTraits<I>::type> &linearMap)
{
    static_assert(
        MatrixTraits<F>::isVector,
        "Factors of the polynomial must be either a row or column vector.");

    auto expandedIndependent = ExpandIndependent(independent, factors);

    auto powers = CreatePowers(factors);

    auto expandedPowers = ExpandPowers<F>(powers, independent);

    auto terms =
        linearMap(
            expandedIndependent.array()).pow(expandedPowers.array()).eval();

    using fTraits = MatrixTraits<F>;

    if constexpr (fTraits::isColumnVector)
    {
        // Coefficients is a column vector.
        terms.array().colwise() *= factors.array();
    }
    else if constexpr (fTraits::isRowVector)
    {
        terms.array().rowwise() *= factors.array();
    }
    else
    {
        static_assert(fTraits::isDynamic, "Factors must be column or row");

        // Runtime decision about transposition.
        if (IsColumnVector(factors))
        {
            // factors is a column vector.
            terms.array().colwise() *= factors(Eigen::all, 0).array();
        }
        else
        {
            // factors is a row vector.
            terms.array().rowwise() *= factors(0, Eigen::all).array();
        }
    }

    auto sum = Sum<F>{}(terms).eval();

    using iTraits = MatrixTraits<I>;

    if constexpr (iTraits::isDynamic && iTraits::isMatrix)
    {
        // Dynamic matrix or map.
        using type =
            typename Eigen::Map<Eigen::MatrixX<typename iTraits::type>>;
        
        assert(sum.size() == independent.rows() * independent.cols());

        return type(
            sum.data(),
            independent.rows(),
            independent.cols()).eval();

    }
    else if constexpr (iTraits::isMatrix && !iTraits::isDynamic)
    {
        // Static matrix or map.
        using type = typename Eigen::Map<
            Eigen::Matrix<
                typename iTraits::type,
                iTraits::rows,
                iTraits::columns>>;

        return type(sum.data()).eval();
    }
    else if constexpr (iTraits::isDynamic && !IsMatrix<iTraits>::value)
    {
        // Input is a value container. Return a column vector by default.
        using type = 
            typename Eigen::Map<Eigen::VectorX<typename iTraits::type>>;

        return type(sum.data(), sum.size()).eval();
    }
    else
    {
        static_assert(std::is_arithmetic_v<std::remove_cvref_t<I>>);
        return sum(0);
    }
}


} // end namespace tau
