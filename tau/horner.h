/**
  * @file horner.h
  *
  * @brief Compute values of polynomials.
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
    auto operator()(const Eigen::ArrayBase<U> &value) const
    {
        return value * this->scale_ + this->offset_;
    }

    template<typename U>
    auto operator()(const Eigen::DenseBase<U> &value) const
    {
        return value.derived().array() * this->scale_ + this->offset_;
    }

private:
    T offset_;
    T scale_;
};


template<typename T>
struct HornerResult_
{
    using Type = std::remove_cv_t<T>;
};

template<typename T>
struct HornerResult_<Eigen::Map<T>>
{
    using Type = std::remove_cv_t<T>;
};

template<typename T>
using HornerResult = typename HornerResult_<T>::Type;


template<typename I, typename F>
HornerResult<I> Horner(
    const Eigen::DenseBase<I> &independent,
    const Eigen::DenseBase<F> &factors)
{
    Eigen::Index lastIndex = factors.size() - 1;
    using Result = HornerResult<I>;
    Result result = Result(independent.rows(), independent.cols());
    result.array() = factors(lastIndex);

    for (Eigen::Index i = lastIndex - 1; i >= 0; --i)
    {
        result = independent.derived().array() * result.array() + factors(i);
    }

    return result;
}


template<typename I, typename F>
std::enable_if_t<std::is_floating_point_v<I>, I> Horner(
    I independent,
    const Eigen::DenseBase<F> &factors)
{
    Eigen::Index lastIndex = factors.size() - 1;
    I result = factors(lastIndex);

    for (Eigen::Index i = lastIndex - 1; i >= 0; --i)
    {
        result = independent * result + factors(i);
    }

    return result;
}


template<typename I, typename F>
I Horner(
    const Eigen::DenseBase<I> &independent,
    const Eigen::DenseBase<F> &factors,
    const LinearMap<typename MatrixTraits<I>::type> &linearMap)
{
    return Horner(linearMap(independent).eval(), factors);
}


template<typename I, typename F>
std::vector<I> Horner(
    const std::vector<I> &independent,
    const std::vector<F> &factors)
{
    using Independent = Eigen::Map<const Eigen::VectorX<I>>;
    using Factors = Eigen::Map<const Eigen::VectorX<F>>;

    auto result = Horner(
        Independent(independent.data(), Eigen::Index(independent.size())),
        Factors(factors.data(), Eigen::Index(factors.size())));

    auto begin = result.data();
    auto end = begin + independent.size();
    return {begin, end};
}


} // end namespace tau
