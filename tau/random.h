#pragma once

#include <limits>
#include <random>
#include <jive/range.h>

#include "tau/eigen.h"


namespace tau
{


using Seed = unsigned int;
using SeedLimits = std::numeric_limits<Seed>;


template<typename T, typename Enable = void>
struct Distribution_
{
    using Type = std::uniform_real_distribution<T>;
};

template<typename T>
struct Distribution_
<
    T,
    std::enable_if_t<std::is_integral_v<T>>
>
{
    using Type = std::uniform_int_distribution<T>;
};


template<typename T>
using Distribution = typename Distribution_<T>::Type;


template<typename T>
struct DefaultRange
{
    static constexpr double defaultLow = -1000.0;
    static constexpr double defaultHigh = 1000.0;

    static constexpr T low = static_cast<T>(
        std::max(
            static_cast<double>(std::numeric_limits<T>::lowest()),
            defaultLow));

    static constexpr T high = static_cast<T>(
        std::min(
            static_cast<double>(std::numeric_limits<T>::max()),
            defaultHigh));
};


template<typename Scalar_, typename Generator = std::mt19937>
struct UniformRandom
{
public:
    using Scalar = Scalar_;
    using DefaultRange = DefaultRange<Scalar>;

    UniformRandom(Seed seed)
        :
        generator_(seed),
        distribution_(
            static_cast<Scalar>(DefaultRange::low),
            static_cast<Scalar>(DefaultRange::high))
    {

    }

    UniformRandom(Seed seed, Scalar low, Scalar high)
        :
        generator_(seed),
        distribution_(low, high)
    {

    }

    void SetRange(Scalar low, Scalar high)
    {
        if (low >= high)
        {
            throw std::logic_error("low must be less than high");
        }

        this->distribution_ = Distribution<Scalar>(low, high);
    }

    void SetLow(Scalar low)
    {
        this->SetRange(low, this->distribution_.max());
    }

    Scalar GetLow() const
    {
        return this->distribution_.min();
    }

    void SetHigh(Scalar high)
    {
        this->SetRange(this->distribution_.min(), high);
    }

    Scalar GetHigh() const
    {
        return this->distribution_.max();
    }

    Scalar operator()()
    {
        return this->distribution_(this->generator_);
    }

    template<typename Matrix>
    void operator()(Matrix &matrix)
    {
        static_assert(
            std::is_same_v<Scalar, typename tau::MatrixTraits<Matrix>::type>);

        for (auto i: jive::Range<Eigen::Index>(0, matrix.rows()))
        {
            for (auto j: jive::Range<Eigen::Index>(0, matrix.cols()))
            {
                matrix(i, j) = this->operator()();
            }
        }
    }

private:
    Generator generator_;
    Distribution<Scalar> distribution_;
};


template<typename UniformRandom>
class RestoreDistribution
{
public:
    RestoreDistribution(UniformRandom &uniformRandom)
        :
        uniformRandom_(uniformRandom),
        low_(uniformRandom.GetLow()),
        high_(uniformRandom.GetHigh())
    {

    }

    ~RestoreDistribution()
    {
        this->uniformRandom_.SetRange(this->low_, this->high_);
    }

private:
    UniformRandom & uniformRandom_;
    typename UniformRandom::Scalar low_;
    typename UniformRandom::Scalar high_;
};



} // end namespace tau
