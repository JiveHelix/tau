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
    std::enable_if_t<std::is_integral_v<T> && (sizeof(T) > 1)>
>
{
    using Type = std::uniform_int_distribution<T>;
};


template<typename T, typename Enable = void>
struct DistributionType_ {};

template<typename T>
struct DistributionType_<T, std::enable_if_t<std::is_signed_v<T>>>
{
    using Type = int16_t;
};

template<typename T>
struct DistributionType_<T, std::enable_if_t<std::is_unsigned_v<T>>>
{
    using Type = uint16_t;
};


template<typename T>
using DistributionType = typename DistributionType_<T>::Type;


template<typename T>
struct Distribution_
<
    T,
    std::enable_if_t<std::is_integral_v<T> && (sizeof(T) == 1)>
>
{
    using Type = std::uniform_int_distribution<DistributionType<T>>;
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


template<typename T>
struct DefaultFilter
{
    bool operator()(T) const
    {
        return true;
    }
};


template
<
    typename Scalar_,
    template<typename> typename Filter = DefaultFilter,
    typename Generator = std::mt19937
>
struct UniformRandom
{
public:
    using Scalar = Scalar_;

    UniformRandom(Seed seed)
        :
        generator_(seed),
        distribution_(
            static_cast<Scalar>(DefaultRange<Scalar>::low),
            static_cast<Scalar>(DefaultRange<Scalar>::high))
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
        assert(
            this->distribution_.max()
            <= std::numeric_limits<Scalar>::max());

        this->SetRange(low, static_cast<Scalar>(this->distribution_.max()));
    }

    Scalar GetLow() const
    {
        return static_cast<Scalar>(this->distribution_.min());
    }

    void SetHigh(Scalar high)
    {
        assert(
            this->distribution_.min()
            >= std::numeric_limits<Scalar>::lowest());

        this->SetRange(static_cast<Scalar>(this->distribution_.min()), high);
    }

    Scalar GetHigh() const
    {
        return static_cast<Scalar>(this->distribution_.max());
    }

    Scalar operator()()
    {
        assert(
            this->distribution_.min()
            >= std::numeric_limits<Scalar>::lowest());

        assert(
            this->distribution_.max()
            <= std::numeric_limits<Scalar>::max());

        auto result =
            static_cast<Scalar>(this->distribution_(this->generator_));

        while (!Filter<Scalar>{}(result))
        {
            result =
                static_cast<Scalar>(this->distribution_(this->generator_));
        }

        return result;
    }

    template<typename Matrix>
    void operator()(Matrix &matrix)
    {
        assert(
            this->distribution_.max()
            <= std::numeric_limits<Scalar>::max());

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


template
<
    typename Scalar_,
    typename Generator = std::mt19937
>
struct Normal
{
public:
    using Scalar = Scalar_;

public:
    Normal()
        :
        distribution_()
    {

    }

    Normal(Scalar mean, Scalar stddev)
        :
        distribution_(mean, stddev)
    {

    }

    Scalar operator()()
    {
        return static_cast<Scalar>(this->distribution_(this->generator_));
    }

    template<typename Matrix>
    void operator()(Matrix &matrix)
    {
        for (auto i: jive::Range<Eigen::Index>(0, matrix.rows()))
        {
            for (auto j: jive::Range<Eigen::Index>(0, matrix.cols()))
            {
                matrix(i, j) = this->operator()();
            }
        }
    }

private:
    // Make global generator thread_local.
    // Each thread get its own, simplifying thread-safe operation without
    // depleting system entropy.
    static thread_local Generator generator_;
    std::normal_distribution<Scalar> distribution_;
};


template<typename Scalar, typename Generator>
thread_local Generator Normal<Scalar, Generator>::generator_{
    std::random_device{}()};


} // end namespace tau
