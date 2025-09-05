#pragma once


#include "tau/eigen_shim.h"
#include "tau/angles.h"
#include "tau/percentile.h"


namespace tau
{


template<typename T, typename Lines>
static T GetAverageAngleRadians(const Lines &lines)
{
    auto lineCount = static_cast<Eigen::Index>(lines.size());

    if (!lineCount)
    {
        throw std::runtime_error("lines must not be empty");
    }

    Eigen::VectorX<T> angles(lineCount);

    for (Eigen::Index i = 0; i < lineCount; ++i)
    {
        angles(i) = lines[static_cast<size_t>(i)].GetAngleRadians();
    }

    // Lines with angles ±180 degrees are the same line.
    static constexpr T pi = tau::Angles<T>::pi;
    static constexpr T quarterTurn = pi / 2;
    angles.array() += pi;
    Eigen::VectorX<T> shiftedAngles = tau::Modulo(angles, pi);

    assert(angles.size() == lineCount);

    T angle;

    // All points are expected to be close to a line.
    // If that line is close to 0/180, averaging 179 with 1 will not yield
    // the expected result.
    EIGEN_SHIM_PUSH_IGNORES
    T first = shiftedAngles(0);
    EIGEN_SHIM_POP_IGNORES

    if (first < (pi / 4) || first > (3 * pi / 4))
    {
        // The first angle is within ±45 degrees of 0/180.
        // Shift the values by 90 degrees.
        shiftedAngles.array() += quarterTurn;

        Eigen::VectorX<T> reshifted = tau::Modulo(shiftedAngles, pi);

        T reshiftedAngle = reshifted.sum() / static_cast<T>(lineCount);

        // Now shift the result back to the correct value.
        angle = std::fmod(reshiftedAngle + quarterTurn, pi);
    }
    else
    {
        angle = shiftedAngles.sum() / static_cast<T>(lineCount);
    }

    return angle;
}


template<typename T>
static T LineAngleDifference(T firstDegrees, T secondDegrees)
{
    static constexpr T quarterTurn = 90;

    if ((firstDegrees < 45 || firstDegrees > 135)
        || (secondDegrees < 45 || secondDegrees > 135))
    {
        // At least one of the angles is within ±45 degrees of 0/180.
        // Shift the values by 90 degrees.
        firstDegrees = static_cast<T>(
            std::fmod(firstDegrees + quarterTurn, 180));

        secondDegrees = static_cast<T>(
            std::fmod(secondDegrees + quarterTurn, 180));
    }

    return firstDegrees - secondDegrees;
}


template<typename T>
static bool CompareLineAngles(
    T firstDegrees,
    T secondDegrees,
    T toleranceDegrees)
{
    auto angleError =
        std::abs(LineAngleDifference(firstDegrees, secondDegrees));

    if (angleError > toleranceDegrees)
    {
        return false;
    }

    return true;
}


template<typename Scalar>
struct AngularQuartiles
{
    Scalar lower;
    Scalar median;
    Scalar upper;

    Scalar Range() const
    {
        return LineAngleDifference(this->upper, this->lower);
    }

    Scalar ComputeLowerLimit(Scalar scale, Scalar range) const
    {
        return std::fmod(
            LineAngleDifference(this->median, range * scale) + 180.0,
            180.0);
    }

    Scalar ComputeUpperLimit(Scalar scale, Scalar range) const
    {
        return std::fmod(this->median + range * scale, 180.0);
    }

    Scalar LowerLimit(Scalar scale = 1.5) const
    {
        return this->ComputeLowerLimit(scale, this->Range());
    }

    Scalar UpperLimit(Scalar scale = 1.5) const
    {
        return this->ComputeUpperLimit(scale, this->Range());
    }
};


template<typename Scalar>
std::ostream & operator<<(
    std::ostream &outputStream,
    const AngularQuartiles<Scalar> &quartiles)
{
    return outputStream << "AngularQuartiles{"
        << quartiles.lower << ", "
        << quartiles.median << ", "
        << quartiles.upper << ", "
        << "range: " << quartiles.Range() << "}" << std::endl;
}


template<typename Derived>
AngularQuartiles<typename Derived::Scalar> GetAngularQuartiles(
    const Eigen::DenseBase<Derived> &values)
{
    using Scalar = typename Derived::Scalar;

    Derived sortedValues = values.derived();

    std::sort(
        std::begin(sortedValues),
        std::end(sortedValues),
        [](Scalar first, Scalar second)
        {
            return LineAngleDifference(first, second) < 0.0;
        });

    auto quartiles = PresortedPercentile(
        sortedValues,
        Eigen::Vector3<double>(0.25, 0.5, 0.75));

    return {quartiles(0), quartiles(1), quartiles(2)};
}


} // end namespace tau
