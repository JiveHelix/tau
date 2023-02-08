/**
  * @file angles.h
  * 
  * @brief Tau, one full rotation. https://tauday.com/tau-manifesto
  * 
  * @author Jive Helix (jivehelix@gmail.com)
  * @date 05 May 2020
  * @copyright Jive Helix
  * Licensed under the MIT license. See LICENSE file.
**/

#pragma once

#include <tuple>
#include <cmath>
#include "tau/eigen.h"


namespace tau
{


namespace constants
{
inline constexpr long double tau = 6.283185307179586476925286766559005L;
inline constexpr long double pi = tau / 2.0L;
inline constexpr long double tauDegrees = 360.0L;
inline constexpr long double piDegrees = 180.0L;
inline constexpr long double degreesPerRadian = tauDegrees / tau;
inline constexpr long double radiansPerDegree = tau / tauDegrees;
} // end namespace constants


template<typename T>
struct Angles
{
    static_assert(std::is_floating_point_v<T>);

    static constexpr T tau = static_cast<T>(constants::tau);

    static constexpr T pi = static_cast<T>(constants::pi);

    static constexpr T tauDegrees =
        static_cast<T>(constants::tauDegrees);

    static constexpr T piDegrees =
        static_cast<T>(constants::piDegrees);

    static constexpr T degreesPerRadian =
        static_cast<T>(constants::degreesPerRadian);

    static constexpr T radiansPerDegree =
        static_cast<T>(constants::radiansPerDegree);
};


template<typename T, typename Enable = void>
struct AngleType_;

template<typename T>
struct AngleType_<T, std::enable_if_t<std::is_floating_point_v<T>>>
{
    using Type = T;
};

template<typename T>
struct AngleType_<T, std::enable_if_t<HasScalar<T>>>
{
    using Type = typename T::Scalar;
};

template<typename T>
using AngleType = typename AngleType_<T>::Type;


template<typename T>
auto ToDegrees(T radians)
{
    return radians * Angles<AngleType<T>>::degreesPerRadian;
}


template<typename T, typename ...U>
auto ToDegrees(T radians, U ...otherRadians) -> std::tuple<T, U...>
{
    return std::make_tuple(ToDegrees(radians), ToDegrees(otherRadians)...);
}


template<typename T>
auto ToRadians(T degrees)
{
    return degrees * Angles<AngleType<T>>::radiansPerDegree;
}


template<typename T, typename ...U>
auto ToRadians(T degrees, U ...otherDegrees)
{
    return std::make_tuple(ToRadians(degrees), ToRadians(otherDegrees)...);
}


} // end namespace tau
