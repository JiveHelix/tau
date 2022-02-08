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

namespace tau
{



inline constexpr long double tau = 6.283185307179586476925286766559005L;
inline constexpr long double pi = tau / 2.0L;
inline constexpr long double tauDegrees = 360.0L;
inline constexpr long double degreesPerRadian = tauDegrees / tau;
inline constexpr long double radiansPerDegree = tau / tauDegrees;


template<typename T>
struct Angles
{
    static_assert(std::is_floating_point_v<T>);

    static constexpr T tau = static_cast<T>(::tau::tau);

    static constexpr T pi = static_cast<T>(::tau::pi);

    static constexpr T tauDegrees =
        static_cast<T>(::tau::tauDegrees);

    static constexpr T degreesPerRadian =
        static_cast<T>(::tau::degreesPerRadian);

    static constexpr T radiansPerDegree =
        static_cast<T>(::tau::radiansPerDegree);
};



template<typename T>
auto ToDegrees(T radians) -> T
{
    return radians * Angles<T>::degreesPerRadian;
}


template<typename T, typename ...U>
auto ToDegrees(T radians, U ...otherRadians) -> std::tuple<T, U...>
{
    return std::make_tuple(ToDegrees(radians), ToDegrees(otherRadians)...);
}


template<typename T>
auto ToRadians(T degrees) -> T
{
    return degrees * Angles<T>::radiansPerDegree;
}


template<typename T, typename ...U>
auto ToRadians(T degrees, U ...otherDegrees) -> std::tuple<T, U...>
{
    return std::make_tuple(ToRadians(degrees), ToRadians(otherDegrees)...);
}


} // end namespace tau
