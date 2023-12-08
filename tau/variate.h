/**
  * @file variate.h
  *
  * @brief Encapsulates a random variable with its standard deviation.
  *
  * @author Jive Helix (jivehelix@gmail.com)
  * @date 09 Nov 2023
  * @copyright Jive Helix
  * Licensed under the MIT license. See LICENSE file.
**/

#pragma once


#include <fields/fields.h>
#include <pex/group.h>


namespace tau
{


template<typename T>
struct VariateFields
{
    static constexpr auto fields = std::make_tuple(
        // The mean value
        fields::Field(&T::value, "value"),

        // The standard deviation
        fields::Field(&T::variance, "variance"));
};


template<typename U>
struct VariateTemplate
{
    template<template<typename> typename T>
    struct Template
    {
        T<U> value;
        T<U> variance;

        static constexpr auto fields = VariateFields<Template>::fields;
        static constexpr auto fieldsTypeName = "Variate";
    };
};



template<typename T>
struct Variate: public VariateTemplate<T>::template Template<pex::Identity>
{
    // For addition and subtraction, variances add.
    Variate & operator+=(const Variate &other)
    {
        this->variance += other.variance;
        this->value += other.value;

        return *this;
    }

    Variate & operator-=(const Variate &other)
    {
        this->variance += other.variance;
        this->value -= other.value;

        return *this;
    }

    Variate operator+(const Variate &other)
    {
        Variate result = *this;
        result += other;

        return result;
    }

    Variate operator-(const Variate &other)
    {
        Variate result = *this;
        result -= other;

        return result;
    }


    /*

    Using Goodman's expression for f = AB, where random variables A and B have
    no covariance:

    V_f^2 = A^2 * V_B + B^2 * V_A + V_A * V_B

          2         2
    V  = A  ⋅ V  + B  ⋅ V  + V  ⋅ V
     f         B         A    A    B

    https://www.cs.cmu.edu/~cga/var/2281592.pdf

    */

    Variate & operator*=(const Variate &other)
    {
        this->variance =
            this->value * this->value * other.variance
            + other.value * other.value * this->variance
            + this->variance * other.variance;

        this->value *= other.value;

        return *this;
    }

    Variate & operator/=(const Variate &other)
    {
        /*
                     b
            f = a ⋅ A

                                        2
                 ⎛         (b - 1)     ⎞
            V  = ⎜a ⋅ b ⋅ A        ⋅ s ⎟
             f   ⎝                    A⎠


                1
            f = ─
                A

            a = 1
            b = -1

                     2
                 ⎛s ⎞    V
                 ⎜ A⎟     A
            V  = ⎜──⎟  = ──
             f   ⎜ 2⎟     4
                 ⎝A ⎠    A

                A       ⎛1⎞
            f = ─ = A ⋅ ⎜─⎟
                C       ⎝C⎠

                1
            B = ─
                C

                 V
                  C
            V  = ──
             B    4
                 C

                  2         2
            V  = A  ⋅ V  + B  ⋅ V  + V  ⋅ V
             f         B         A    A    B

                      ⎛V ⎞                    ⎛V ⎞
                  2   ⎜ C⎟   ⎛ 1⎞             ⎜ C⎟
            V  = A  ⋅ ⎜──⎟ + ⎜──⎟ ⋅ V  + V  ⋅ ⎜──⎟
             f        ⎜ 4⎟   ⎜ 2⎟    A    A   ⎜ 4⎟
                      ⎝C ⎠   ⎝C ⎠             ⎝C ⎠

            V_f = A^2 * (V_C / C^4) + V_A * (1 / C^2 + V_C / C^4)

                      ⎛V ⎞        ⎛     V ⎞
                  2   ⎜ C⎟        ⎜ 1    C⎟
            V  = A  ⋅ ⎜──⎟ + V  ⋅ ⎜── + ──⎟
             f        ⎜ 4⎟    A   ⎜ 2    4⎟
                      ⎝C ⎠        ⎝C    C ⎠


            V_f = A^2 * (V_C / C^4) + V_A * (C^2 / C^4 + V_C / C^4)

                      ⎛V ⎞        ⎛ 2   V ⎞
                  2   ⎜ C⎟        ⎜C     C⎟
            V  = A  ⋅ ⎜──⎟ + V  ⋅ ⎜── + ──⎟
             f        ⎜ 4⎟    A   ⎜ 4    4⎟
                      ⎝C ⎠        ⎝C    C ⎠

            V_f = A^2 * (V_C / C^4) + (V_A / C^4) * (C^2 + V_C)

                      ⎛V ⎞   ⎛V ⎞
                  2   ⎜ C⎟   ⎜ A⎟   ⎛ 2     ⎞
            V  = A  ⋅ ⎜──⎟ + ⎜──⎟ ⋅ ⎜C  + V ⎟
             f        ⎜ 4⎟   ⎜ 4⎟   ⎝      C⎠
                      ⎝C ⎠   ⎝C ⎠

            V_f = (A^2 * V_C + V_A * (C^2 + V_C)) / C^4

                  2             ⎛ 2     ⎞
                 A  ⋅ V  + V  ⋅ ⎜C  + V ⎟
                       C    A   ⎝      C⎠
            V  = ────────────────────────
             f               4
                            C
        */

        T otherSquared = other.value * other.value;

        this->variance =
            (
                (this->value * this->value * other.variance)
                + (this->variance * (otherSquared + other.variance)))
            / (otherSquared * otherSquared);

        this->value /= other.value;

        return *this;
    }

    Variate operator*(const Variate &other)
    {
        Variate result = *this;
        result *= other;

        return result;

    }

    Variate operator/(const Variate &other)
    {
        Variate result = *this;
        result /= other;

        return result;
    }

    Variate & operator*=(T scalar)
    {
        this->variance = scalar * scalar * this->variance;
        this->value *= scalar;

        return *this;
    }

    Variate & operator/=(T scalar)
    {
        this->variance = this->variance / (scalar * scalar);
        this->value /= scalar;

        return *this;
    }

    Variate operator*(T scalar)
    {
        Variate result = *this;
        result *= scalar;

        return result;

    }

    Variate operator/(T scalar)
    {
        Variate result = *this;
        result /= scalar;

        return result;
    }

    Variate Power(T exponent) const
    {
        Variate result{};

        result.value = std::pow(this->value, exponent);

        /*
                     b
            f = a ⋅ A

                                        2
                 ⎛         (b - 1)     ⎞
            V  = ⎜a ⋅ b ⋅ A        ⋅ s ⎟
             f   ⎝                    A⎠

                               2
                 ⎛     (b - 1)⎞
            V  = ⎝b ⋅ A       ⎠  ⋅ V
             f                      A


                  2    ((b - 1) * 2)
            V  = b  ⋅ A              ⋅ V
             f                          A

                  2    (2 ⋅ (b - 1))
            V  = b  ⋅ A              ⋅ V
             f                          A


        */

        result.variance =
            (exponent * exponent)
            * std::pow(this->value, 2 * (exponent - 1))
            * this->variance;

        return result;
    }
};


TEMPLATE_OUTPUT_STREAM(Variate)


template<typename T>
using VariateGroup =
    pex::Group
    <
        VariateFields,
        VariateTemplate<T>::template Template,
        Variate<T>
    >;


template<typename T>
using VariteModel = typename VariateGroup<T>::Model;

template<typename T>
using VariteControl = typename VariateGroup<T>::Control;

template<typename T>
using VariateGroupMaker = pex::MakeGroup<VariateGroup<T>>;


} // end namespace tau
