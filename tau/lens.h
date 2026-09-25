#pragma once


#include <fields/fields.h>
#include <pex/group.h>
#include <pex/identity.h>
#include "tau/arithmetic.h"


namespace tau
{


template<typename T>
struct LensSchema
{
    template<template<typename> typename U>
    struct Schema
    {
        U<T> focusDistance_m;
        U<T> aperture_fstop;

        static constexpr auto fieldsTypeName = "Lens";
    };
};


template<typename T>
struct Lens: public LensSchema<T>::template Schema<pex::Identity>
{
    using Base = typename LensSchema<T>::template Schema<pex::Identity>;

    Lens()
        :
        Base{1, 1}
    {

    }

    Lens(T focusDistance_m_, T aperture_fstop_)
        :
        Base{focusDistance_m_, aperture_fstop_}
    {

    }

    template<typename U, typename Style = Round>
    Lens<U> Cast() const
    {
        return CastFields<Lens<U>, U, Style>(*this);
    }
};



template<typename T>
class CircleOfConfusion
{
public:
    CircleOfConfusion(const Lens<T> &lens, T focalLength_m)
        :
        lens_(lens),
        focalLength_m_(focalLength_m),
        factor_(this->GetFactor())
    {

    }

    T GetFactor() const
    {
        return (this->focalLength_m_ * this->focalLength_m_)
            / (this->lens_.aperture_fstop
                * (this->lens_.focusDistance_m - this->focalLength_m_));
    }

    T operator()(T objectDistance_m) const
    {
        return this->factor_
            * std::abs(objectDistance_m - this->lens_.focusDistance_m)
            / objectDistance_m;
    }

    template<typename Result, typename, typename, typename Source>
    friend Result CastFields(const Source &);

    template<typename U, typename Style = Round>
    CircleOfConfusion<U> Cast() const
    {
        return CastFields<CircleOfConfusion<U>, U, Style>(*this);
    }

private:
    CircleOfConfusion(
        const Lens<T> &lens,
        T focalLength_m,
        T factor)
        :
        lens_(lens),
        focalLength_m_(focalLength_m),
        factor_(factor)
    {

    }

    CircleOfConfusion() = default;

private:
    Lens<T> lens_;
    T focalLength_m_;
    T factor_;
};


} // end namespace tau
