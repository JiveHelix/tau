#pragma once


#include <fields/fields.h>
#include <pex/group.h>
#include <pex/identity.h>


namespace tau
{


template<typename T>
struct LensFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::focusDistance_m, "focusDistance (m)"),
        fields::Field(&T::aperture_fstop, "aperture (f/stop)"));
};


template<typename T>
struct LensTemplate
{
    template<template<typename> typename U>
    struct Template
    {
        U<T> focusDistance_m;
        U<T> aperture_fstop;

        static constexpr auto fields = LensFields<Template>::fields;
        static constexpr auto fieldsTypeName = "Lens";
    };
};


template<typename T>
struct Lens: public LensTemplate<T>::template Template<pex::Identity>
{
    using Base = typename LensTemplate<T>::template Template<pex::Identity>;
    Lens()
        :
        Base{1, 1}
    {

    }

    Lens(T focusDistance_m_, T aperature_fstop_)
        :
        Base{focusDistance_m_, aperature_fstop_}
    {

    }

    static Lens Default()
    {
        return {};
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

private:
    Lens<T> lens_;
    T focalLength_m_;
    T factor_;
};


} // end namespace tau
