#pragma once


#include <fields/fields.h>
#include <fields/compare.h>
#include <pex/endpoint.h>
#include <pex/linked_ranges.h>


namespace tau
{


namespace colors
{


struct SignedGradient
{
    static constexpr auto name = "Signed Gradient";
};


struct Turbo
{
    static constexpr auto name = "Turbo";
};


struct Grayscale
{
    static constexpr auto name = "Gray";
};


} // end namespace colors


using DefaultLowColor = pex::Limit<0>;
using DefaultHighColor = pex::Limit<255>;

template<typename Value>
using ColorRange =
    pex::LinkedRanges
    <
        Value,
        DefaultLowColor,
        DefaultLowColor,
        DefaultHighColor,
        DefaultHighColor
    >;


template<typename Value>
struct ColorMapSettingsSchema
{
    template<template<typename> typename T>
    struct Schema
    {
        T<bool> turbo;
        T<typename ColorRange<Value>::Group> range;
        T<Value> maximum;

        static constexpr auto fieldsTypeName = "Color";
    };
};


template<typename Value>
struct ColorMapSettings:
    public ColorMapSettingsSchema<Value>::template Schema<pex::Identity>
{
    ColorMapSettings()
        :
        ColorMapSettingsSchema<Value>::template Schema<pex::Identity>{
            true,
            typename ColorRange<Value>::Settings{},
            DefaultHighColor::Get<Value>()}
    {

    }
};


template<typename Value>
struct ColorMapSettingsFinisher
{
    using Plain = ColorMapSettings<Value>;

    template<typename Base>
    struct Model: public Base
    {
    public:
        using Base::operator=;

        Model()
            :
            Base(),

            maximumEndpoint_(
                this,
                this->maximum,
                &Model::OnMaximum_)
        {

        }

    private:
        void OnMaximum_(Value maximum_)
        {
            this->range.SetMaximumValue(maximum_);
        }

    private:
        using MaximumEndpoint = pex::Endpoint<Model, decltype(Model::maximum)>;
        MaximumEndpoint maximumEndpoint_;
    };
};


TEMPLATE_EQUALITY_OPERATORS(ColorMapSettings)
TEMPLATE_OUTPUT_STREAM(ColorMapSettings)


template<typename Value>
using ColorMapSettingsGroup =
    pex::Group
    <
        ColorMapSettingsSchema<Value>::template Schema,
        ColorMapSettingsFinisher<Value>
    >;

template<typename Value>
using ColorMapSettingsModel = typename ColorMapSettingsGroup<Value>::Model;

template<typename Value>
using ColorMapSettingsControl =
    typename ColorMapSettingsGroup<Value>::DefaultControl;


} // end namespace tau


extern template struct pex::Group
    <
        tau::ColorMapSettingsSchema<int32_t>::template Schema,
        tau::ColorMapSettingsFinisher<int32_t>
    >;
