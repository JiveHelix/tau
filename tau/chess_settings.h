#pragma once


#include <jive/range.h>
#include <fields/fields.h>
#include <pex/interface.h>
#include <pex/linked_ranges.h>


namespace tau
{


template<typename T>
struct ChessFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::minimumPointsPerLine, "minimumPointsPerLine"),
        fields::Field(&T::maximumPointError, "maximumPointError"),
        fields::Field(&T::angleToleranceDegrees, "angleToleranceDegrees"),
        fields::Field(&T::lineSeparation, "lineSeparation"),
        fields::Field(&T::enableGroup, "enableGroup"),
        fields::Field(&T::groupSeparationDegrees, "groupSeparationDegrees"),
        fields::Field(&T::minimumLinesPerGroup, "minimumLinesPerGroup"),
        fields::Field(&T::rowCount, "rowCount"),
        fields::Field(&T::columnCount, "columnCount"),
        fields::Field(&T::angleFilter, "angleFilter"));
};



template<typename Float>
struct ChessTemplate
{
    static_assert(std::is_floating_point_v<Float>);

    using PointsLow = pex::Limit<3>;
    using PointsHigh = pex::Limit<32>;

    using AngleFilterLow = pex::Limit<0>;
    using AngleFilterHigh = pex::Limit<180>;

    using AngleFilterRanges =
        pex::LinkedRanges
        <
            float,
            AngleFilterLow,
            AngleFilterLow,
            AngleFilterHigh,
            AngleFilterHigh
        >;

    template<template<typename> typename T>
    struct Template
    {
        T<pex::MakeRange<size_t, PointsLow, PointsHigh>> minimumPointsPerLine;
        T<Float> maximumPointError;
        T<Float> angleToleranceDegrees;
        T<Float> lineSeparation;
        T<bool> enableGroup;
        T<Float> groupSeparationDegrees;
        T<size_t> minimumLinesPerGroup;
        T<size_t> rowCount;
        T<size_t> columnCount;
        T<AngleFilterRanges::GroupMaker> angleFilter;

        static constexpr auto fields = ChessFields<Template>::fields;
        static constexpr auto fieldsTypeName = "Chess";
    };
};


template<typename Float>
struct ChessSettings
    :
    public ChessTemplate<Float>::template Template<pex::Identity>
{
    static ChessSettings Default()
    {
        static constexpr size_t defaultMinimumPointsPerLine = 5;

        static constexpr Float defaultMaximumPointError =
            static_cast<Float>(2.0);

        static constexpr Float defaultAngleTolerance = 2;
        static constexpr Float defaultLineSeparation = 2;
        static constexpr size_t defaultMinimumLinesPerGroup = 3;
        static constexpr size_t defaultRowCount = 6;
        static constexpr size_t defaultColumnCount = 8;

        return {{
            defaultMinimumPointsPerLine,
            defaultMaximumPointError,
            defaultAngleTolerance,
            defaultLineSeparation,
            true,
            20,
            defaultMinimumLinesPerGroup,
            defaultRowCount,
            defaultColumnCount,
            {{0, 180}}}};
    }
};


} // end namespace tau
