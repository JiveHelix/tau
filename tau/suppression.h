#pragma once


#include <fields/fields.h>
#include <pex/interface.h>
#include <pex/selectors.h>
#include <pex/range.h>
#include <tau/eigen.h>
#include <tau/image.h>


namespace tau
{


template<typename T>
struct SuppressionFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::window, "window"),
        fields::Field(&T::count, "count"));
};


struct SuppressionRanges
{
    using WindowLow = pex::Limit<2>;
    using WindowHigh = pex::Limit<10>;

    using CountLow = pex::Limit<1>;
    using CountHigh = pex::Limit<10>;
};


template<typename Ranges = SuppressionRanges>
struct SuppressionTemplate
{
    using WindowLow = typename Ranges::WindowLow;
    using WindowHigh = typename Ranges::WindowHigh;

    using CountLow = typename Ranges::CountLow;
    using CountHigh = typename Ranges::CountHigh;

    template<template<typename> typename T>
    struct Template
    {
        using Index = Eigen::Index;

        T<pex::MakeRange<Eigen::Index, WindowLow, WindowHigh>> window;
        T<pex::MakeRange<Eigen::Index, CountLow, CountHigh>> count;

        static constexpr auto fields = SuppressionFields<Template>::fields;
        static constexpr auto fieldsTypeName = "Suppression";
    };
};


struct SuppressionSettings
    :
    public SuppressionTemplate<SuppressionRanges>::template Template<pex::Identity>
{
    static SuppressionSettings Default()
    {
        static constexpr Eigen::Index defaultWindow = 3;
        static constexpr Eigen::Index defaultCount = 1;

        return {{defaultWindow, defaultCount}};
    }
};


class Suppression
{
public:
    using Index = typename Eigen::Index;

public:
    Suppression(const SuppressionSettings &settings)
        :
        settings_(settings),
        windowSize_(settings.window)
    {
        assert(settings.count < settings.window * settings.window);
        assert(settings.count > 0);
    }

public:
    ImageMatrixFloat Filter(const ImageMatrixFloat &input)
    {
        ImageMatrixFloat result = input;

        Index rowCount = input.rows();
        Index columnCount = input.cols();

        Index limitRow = rowCount - this->windowSize_ + 1;
        Index limitColumn = columnCount - this->windowSize_ + 1;

        if (this->settings_.count == 1)
        {
            this->SelectOne_(limitRow, limitColumn, result);
            return result;
        }

        using Float = typename ImageMatrixFloat::Scalar;
        std::vector<Detection_<Float>> detections;

        detections.reserve(
            static_cast<size_t>(this->windowSize_ * this->windowSize_));

        if constexpr (MatrixTraits<ImageMatrixFloat>::isRowMajor)
        {
            for (Index row = 0; row < limitRow; ++row)
            {
                for (Index column = 0; column < limitColumn; ++column)
                {
                    this->FilterWindowedDetections_(
                        result,
                        detections,
                        row,
                        column);
                }
            }
        }
        else
        {
            // Iterate in column-major order
            for (Index column = 0; column < limitColumn; ++column)
            {
                for (Index row = 0; row < limitRow; ++row)
                {
                    this->FilterWindowedDetections_(
                        result,
                        detections,
                        row,
                        column);
                }
            }
        }

        return result;
    }

private:
    template<typename Float>
    struct Detection_
    {
        Index row;
        Index column;
        Float value;

        Detection_(Index row_, Index column_, Float value_)
            :
            row(row_),
            column(column_),
            value(value_)
        {

        }

        bool operator>(const Detection_<Float> &other) const
        {
            return this->value > other.value;
        }
    };

    template<typename Data>
    void SelectOne_(
        Index limitRow,
        Index limitColumn,
        Eigen::MatrixBase<Data> &result)
    {
        for (Index column = 0; column < limitColumn; ++column)
        {
            for (Index row = 0; row < limitRow; ++row)
            {
                auto block = result.block(
                    row,
                    column,
                    this->windowSize_,
                    this->windowSize_);

                auto maximum = block.maxCoeff();
                block = (block.array() < maximum).select(0, block);
            }
        }
    }


    template<typename Data>
    void MakeDetections_(
        Data &data,
        std::vector<Detection_<typename Data::Scalar>> &detections,
        Index windowRow,
        Index windowColumn)
    {
        using Scalar = typename Data::Scalar;
        detections.clear();

        auto MakeDetection =
            [&data, &detections] (
                Index detectionRow,
                Index detectionColumn) -> void
            {
                Scalar value = data(detectionRow, detectionColumn);

                if (value > 0)
                {
                    detections.emplace_back(
                        detectionRow,
                        detectionColumn,
                        value);
                }
            };

        if constexpr (MatrixTraits<Data>::isRowMajor)
        {
            for (Index row = 0; row < this->windowSize_; ++row)
            {
                Index detectionRow = windowRow + row;

                for (Index column = 0; column < this->windowSize_; ++column)
                {
                    MakeDetection(detectionRow, windowColumn + column);
                }
            }
        }
        else
        {
            // Iterate in column-major order
            for (Index column = 0; column < this->windowSize_; ++column)
            {
                Index detectionColumn = windowColumn + column;

                for (Index row = 0; row < this->windowSize_; ++row)
                {
                    MakeDetection(windowRow + row, detectionColumn);
                }
            }
        }
    }

    template<typename Data>
    void FilterWindowedDetections_(
        Data &data,
        std::vector<Detection_<typename Data::Scalar>> &detections,
        Index windowRow,
        Index windowColumn)
    {
        this->MakeDetections_(data, detections, windowRow, windowColumn);

        // Keep the top detections
        data.block(
            windowRow,
            windowColumn,
            this->windowSize_,
            this->windowSize_).array() = 0;

        if (detections.empty())
        {
            // None of the pixels in this window were greater than zero.
            return;
        }

        using Float = typename Data::Scalar;

        // Sort detections in descending order.
        std::sort(
            detections.begin(),
            detections.end(),
            std::greater<Detection_<Float>>());

        auto detection = detections.begin();

        Index count = std::min(
            this->settings_.count,
            static_cast<Index>(detections.size()));

        while (count-- > 0)
        {
            data(detection->row, detection->column) = detection->value;
            ++detection;
        }
    }

private:
    SuppressionSettings settings_;
    Index windowSize_;
};


} // end namespace tau
