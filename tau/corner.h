#pragma once


#include <fields/fields.h>
#include <pex/interface.h>
#include <pex/range.h>
#include <tau/eigen.h>
#include <tau/vector2d.h>


namespace tau
{


template<typename T>
struct CornerFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::window, "window"),
        fields::Field(&T::count, "count"));
};


template<template<typename> typename T>
struct CornerTemplate
{
    using WindowLow = pex::Limit<3>;
    using WindowHigh = pex::Limit<32>;

    using CountLow = pex::Limit<1>;
    using CountHigh = pex::Limit<4>;

    T<pex::MakeRange<Eigen::Index, WindowLow, WindowHigh>> window;
    T<pex::MakeRange<Eigen::Index, CountLow, CountHigh>> count;

    static constexpr auto fields =
        CornerFields<CornerTemplate>::fields;

    static constexpr auto fieldsTypeName = "Corner";
};


struct CornerSettings
    :
    public CornerTemplate<pex::Identity>
{
    static CornerSettings Default()
    {
        static constexpr Eigen::Index defaultWindow = 12;
        static constexpr Eigen::Index defaultCount = 4;

        return {{defaultWindow, defaultCount}};
    }
};


struct CornerPoint
{
    Point2d<double> point;
    double count;

    CornerPoint(double x, double y, double count_)
        :
        point(x, y),
        count(count_)
    {

    }

    bool operator>(const CornerPoint &other) const
    {
        if (this->point.template Convert<int>()
                > other.point.template Convert<int>())
        {
            return true;
        }

        return this->count > other.count;
    }

    // For the purpose of determining unique corners, corners with the same
    // point compare equal, even if their counts differ.
    bool operator==(const CornerPoint &other) const
    {
        return (this->point.template Convert<int>()
            == other.point.template Convert<int>());
    }
};


using CornerPointsCollection = std::vector<CornerPoint>;


template<typename Data>
static CornerPointsCollection GetPoints(
    const Eigen::MatrixBase<Data> &input)
{
    using Eigen::Index;

    CornerPointsCollection points;

    if constexpr (MatrixTraits<Data>::isRowMajor)
    {
        for (Index row = 0; row < input.rows(); ++row)
        {
            for (Index column = 0; column < input.cols(); ++column)
            {
                if (input(row, column) != 0)
                {
                    points.emplace_back(
                        static_cast<double>(column),
                        static_cast<double>(row),
                        1);
                }
            }
        }
    }
    else
    {
        // Iterate in column-major order.
        for (Index column = 0; column < input.cols(); ++column)
        {
            for (Index row = 0; row < input.rows(); ++row)
            {
                if (input(row, column) != 0)
                {
                    points.emplace_back(
                        static_cast<double>(column),
                        static_cast<double>(row),
                        1);
                }
            }
        }
    }

    return points;
}


namespace internal
{


class CornerCollector
{
public:
    using Index = typename Eigen::Index;

    CornerCollector(size_t windowSize, size_t count)
        :
        windowSize_(windowSize),
        count_(count),
        points_(),
        corners_()
    {
        this->points_.reserve(windowSize * windowSize);
    }

    const CornerPointsCollection & GetCorners()
    {
        return this->corners_;
    }

    template<typename Input>
    void CollectFromWindow(
        Input &input,
        Eigen::Index windowRow,
        Eigen::Index windowColumn)
    {
        using Eigen::Index;

        auto MakePoints = [this, &input] (Index row, Index column) -> void
        {
            if (input(row, column) != 0)
            {
                this->points_.emplace_back(
                    static_cast<double>(column),
                    static_cast<double>(row));
            }
        };

        // Iterate over the window at this position.
        this->points_.clear();

        auto windowSize = static_cast<Index>(this->windowSize_);

        if constexpr (MatrixTraits<Input>::isRowMajor)
        {
            for (Index row = 0; row < windowSize; ++row)
            {
                Index detectionRow = windowRow + row;

                for (Index column = 0; column < windowSize; ++column)
                {
                    MakePoints(detectionRow, windowColumn + column);
                }
            }
        }
        else
        {
            // Iterate in column-major order.
            for (Index column = 0; column < windowSize; ++column)
            {
                Index detectionColumn = windowColumn + column;

                for (Index row = 0; row < windowSize; ++row)
                {
                    MakePoints(windowRow + row, detectionColumn);
                }
            }
        }

        if (this->points_.size() < this->count_)
        {
            return;
        }

        double centroidX = 0;
        double centroidY = 0;
        auto pointCount = static_cast<double>(this->points_.size());

        for (auto &point: this->points_)
        {
            centroidX += point.x;
            centroidY += point.y;

            // Remove the used points so they do not contribute to other corner
            // detections.
            input(static_cast<Index>(point.y), static_cast<Index>(point.x)) = 0;
        }

        this->corners_.emplace_back(
            centroidX / pointCount,
            centroidY / pointCount,
            pointCount);
    }

private:
    size_t windowSize_;
    size_t count_;
    Point2dCollection<double> points_;
    CornerPointsCollection corners_;
};


} // end namespace internal


class Corner
{
public:
    Corner(const CornerSettings &settings)
        :
        count_(static_cast<size_t>(settings.count)),
        windowSize_(settings.window)
    {
        assert(settings.count < settings.window * settings.window);
        assert(settings.count > 0);
    }

    template<typename Data>
    CornerPointsCollection Filter(const Eigen::MatrixBase<Data> &input)
    {
        using Eigen::Index;

        // Make a modifiable copy of the input points.
        Data points = input.derived();

        Index rowCount = input.rows();
        Index columnCount = input.cols();

        Index limitRow = rowCount - this->windowSize_ + 1;
        Index limitColumn = columnCount - this->windowSize_ + 1;

        internal::CornerCollector cornerCollector(
            static_cast<size_t>(this->windowSize_),
            this->count_);

        // Move the window.
        if constexpr (MatrixTraits<Data>::isRowMajor)
        {
            for (Index row = 0; row < limitRow; ++row)
            {
                for (Index column = 0; column < limitColumn; ++column)
                {
                    cornerCollector.CollectFromWindow(points, row, column);
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
                    cornerCollector.CollectFromWindow(points, row, column);
                }
            }
        }

        return cornerCollector.GetCorners();
    }

private:
    size_t count_;
    Eigen::Index windowSize_;
};


} // end namespace tau
