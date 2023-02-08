#include "tau/corner.h"


namespace tau
{


CornerPoint::CornerPoint(double x, double y, double count_)
    :
    point(x, y),
    count(count_)
{

}

bool CornerPoint::operator>(const CornerPoint &other) const
{
    if (this->point.template Convert<int>()
        == other.point.template Convert<int>())
    {
        return this->count > other.count;
    }

    return this->point.template Convert<int>()
        > other.point.template Convert<int>();
}

bool CornerPoint::operator<(const CornerPoint &other) const
{
    if (this->point.template Convert<int>()
        == other.point.template Convert<int>())
    {
        return this->count < other.count;
    }

    return (this->point.template Convert<int>()
        < other.point.template Convert<int>());
}

// For the purpose of determining unique corners, corners with the same
// point compare equal, even if their counts differ.
bool CornerPoint::operator==(const CornerPoint &other) const
{
    return (this->point.template Convert<int>()
        == other.point.template Convert<int>());
}


CornerPointsCollection GetPoints(const ImageMatrixFloat &input)
{
    using Eigen::Index;

    CornerPointsCollection points;

    if constexpr (MatrixTraits<ImageMatrixFloat>::isRowMajor)
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


CornerCollector::CornerCollector(size_t windowSize, size_t count)
    :
    windowSize_(windowSize),
    count_(count),
    points_(),
    corners_()
{
    this->points_.reserve(windowSize * windowSize);
}


const CornerPointsCollection & CornerCollector::GetCorners()
{
    return this->corners_;
}


void CornerCollector::CollectFromWindow(
    ImageMatrixFloat &input,
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

    if constexpr (MatrixTraits<ImageMatrixFloat>::isRowMajor)
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


} // end namespace internal


Corner::Corner(const CornerSettings &settings)
    :
    count_(static_cast<size_t>(settings.count)),
    windowSize_(settings.window)
{
    assert(settings.count < settings.window * settings.window);
    assert(settings.count > 0);
}

CornerPointsCollection Corner::Filter(const ImageMatrixFloat &input)
{
    using Eigen::Index;

    // Make a modifiable copy of the input points.
    ImageMatrixFloat points = input.derived();

    Index rowCount = input.rows();
    Index columnCount = input.cols();

    Index limitRow = rowCount - this->windowSize_ + 1;
    Index limitColumn = columnCount - this->windowSize_ + 1;

    internal::CornerCollector cornerCollector(
        static_cast<size_t>(this->windowSize_),
        this->count_);

    // Move the window.
    if constexpr (MatrixTraits<ImageMatrixFloat>::isRowMajor)
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

    auto corners = cornerCollector.GetCorners();
    std::sort(begin(corners), end(corners));


    auto size = corners.size();

    std::cout << "corners: " << size;

    for (auto &corner: corners)
    {
        std::cout << "\n  " << fields::DescribeColorized(corner.point)
            << std::endl;
    }

    auto last = std::unique(begin(corners), end(corners));

    corners.erase(last, end(corners));

    std::cout << "Removed " << size - corners.size() << " duplicate corners" << std::endl;

    return corners;
}


} // end namespace tau
