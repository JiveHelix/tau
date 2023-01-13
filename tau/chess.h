#pragma once

#include <fields/fields.h>
#include <pex/interface.h>
#include <pex/linked_ranges.h>
#include <tau/corner.h>
#include <tau/vector2d.h>
#include <tau/line2d.h>


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

        return {{
            defaultMinimumPointsPerLine,
            defaultMaximumPointError,
            defaultAngleTolerance,
            defaultLineSeparation,
            false,
            20,
            {0, 180}}};
    }
};


template<typename Float>
class ChessLine: public Line2d<Float>
{
public:
    using Base = Line2d<Float>;
    using Point = Point2d<Float>;

    ChessLine(const Point &first, const Point &second)
        :
        Base(first, second),
        points_()
    {
        this->points_.push_back(first);
        this->points_.push_back(second);
    }

    Float GetError(const Point &point) const
    {
        return this->DistanceToPoint(point);
    }

    bool AddPoint(const Point &point)
    {
        auto exists = std::find(
            begin(this->points_),
            end(this->points_),
            point);

        if (exists != end(this->points_))
        {
            // This point is already a member of this line.
            return false;
        }

        this->points_.push_back(point);

        // Recreate the line to account for the new point.
        auto updated = Base(this->points_);
        this->point = updated.point;
        this->vector = updated.vector;

        return true;
    }

    void RemoveOutliers(Float maximumPointError)
    {
        Float pointError = 0;

        for (const auto &point: this->points_)
        {
            pointError = std::max(pointError, this->GetError(point));
        }

        if (pointError > maximumPointError)
        {
            std::cout << "pointError: " << pointError << std::endl;
        }

        auto pointsEnd = std::remove_if(
            begin(this->points_),
            end(this->points_),
            [this, maximumPointError] (const auto &point) -> bool
            {
                return this->GetError(point) > maximumPointError;
            });

        if (pointsEnd != end(this->points_))
        {
            size_t oldSize = this->points_.size();

            this->points_.erase(pointsEnd, end(this->points_));

            std::cout << "Removed " << oldSize - this->points_.size()
                << " outliers." << std::endl;

            if (this->points_.size() >=2)
            {
                // Recreate the line to account for the point removal.
                auto updated = Base(this->points_);
                this->point = updated.point;
                this->vector = updated.vector;
            }
            // else, there are not enough points to define a line.
        }
    }

    size_t GetPointCount() const
    {
        return this->points_.size();
    }

    std::vector<Point> GetPoints() const
    {
        return this->points_;
    }

private:
    std::vector<Point> points_;
};


template<typename Float>
struct ChessLineGroup
{
    using LineCollection = std::vector<ChessLine<Float>>;

    ChessLineGroup()
        :
        angle{},
        lines{}
    {

    }

    ChessLineGroup(const ChessLine<Float> &line)
        :
        angle(line.GetAngleDegrees()),
        lines({line})
    {

    }

    void AddLine(const ChessLine<Float> &line)
    {
        this->lines.push_back(line);
        this->angle = ToDegrees(GetAverageAngleRadians<Float>(this->lines));
    }

    Float angle;
    LineCollection lines;
};


template<typename Float>
struct ChessLines
{
    using LineGroup = ChessLineGroup<Float>;
    using GroupCollection = std::list<LineGroup>;
    using LineCollection = typename LineGroup::LineCollection;

    ChessLines() = default;

    ChessLines(const LineCollection &lines_, Float tolerance)
        :
        lines(lines_),
        groups({LineGroup(this->lines.at(0))}),
        horizontal{},
        vertical{}
    {
        std::cout << "ChessLines sorting into horizontal and vertical groups"
            << std::endl;

        size_t index = 1;

        while (index < this->lines.size())
        {
            this->AddLine(this->lines[index++], tolerance);
        }

        // Sort descending by number of lines.
        this->groups.sort(
            [](const LineGroup &first, const LineGroup &second) -> bool
            {
                return first.lines.size() > second.lines.size();
            });

        // Choose the largest group to be "vertical"...
        auto group = begin(this->groups);
        this->vertical = *group;
        Float verticalAngle = this->vertical.angle;

        ++group;

        if (group == end(this->groups))
        {
            return;
        }

        GroupCollection theRest(group, end(this->groups));

        // Sort descending by number of lines and by how close the angle is to
        // 90 from the vertical
        this->groups.sort(
            [&](const LineGroup &first, const LineGroup &second) -> bool
            {
                auto sizeDifference = std::abs(
                    static_cast<ssize_t>(first.lines.size())
                        - static_cast<ssize_t>(second.lines.size()));

                Float firstAngleDifference = std::abs(
                    LineAngleDifference(first.angle, verticalAngle));

                Float secondAngleDifference = std::abs(
                    LineAngleDifference(second.angle, verticalAngle));

                Float firstAngle = std::abs(
                    LineAngleDifference(Float{90}, firstAngleDifference));

                Float secondAngle = std::abs(
                    LineAngleDifference(Float{90}, secondAngleDifference));

                if (sizeDifference < 2)
                {
                    // The difference in size is minimal.
                    // Prefer the line closest to 90 degrees with the vertical.
                    return LineAngleDifference(firstAngle, secondAngle) < 0;
                }

                return first.lines.size() > second.lines.size();
            });

        group = begin(groups);

        while (group != end(groups))
        {
            Float difference = std::abs(
                LineAngleDifference(group->angle, verticalAngle));

            if (difference > 70)
            {
                this->horizontal = *group;
                return;
            }

            ++group;
        }
    }

    void AddLine(const ChessLine<Float> &line, Float tolerance)
    {
        Float angle = line.GetAngleDegrees();

        auto group = std::upper_bound(
            begin(this->groups),
            end(this->groups),
            angle,
            [](Float angle, const auto &group) -> bool
            {
                return angle < group.angle;
            });

        // The group iterator points to the first group for which
        // angle < group.angle is false.
        // It could also point to the end.

        // Check the group found by upper_bound
        if (group != end(this->groups))
        {
            // It is not the end, so it is safe to dereference.
            if (CompareLineAngles( angle, group->angle, tolerance))
            {
                group->AddLine(line);
                return;
            }
        }

        // Check the preceding group
        if (group != begin(this->groups))
        {
            // Check the previous group for a match
            --group;

            if (CompareLineAngles( angle, group->angle, tolerance))
            {
                group->AddLine(line);
                return;
            }

            // Restore group to the upper_bound for insertion of a new group.
            ++group;
        }

        // Insert a new group.
        this->groups.insert(group, LineGroup(line));
    }

    LineCollection lines;
    GroupCollection groups;
    LineGroup horizontal;
    LineGroup vertical;
};


template<typename Float>
class Chess
{
public:
    using LineCollection = std::vector<ChessLine<Float>>;

    Chess(const ChessSettings<Float> &settings)
        :
        settings_(settings)
    {

    }

    ChessLines<Float> GroupLines(const LineCollection &lines)
    {
        if (lines.empty())
        {
            return {};
        }

        return {lines, this->settings_.groupSeparationDegrees};
    }

    LineCollection FormLines(const CornerPointsCollection<Float> &corners)
    {
        if (corners.empty())
        {
            std::cerr << "corners is empty!" << std::endl;
            return {};
        }

        LineCollection lines;
        Float maximumPointError = this->settings_.maximumPointError;
        Float angleToleranceDegrees = this->settings_.angleToleranceDegrees;
        Float lineSeparation = this->settings_.lineSeparation;

        auto AddToLines =
            [&lines, maximumPointError, angleToleranceDegrees, lineSeparation] (
                    const auto candidateLine,
                    const auto &firstPoint,
                    const auto &secondPoint) -> bool
            {
                // Add point to any existing lines that are within the error
                // threshold.
                bool exists = false;

                for (auto &line: lines)
                {
                    bool thisLine = false;

                    auto oldLine = line;

                    if (line.IsColinear(
                            candidateLine,
                            angleToleranceDegrees,
                            lineSeparation))
                    {
                        exists = true;
                        thisLine = true;
                    }

                    if (line.GetError(firstPoint) <= maximumPointError)
                    {
                        line.AddPoint(firstPoint);
                    }

                    if (line.GetError(secondPoint) <= maximumPointError)
                    {
                        line.AddPoint(secondPoint);
                    }

                    if (thisLine)
                    {
                        if (!line.IsColinear(
                                candidateLine,
                                angleToleranceDegrees,
                                lineSeparation))
                        {
                            std::cout << "Line WAS co-linear!" << std::endl;

                            std::cout << "Added firstPoint: " << firstPoint
                                << ", secondPoint: " << secondPoint
                                << std::endl;

                            std::cout << "candidateLine: "
                                << candidateLine.GetAngleDegrees()
                                << ": " << candidateLine.point << std::endl;

                            std::cout << "oldLine: "
                                << oldLine.GetAngleDegrees()
                                << ": " << oldLine.point << std::endl;

                            for (const auto &p: oldLine.GetPoints())
                            {
                                std::cout << "\n  " << p << std::endl;
                            }

                            std::cout << "line: "
                                << line.GetAngleDegrees()
                                << ": " << line.point << std::endl;

                            for (const auto &p: line.GetPoints())
                            {
                                std::cout << "\n  " << p << std::endl;
                            }
                        }
                    }
                }

                return exists;
            };

        for (size_t i = 0; i < corners.size() - 1; ++i)
        {
            const auto &firstCorner = corners[i];

            for (size_t j = i + 1; j < corners.size(); ++j)
            {
                const auto &secondCorner = corners[j];

                auto line = ChessLine<Float>(
                    firstCorner.point,
                    secondCorner.point);

                if (!AddToLines(line, firstCorner.point, secondCorner.point))
                {
                    // line does not already exist.
                    // Add it to the collection.
                    lines.push_back(line);
                }
            }
        }

        size_t minimumPointsPerLine = this->settings_.minimumPointsPerLine;
        float angleFilterLow = this->settings_.angleFilter.low;
        float angleFilterHigh = this->settings_.angleFilter.high;

        // As lines are constructed, only the error from the newest point is
        // considered. The updated average line can be pulled away from earlier
        // points, leaving them as outliers that have too much point error.
        for (auto &line: lines)
        {
            line.RemoveOutliers(maximumPointError);
        }

        auto linesEnd = std::remove_if(
            begin(lines),
            end(lines),
            [minimumPointsPerLine, angleFilterLow, angleFilterHigh] (
                const auto &line) -> bool
            {
                return (line.GetPointCount() < minimumPointsPerLine)
                    || (line.GetAngleDegrees() < angleFilterLow)
                    || (line.GetAngleDegrees() > angleFilterHigh);
            });

        if (linesEnd != end(lines))
        {
            lines.erase(linesEnd, end(lines));
        }

        return lines;
    }

private:
    ChessSettings<Float> settings_;
};


} // end namespace tau
