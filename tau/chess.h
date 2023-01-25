#pragma once

#include <optional>
#include <tau/corner.h>
#include <tau/chess_settings.h>
#include <tau/chess_solution.h>


namespace tau
{


template<typename Float>
class ChessMethods
{
public:
    using Point = Point2d<Float>;
    using Solution = ChessSolution<Float>;
    using Line = typename ChessLineGroup<Float>::Line;
    using LineCollection = typename Solution::LineCollection;
    using Intersections = typename Solution::Intersections;
    using Group = typename Solution::LineGroup;
    using GroupCollection = typename Solution::GroupCollection;


    static void SortGroup(Group &group, Float lineSeparation, bool isHorizontal)
    {

        if (group.lines.size() < 2)
        {
            // Not enough lines to sort.
            return;
        }

        // Get perpendicular line
        // The range of values for group.angle is 0 to 180.
        Float perpendicularAngle = group.angle + 90;

        if (isHorizontal)
        {
            // We want the perpendicularAngle to be close to 90, not -90.
            if (perpendicularAngle > 180)
            {
                perpendicularAngle -= 180;
            }
        }
        else
        {
            // We want the perpendicularAngle to be close to 0, not
            // 180
            if (perpendicularAngle > 90)
            {
                perpendicularAngle -= 180;
            }
        }

        auto perpendicularAngle_rad = tau::ToRadians(perpendicularAngle);

        auto perpendicular = Line2d<Float>(
            group.lines[0].point,
            Vector2d<Float>(
                std::cos(perpendicularAngle_rad),
                std::sin(perpendicularAngle_rad)));

        // Sort the lines by their position along the intersecting line.
        std::sort(
            begin(group.lines),
            end(group.lines),
            [&](const Line &first, const Line &second) -> bool
            {
                return perpendicular.DistanceToIntersection(first)
                    < perpendicular.DistanceToIntersection(second);
            });

        using Eigen::Index;

        group.spacings = Eigen::VectorX<Float>(group.lines.size() - 1);

        for (size_t i = 0; i < group.lines.size() - 1; ++i)
        {
            const Line &first = group.lines[i];
            const Line &second = group.lines[i + 1];

            group.spacings(static_cast<Index>(i)) = perpendicular.DistanceToIntersection(second)
                - perpendicular.DistanceToIntersection(first);
        }

        group.minimumSpacing = group.spacings.minCoeff();
        group.maximumSpacing = group.spacings.maxCoeff();
        group.logicalIndices = std::vector<size_t>();

        size_t logicalIndex = 0;
        group.logicalIndices.push_back(logicalIndex);

        if (group.minimumSpacing < lineSeparation)
        {
            throw std::runtime_error(
                "minimum spacing is less than line separation");
        }

        for (auto spacing: group.spacings)
        {
            logicalIndex +=
                static_cast<size_t>(std::round(spacing / group.minimumSpacing));

            group.logicalIndices.push_back(logicalIndex);
        }
    }


    static std::optional<Point> FindPoint(
        const Point &candidate,
        const std::vector<Point> &horizontalPoints,
        const Line &vertical,
        Float maximumPointError)
    {
        if (horizontalPoints.empty())
        {
            throw std::logic_error("must have points");
        }

        for (const auto &point: horizontalPoints)
        {
            auto distance = candidate.Distance(point);

            if (distance < maximumPointError)
            {
                return {point};
            }
        }

        for (const auto &point: vertical.GetPoints())
        {
            auto distance = candidate.Distance(point);

            if (distance < maximumPointError)
            {
                return {point};
            }
        }

        return {};
    }


    static Intersections FormIntersections(
        const Group &vertical,
        const Group &horizontal,
        Float maximumPointError)
    {
        auto result = Intersections{};

        if (vertical.lines.empty() || horizontal.lines.empty())
        {
            return result;
        }

        size_t verticalCount = vertical.lines.size();
        size_t horizontalCount = horizontal.lines.size();

        // Iterate over the intersections of horizontal and vertical lines.
        // If an intersection has a point within the maximumPointError
        // threshold, consider that point a point on the chess board.
        for (auto j: jive::Range<size_t>(0, horizontalCount))
        {
            const Line &horizontalLine = horizontal.lines[j];
            const auto &horizontalPoints = horizontalLine.GetPoints();
            auto logicalRow = horizontal.GetLogicalIndex(j);

            for (auto i: jive::Range<size_t>(0, verticalCount))
            {
                const Line &verticalLine = vertical.lines[i];
                auto intersection = verticalLine.Intersect(horizontalLine);

                auto point = FindPoint(
                    intersection,
                    horizontalPoints,
                    verticalLine,
                    2 * maximumPointError);

                if (point.has_value())
                {
                    auto logicalColumn = vertical.GetLogicalIndex(i);

                    result.push_back(
                        {{logicalColumn, logicalRow}, *point});
                }
            }
        }

        return result;
    }


    static void AddLine(
        GroupCollection &groups,
        const Line &line,
        Float groupSeparationDegrees)
    {
        Float angle = line.GetAngleDegrees();

        auto group = std::upper_bound(
            begin(groups),
            end(groups),
            angle,
            [](Float angle, const auto &group) -> bool
            {
                return angle < group.angle;
            });

        // The group iterator points to the first group for which
        // angle < group.angle is false.
        // It could also point to the end.

        // Check the group found by upper_bound
        if (group != end(groups))
        {
            // It is not the end, so it is safe to dereference.
            if (CompareLineAngles(angle, group->angle, groupSeparationDegrees))
            {
                group->AddLine(line);
                return;
            }
        }

        // Check the preceding group
        if (group != begin(groups))
        {
            // Check the previous group for a match
            --group;

            if (CompareLineAngles( angle, group->angle, groupSeparationDegrees))
            {
                group->AddLine(line);
                return;
            }

            // Restore group to the upper_bound for insertion of a new group.
            ++group;
        }

        // Insert a new group.
        groups.insert(group, Group(line));
    }


    static Solution Create(
        const LineCollection &lines_,
        const ChessSettings<Float> &settings)
    {
        Solution solution{};
        solution.lines = lines_;
        solution.groups.emplace_back(solution.lines.at(0));

        Float groupSeparationDegrees = settings.groupSeparationDegrees;
        size_t index = 1;

        while (index < solution.lines.size())
        {
            AddLine(
                solution.groups,
                solution.lines[index++],
                groupSeparationDegrees);
        }

        auto upperLimit = std::max(settings.rowCount, settings.columnCount);

        // Remove groups that are outside the allowable range.
        auto filteredEnd = std::remove_if(
            begin(solution.groups),
            end(solution.groups),
            [&] (const auto &group) -> bool
            {
                return (group.lines.size() < settings.minimumLinesPerGroup)
                    || (group.lines.size() > upperLimit);
            });

        if (filteredEnd != end(solution.groups))
        {
            solution.groups.erase(filteredEnd, end(solution.groups));
        }

        if (solution.groups.empty())
        {
            return solution;
        }

        // There is at least one group, and
        // all remaining groups have the minimum number of lines.

        // Sort descending by number of lines.
        solution.groups.sort(
            [](const Group &first, const Group &second) -> bool
            {
                return first.lines.size() > second.lines.size();
            });

        // Choose the largest group to be "vertical"...
        auto group = begin(solution.groups);

        solution.vertical = *group;
        SortGroup(solution.vertical, settings.lineSeparation, false);

        Float verticalAngle = solution.vertical.angle;

        ++group;

        if (group == end(solution.groups))
        {
            return solution;
        }

        GroupCollection theRest(group, end(solution.groups));

        // Sort descending by number of lines and by how close the angle is to
        // 90 from the vertical
        solution.groups.sort(
            [&](const Group &first, const Group &second) -> bool
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

        group = begin(solution.groups);

        while (group != end(solution.groups))
        {
            Float difference = std::abs(
                LineAngleDifference(group->angle, verticalAngle));

            if (difference > 70)
            {
                solution.horizontal = *group;
                SortGroup(solution.horizontal, settings.lineSeparation, true);
                break;
            }

            ++group;
        }

        solution.intersections = FormIntersections(
            solution.vertical,
            solution.horizontal,
            settings.maximumPointError);

        return solution;
    }
};


template<typename Float>
class LineCollector
{
public:
    using LineCollection = std::vector<ChessLine<Float>>;

    LineCollection lines;
    Float maximumPointError;
    Float angleToleranceDegrees;
    Float lineSeparation;

    size_t minimumPointsPerLine;
    Float angleFilterLow;
    Float angleFilterHigh;

    LineCollector(ChessSettings<Float> &settings)
        :
        lines{},
        maximumPointError(settings.maximumPointError),
        angleToleranceDegrees(settings.angleToleranceDegrees),
        lineSeparation(settings.lineSeparation),
        minimumPointsPerLine(settings.minimumPointsPerLine),
        angleFilterLow(settings.angleFilter.low),
        angleFilterHigh(settings.angleFilter.high)
    {

    }

    void AddToLines(
        const Point2d<Float> &firstPoint,
        const Point2d<Float> &secondPoint)
    {
        // Add points to any existing lines that are below the threshold.
        bool exists = false;
        ChessLine<Float> candidateLine(firstPoint, secondPoint);

        for (auto &line: this->lines)
        {
            if (line.IsColinear(
                    candidateLine,
                    this->angleToleranceDegrees,
                    this->lineSeparation))
            {
                exists = true;
            }

            if (line.GetError(firstPoint) <= this->maximumPointError)
            {
                line.AddPoint(firstPoint);
            }

            if (line.GetError(secondPoint) <= this->maximumPointError)
            {
                line.AddPoint(secondPoint);
            }
        }

        if (!exists)
        {
            // The candidate line is not colinear with any of the other lines.
            // Add it to the collection.
            this->lines.push_back(candidateLine);
        }
    };

    // As lines are constructed, only the error from the newest point is
    // considered. The updated average line can be pulled away from earlier
    // points, leaving them as outliers that have too much point error.
    void RemoveOutliers()
    {
        for (auto &line: this->lines)
        {
            line.RemoveOutliers(this->maximumPointError);
        }
    }

    // Apply angle and minimum points filters.
    void Filter()
    {
        auto linesEnd = std::remove_if(
            begin(this->lines),
            end(this->lines),
            [this] (const auto &line) -> bool
            {
                return (line.GetPointCount() < this->minimumPointsPerLine)
                    || (line.GetAngleDegrees() < this->angleFilterLow)
                    || (line.GetAngleDegrees() > this->angleFilterHigh);
            });

        if (linesEnd != end(this->lines))
        {
            this->lines.erase(linesEnd, end(this->lines));
        }
    }

    LineCollection FormLines(const CornerPointsCollection<Float> &corners)
    {
        assert(!corners.empty());

        for (size_t i = 0; i < corners.size() - 1; ++i)
        {
            const auto &firstCorner = corners[i];

            for (size_t j = i + 1; j < corners.size(); ++j)
            {
                const auto &secondCorner = corners[j];
                this->AddToLines(firstCorner.point, secondCorner.point);
            }
        }

        this->RemoveOutliers();
        this->Filter();

        return this->lines;
    }
};


template<typename Float>
class Chess
{
public:
    using Methods = ChessMethods<Float>;
    using LineCollection = typename Methods::LineCollection;

    Chess(const ChessSettings<Float> &settings)
        :
        settings_(settings)
    {

    }

    ChessSolution<Float> GroupLines(const LineCollection &lines)
    {
        if (lines.empty())
        {
            return {};
        }

        return Methods::Create(lines, this->settings_);
    }


    LineCollection FormLines(const CornerPointsCollection<Float> &corners)
    {
        if (corners.empty())
        {
            return {};
        }

        return LineCollector<Float>(this->settings_).FormLines(corners);
    }

private:
    ChessSettings<Float> settings_;
};


} // end namespace tau
