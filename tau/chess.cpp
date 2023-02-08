#include <tau/chess.h>


namespace tau
{


ChessLine::ChessLine(
        double angleToleranceDegrees,
        double lineSeparation,
        const Point &first,
        const Point &second)
    :
    Base(first, second),
    angleToleranceDegrees_(angleToleranceDegrees),
    lineSeparation_(lineSeparation),
    points_()
{
    assert(first != second);
    this->points_.push_back(first);
    this->points_.push_back(second);

    // points_ must remain sorted for later unique insertion to work.
    std::sort(begin(this->points_), end(this->points_));
}


bool ChessLine::operator<(const ChessLine &other) const
{
    return this->LessThan(other, this->angleToleranceDegrees_);
}


bool ChessLine::operator==(const ChessLine &other) const
{
    return this->IsColinear(
        other,
        this->angleToleranceDegrees_,
        this->lineSeparation_);
}


double ChessLine::GetError(const Point &point_) const
{
    return this->DistanceToPoint(point_);
}


std::optional<ChessLine::Iterator> ChessLine::GetInsertion_(const Point &point_)
{
    return tau::GetUniqueInsertion(this->points_, point_);
}


void ChessLine::Combine(const ChessLine &other)
{
    for (const auto &point_: other.points_)
    {
        auto insertion = this->GetInsertion_(point_);

        if (insertion)
        {
            this->points_.insert(*insertion, point_);
        }
    }

    // Recreate the line to account for the new points.
    auto updated = Base(this->points_);
    this->point = updated.point;
    this->vector = updated.vector;
}


bool ChessLine::AddPoint(const Point &point_)
{
    auto insertion = this->GetInsertion_(point_);

    if (!insertion)
    {
        // This point is already a member of this line.
        return false;
    }

    auto sanity = std::find(begin(this->points_), end(this->points_), point_);

    if (sanity != end(this->points_))
    {
        // Insanity!
        throw std::runtime_error("point already exists!");
    }

    this->points_.insert(*insertion, point_);

    // Recreate the line to account for the new point.
    auto updated = Base(this->points_);
    this->point = updated.point;
    this->vector = updated.vector;

    return true;
}


void ChessLine::RemoveOutliers(double maximumPointError)
{
    double pointError = 0;

    for (const auto &point_: this->points_)
    {
        pointError = std::max(pointError, this->GetError(point_));
    }

    auto pointsEnd = std::remove_if(
        begin(this->points_),
        end(this->points_),
        [this, maximumPointError] (const auto &point_) -> bool
        {
            return this->GetError(point_) > maximumPointError;
        });

    if (pointsEnd != end(this->points_))
    {
        this->points_.erase(pointsEnd, end(this->points_));

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


size_t ChessLine::GetPointCount() const
{
    return this->points_.size();
}


std::vector<ChessLine::Point> ChessLine::GetPoints() const
{
    return this->points_;
}


ChessLineGroup::ChessLineGroup()
    :
    angle{},
    lines{}
{

}


ChessLineGroup::ChessLineGroup(const ChessLine &line)
    :
    angle(line.GetAngleDegrees()),
    lines({line})
{

}


void ChessLineGroup::AddLine(const ChessLine &line)
{
    this->lines.push_back(line);
    this->angle = ToDegrees(GetAverageAngleRadians<double>(this->lines));
}


double ChessLineGroup::GetSpacingRatio() const
{
    return this->maximumSpacing / this->minimumSpacing;
}


size_t ChessLineGroup::GetLogicalIndex(size_t lineIndex) const
{
    return this->logicalIndices.at(lineIndex);
}


void ChessLineGroup::Sort(
    [[maybe_unused]] double spacingLimit,
    double lineSeparation,
    bool isHorizontal)
{
    if (this->lines.size() < 2)
    {
        // Not enough lines to sort.
        return;
    }

    // Get perpendicular line
    // The range of values for this->angle is 0 to 180.
    double perpendicularAngle = this->angle + 90;

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

    auto perpendicular = Line2d<double>(
        this->lines[0].point,
        Vector2d<double>(
            std::cos(perpendicularAngle_rad),
            std::sin(perpendicularAngle_rad)));

    // Sort the lines by their position along the intersecting line.
    std::sort(
        begin(this->lines),
        end(this->lines),
        [&](const ChessLine &first, const ChessLine &second) -> bool
        {
            return perpendicular.DistanceToIntersection(first)
                < perpendicular.DistanceToIntersection(second);
        });

    using Eigen::Index;

    this->spacings = Eigen::VectorX<double>(this->lines.size() - 1);

    for (size_t i = 0; i < this->lines.size() - 1; ++i)
    {
        const ChessLine &first = this->lines[i];
        const ChessLine &second = this->lines[i + 1];

        auto spacing =
            perpendicular.DistanceToIntersection(second)
            - perpendicular.DistanceToIntersection(first);

        this->spacings(static_cast<Index>(i)) = spacing;
    }

    this->minimumSpacing = this->spacings.minCoeff();
    this->maximumSpacing = this->spacings.maxCoeff();
    this->logicalIndices = std::vector<size_t>();

    if (this->minimumSpacing < lineSeparation)
    {
        // Unable to safely guess the logical indices.

        std::cout << "Info: minimum spacing ("
            << this->minimumSpacing
            << ") is less than line separation ("
            << lineSeparation << ")" << std::endl;

        return;
    }

    size_t logicalIndex = 0;
    this->logicalIndices.push_back(logicalIndex);

    for (auto spacing: this->spacings)
    {
        logicalIndex +=
            static_cast<size_t>(std::round(spacing / this->minimumSpacing));

        this->logicalIndices.push_back(logicalIndex);
    }
}


void ChessSolution::AddLine(
    const ChessLine &line,
    double groupSeparationDegrees)
{
    double angle = line.GetAngleDegrees();

    auto group = std::upper_bound(
        begin(this->groups),
        end(this->groups),
        angle,
        [](double angle_, const auto &group_) -> bool
        {
            return angle_ < group_.angle;
        });

    // The group iterator points to the first group for which
    // angle < group.angle is false.
    // It could also point to the end.

    // Check the group found by upper_bound
    if (group != end(this->groups))
    {
        // It is not the end, so it is safe to dereference.
        if (CompareLineAngles(angle, group->angle, groupSeparationDegrees))
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

        if (CompareLineAngles(angle, group->angle, groupSeparationDegrees))
        {
            group->AddLine(line);
            return;
        }

        // Restore group to the upper_bound for insertion of a new group.
        ++group;
    }

    // Insert a new group.
    this->groups.insert(group, ChessLineGroup(line));
}


std::optional<ChessLine::Point> ChessSolution::FindPoint(
    const Point &candidate,
    const std::vector<Point> &horizontalPoints,
    const ChessLine &vertical,
    double maximumPointError)
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


ChessSolution::Intersections
ChessSolution::FormIntersections(double maximumPointError)
{
    auto result = Intersections{};

    if (this->vertical.lines.empty() || this->horizontal.lines.empty())
    {
        return result;
    }

    if (
        this->vertical.logicalIndices.empty()
        || this->horizontal.logicalIndices.empty())
    {
        // Previous stages may not have been able to guess the logical
        // indices.
        // We cannot label intersections.
        return result;
    }

    size_t verticalCount = this->vertical.lines.size();
    size_t horizontalCount = this->horizontal.lines.size();

    // Iterate over the intersections of horizontal and vertical lines.
    // If an intersection has a point within the maximumPointError
    // threshold, consider that point a point on the chess board.
    for (auto j: jive::Range<size_t>(0, horizontalCount))
    {
        const ChessLine &horizontalLine = this->horizontal.lines[j];
        const auto &horizontalPoints = horizontalLine.GetPoints();
        auto logicalRow = this->horizontal.GetLogicalIndex(j);

        for (auto i: jive::Range<size_t>(0, verticalCount))
        {
            const ChessLine &verticalLine = this->vertical.lines[i];
            auto intersection = verticalLine.Intersect(horizontalLine);

            auto point = FindPoint(
                intersection,
                horizontalPoints,
                verticalLine,
                2 * maximumPointError);

            if (point.has_value())
            {
                auto logicalColumn = this->vertical.GetLogicalIndex(i);

                result.push_back(
                    {{logicalColumn, logicalRow}, *point});
            }
        }
    }

    return result;
}


ChessSolution ChessSolution::Create(
    const LineCollection &lines_,
    const ChessSettings &settings)
{
    ChessSolution solution{};
    solution.lines = lines_;
    solution.groups.emplace_back(solution.lines.at(0));

    double groupSeparationDegrees = settings.groupSeparationDegrees;
    size_t index = 1;

    while (index < solution.lines.size())
    {
        solution.AddLine(solution.lines[index++], groupSeparationDegrees);
    }

    // Combine the first and last group if they are within the group
    // separation angular limit.
    if (solution.groups.size() > 1)
    {
        // There are at least two groups.
        auto &front = solution.groups.front();
        auto &back = solution.groups.back();

        if (CompareLineAngles(
                front.angle,
                back.angle,
                groupSeparationDegrees))
        {
            // Combine the groups.
            for (auto &line: back.lines)
            {
                front.lines.push_back(line);
            }

            front.angle =
                ToDegrees(GetAverageAngleRadians<double>(front.lines));

            // Remove back.
            solution.groups.pop_back();
        }
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
        [](const ChessLineGroup &first, const ChessLineGroup &second) -> bool
        {
            return first.lines.size() > second.lines.size();
        });

    // Choose the largest group to be "vertical"...
    auto group = begin(solution.groups);

    solution.vertical = *group;

    solution.vertical.Sort(
        settings.spacingLimit,
        settings.lineSeparation,
        false);

    double verticalAngle = solution.vertical.angle;

    ++group;

    if (group == end(solution.groups))
    {
        return solution;
    }

    GroupCollection theRest(group, end(solution.groups));

    // Sort descending by number of lines and by how close the angle is to
    // 90 from the vertical
    solution.groups.sort(
        [&](const ChessLineGroup &first, const ChessLineGroup &second) -> bool
        {
            auto sizeDifference = std::abs(
                static_cast<ssize_t>(first.lines.size())
                    - static_cast<ssize_t>(second.lines.size()));

            double firstAngleDifference = std::abs(
                LineAngleDifference(first.angle, verticalAngle));

            double secondAngleDifference = std::abs(
                LineAngleDifference(second.angle, verticalAngle));

            double firstAngle = std::abs(
                LineAngleDifference(double{90}, firstAngleDifference));

            double secondAngle = std::abs(
                LineAngleDifference(double{90}, secondAngleDifference));

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
        double difference = std::abs(
            LineAngleDifference(group->angle, verticalAngle));

        if (difference > 70)
        {
            solution.horizontal = *group;

            solution.horizontal.Sort(
                settings.spacingLimit,
                settings.lineSeparation,
                true);

            break;
        }

        ++group;
    }

    solution.intersections = solution.FormIntersections(
        settings.maximumPointError);

    return solution;
}


LineCollector::LineCollector(const ChessSettings &settings)
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


void LineCollector::AddToLines(
    const Point2d<double> &firstPoint,
    const Point2d<double> &secondPoint)
{
    // Add points to any existing lines that are below the threshold.
    bool exists = false;

    ChessLine candidateLine(
        this->angleToleranceDegrees,
        this->lineSeparation,
        firstPoint,
        secondPoint);

    for (auto &line: this->lines)
    {
        if (line.GetError(firstPoint) <= this->maximumPointError)
        {
            line.AddPoint(firstPoint);
        }

        if (line.GetError(secondPoint) <= this->maximumPointError)
        {
            line.AddPoint(secondPoint);
        }

        if (line == candidateLine)
        {
            exists = true;
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
void LineCollector::RemoveOutliers()
{
    for (auto &line: this->lines)
    {
        line.RemoveOutliers(this->maximumPointError);
    }
}


// Apply angle and minimum points filters, and remove duplicates.
void LineCollector::Filter()
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

    // As points are added to lines, the lines shift slightly to fit the
    // new points. This leads to duplicate lines.
    std::sort(begin(this->lines), end(this->lines));

    // Remove duplicate lines.
    LineCollection filtered{};

    auto line = begin(this->lines);

    while (line != end(this->lines))
    {
        auto adjacent = std::adjacent_find(line, end(this->lines));
        filtered.insert(end(filtered), line, adjacent);

        if (adjacent == end(this->lines))
        {
            // There were no duplicates.
            break;
        }

        auto duplicate = adjacent;

        while (++duplicate != end(this->lines))
        {
            // Search up to the end for duplicates
            if (*adjacent != *duplicate)
            {
                // duplicate points to the next line greater than adjacent.
                break;
            }

            // duplicate is actually a duplicate.
            // Combine the points into one line.
            adjacent->Combine(*duplicate);
        }

        assert(adjacent != end(this->lines));
        filtered.push_back(*adjacent);
        line = duplicate;
    }

    this->lines = filtered;
}


ChessSolution::LineCollection
LineCollector::FormLines(const CornerPointsCollection &corners)
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


} // end namespace tau
