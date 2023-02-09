#pragma once

#include <optional>

#include <tau/vector2d.h>
#include <tau/line2d.h>
#include <tau/corner.h>
#include <tau/chess_settings.h>


namespace tau
{


class ChessLine: public Line2d<double>
{
public:
    using Base = Line2d<double>;
    using Point = Point2d<double>;
    using Points = std::vector<Point>;

    ChessLine(
        double angleToleranceDegrees,
        double lineSeparation,
        const Point &first,
        const Point &second);

    double GetError(const Point &point_) const;

    void Combine(const ChessLine &other);

    bool AddPoint(const Point &point);

    void RemoveOutliers(double maximumPointError);

    size_t GetPointCount() const;

    Points GetPoints() const;

    bool operator<(const ChessLine &other) const;

    bool operator==(const ChessLine &other) const;

private:
    using Iterator = typename Points::iterator;
    std::optional<Iterator> GetInsertion_(const Point &point);

private:
    double angleToleranceDegrees_;
    double lineSeparation_;
    Points points_;
};


struct ChessLineGroup
{
    using LineCollection = std::vector<ChessLine>;

    ChessLineGroup();

    ChessLineGroup(const ChessLine &line);

    void AddLine(const ChessLine &line);

    double GetSpacingRatio() const;

    size_t GetLogicalIndex(size_t lineIndex) const;

    Line2d<double> GetPerpendicular(bool isHorizontal) const;

    void Sort(bool isHorizontal);

    std::vector<ChessLineGroup> Split(
        double spacingLimit,
        double lineSeparation);

    void ComputeIndices(bool isHorizontal, double lineSeparation);

    double angle;
    LineCollection lines;
    Eigen::VectorX<double> spacings;
    double minimumSpacing;
    double maximumSpacing;
    std::vector<size_t> logicalIndices;
};


struct ChessIntersection
{
    Point2d<size_t> logical;
    Point2d<double> pixel;

    static constexpr auto fields = std::make_tuple(
        fields::Field(&ChessIntersection::logical, "logical"),
        fields::Field(&ChessIntersection::pixel, "pixel"));

    static constexpr auto fieldsTypeName = "ChessIntersection";
};


struct ChessSolution
{
    using Point = typename ChessLine::Point;
    using GroupCollection = std::list<ChessLineGroup>;
    using LineCollection = typename ChessLineGroup::LineCollection;
    using Intersections = std::vector<ChessIntersection>;

    ChessSolution() = default;

    static ChessSolution Create(
        const LineCollection &lines_,
        const ChessSettings &settings);

    void AddLine(
        const ChessLine &line,
        double groupSeparationDegrees);

    Intersections FormIntersections(double maximumPointError);

    static std::optional<Point> FindPoint(
        const Point &candidate,
        const std::vector<Point> &horizontalPoints,
        const ChessLine &vertical,
        double maximumPointError);

    LineCollection lines;
    GroupCollection groups;
    ChessLineGroup horizontal;
    ChessLineGroup vertical;
    Intersections intersections;
};


class LineCollector
{
public:
    using LineCollection = std::vector<ChessLine>;

    LineCollection lines;
    double maximumPointError;
    double angleToleranceDegrees;
    double lineSeparation;

    size_t minimumPointsPerLine;
    double angleFilterLow;
    double angleFilterHigh;

    LineCollector(const ChessSettings &settings);

    void AddToLines(
        const Point2d<double> &firstPoint,
        const Point2d<double> &secondPoint);

    // As lines are constructed, only the error from the newest point is
    // considered. The updated average line can be pulled away from earlier
    // points, leaving them as outliers that have too much point error.
    void RemoveOutliers();

    // Apply angle and minimum points filters.
    void Filter();

    LineCollection FormLines(const CornerPointsCollection &corners);
};


class Chess
{
public:
    using LineCollection = typename LineCollector::LineCollection;

    Chess(const ChessSettings &settings)
        :
        settings_(settings)
    {

    }

    ChessSolution GroupLines(const LineCollection &lines)
    {
        if (lines.empty())
        {
            return {};
        }

        return ChessSolution::Create(lines, this->settings_);
    }


    LineCollection FormLines(const CornerPointsCollection &corners)
    {
        if (corners.empty())
        {
            return {};
        }

        return LineCollector(this->settings_).FormLines(corners);
    }

private:
    ChessSettings settings_;
};


} // end namespace tau
