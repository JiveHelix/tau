#pragma once


#include <tau/vector2d.h>
#include <tau/line2d.h>


namespace tau
{


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

        auto pointsEnd = std::remove_if(
            begin(this->points_),
            end(this->points_),
            [this, maximumPointError] (const auto &point) -> bool
            {
                return this->GetError(point) > maximumPointError;
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
    using Line = ChessLine<Float>;
    using LineCollection = std::vector<Line>;

    ChessLineGroup()
        :
        angle{},
        lines{}
    {

    }

    ChessLineGroup(const Line &line)
        :
        angle(line.GetAngleDegrees()),
        lines({line})
    {

    }

    void AddLine(const Line &line)
    {
        this->lines.push_back(line);
        this->angle = ToDegrees(GetAverageAngleRadians<Float>(this->lines));
    }

    Float GetSpacingRatio() const
    {
        return this->maximumSpacing / this->minimumSpacing;
    }

    size_t GetLogicalIndex(size_t lineIndex) const
    {
        return this->logicalIndices.at(lineIndex);
    }

    Float angle;
    LineCollection lines;
    Eigen::VectorX<Float> spacings;
    Float minimumSpacing;
    Float maximumSpacing;
    std::vector<size_t> logicalIndices;
};


template<typename Float>
struct ChessIntersection
{
    Point2d<size_t> logical;
    Point2d<Float> pixel;

    static constexpr auto fields = std::make_tuple(
        fields::Field(&ChessIntersection::logical, "logical"),
        fields::Field(&ChessIntersection::pixel, "pixel"));

    static constexpr auto fieldsTypeName = "ChessIntersection";
};


template<typename Float>
struct ChessSolution
{
    using LineGroup = ChessLineGroup<Float>;
    using GroupCollection = std::list<LineGroup>;
    using LineCollection = typename LineGroup::LineCollection;
    using Intersection = ChessIntersection<Float>;
    using Intersections = std::vector<Intersection>;

    ChessSolution() = default;

    LineCollection lines;
    GroupCollection groups;
    LineGroup horizontal;
    LineGroup vertical;
    Intersections intersections;
};


} // end namespace tau
