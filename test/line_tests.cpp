#include <catch2/catch.hpp>

#include <jive/range.h>
#include <jive/testing/generator_limits.h>

#include <tau/line2d.h>
#include <tau/angles.h>
#include <tau/random.h>


TEST_CASE("Line2d created from points", "[line2d]")
{
    auto values = GENERATE(
        take(
            16,
            chunk(
                4,
                random(-2000.0, 2000.0))));

    auto origin = tau::Point2d<double>(values.at(0), values.at(1));
    auto endPoint = tau::Point2d<double>(values.at(2), values.at(3));
    auto line = tau::Line2d<double>(origin, endPoint);

    auto expectedDirection = endPoint - origin;

    auto magnitude = std::sqrt(
        std::pow(expectedDirection.x, 2)
        + std::pow(expectedDirection.y, 2));

    expectedDirection /= magnitude;

    REQUIRE(line.vector.x == Approx(expectedDirection.x));
    REQUIRE(line.vector.y == Approx(expectedDirection.y));
}


TEST_CASE("Intersection", "[line2d]")
{
    using namespace tau;

    auto line1 = Line2d<double>(Point2d<double>(0, 0), Point2d<double>(1, 1));
    auto line2 = Line2d<double>(Point2d<double>(2, 0), Point2d<double>(2, 10));

    auto expectedIntersection = Point2d<double>(2, 2);
    REQUIRE(expectedIntersection == line1.Intersect(line2));
    REQUIRE(expectedIntersection == line2.Intersect(line1));
}


TEST_CASE("Distance to point", "[line2d]")
{
    using namespace tau;

    auto line = Line2d<double>(Point2d<double>(0, 0), Point2d<double>(1, 1));
    auto pointOnLine = Point2d<double>(2, 2);
    REQUIRE(line.DistanceToPoint(pointOnLine) == 0);

    auto pointAboveLine = Point2d<double>(2, 4);
    REQUIRE(line.DistanceToPoint(pointAboveLine) == Approx(std::sqrt(2)));

    auto pointBelowLine = Point2d<double>(2, 0);
    REQUIRE(line.DistanceToPoint(pointBelowLine) == Approx(std::sqrt(2)));

    auto pointLeftOfLine = Point2d<double>(-2, 2);
    double expectedDistance = std::sqrt(8);
    REQUIRE(line.DistanceToPoint(pointLeftOfLine) == expectedDistance);

    auto pointRightOfLine = Point2d<double>(2, -2);
    REQUIRE(line.DistanceToPoint(pointRightOfLine) == expectedDistance);
}


TEST_CASE("Line fit to points", "[line2d]")
{
    auto points = std::vector<tau::Point2d<double>>(
        {
            {1, 4},
            {3, 4},
            {3, 5},
            {5, 5},
            {5, 6},
            {7, 6},
            {9, 8},
            {11, 8}
        });

    auto line = tau::Line2d<double>(points);
    REQUIRE(line.point.x == 5.5);
    REQUIRE(line.point.y == 5.75);
}


TEST_CASE("Line fit to randomized points", "[line2d]")
{
    auto seed = GENERATE(
        take(32, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    // Generate a random direction for the "truth" line.
    tau::UniformRandom<double> uniformRandom{
        seed,
        -tau::Angles<double>::pi,
        tau::Angles<double>::pi};

    auto angle = uniformRandom();
    auto truthVector = tau::Vector2d<double>(std::cos(angle), std::sin(angle));

    // Randomly select a starting point.
    uniformRandom.SetRange(0, 1000);

    auto initialPoint =
        tau::Point2d<double>(uniformRandom(), uniformRandom());

    auto truthLine = tau::Line2d<double>(initialPoint, truthVector);

    // Generate random points along the line.
    uniformRandom.SetRange(-1000, 1000);

    const Eigen::Index pointCount = 100;

    Eigen::VectorX<double> offsets(pointCount);
    uniformRandom(offsets);

    std::vector<tau::Point2d<double>> points;
    points.reserve(static_cast<size_t>(pointCount));

    // Populate points and add noise.
    uniformRandom.SetRange(-.5, .5);

    REQUIRE(truthLine.GetAngleRadians() == Approx(angle));

    for (Eigen::Index i = 0; i < pointCount; ++i)
    {
        auto point = truthLine.GetEndPoint(offsets(i));

        point.x += uniformRandom();
        point.y += uniformRandom();
        points.push_back(point);
    }

    auto line = tau::Line2d<double>(points);

    auto lineAngle = std::fmod(
        line.GetAngleRadians() + tau::Angles<double>::pi,
        tau::Angles<double>::pi);

    auto truthAngle =
        std::fmod(angle + tau::Angles<double>::pi, tau::Angles<double>::pi);

    auto angleError = tau::ToDegrees(lineAngle - truthAngle);
    auto distanceError = line.DistanceToPoint(initialPoint);

    REQUIRE(angleError < 0.2);
    REQUIRE(distanceError < 0.2);
}


TEST_CASE("Intersection with Region", "[line2d]")
{
    using namespace tau;

    using Point = Point2d<double>;
    using Line = Line2d<double>;
    using Size_ = Size<double>;
    using Region_ = Region<double>;

    auto region = Region_{Point(0, 0), Size_(600, 400)};
    auto point1 = Point(0, 200);
    auto point2 = Point(600, 100);
    auto line = Line(point1, point2);

    auto intersection = line.Intersect(region);

    REQUIRE(!!intersection);
    REQUIRE(intersection->first == point1);
    REQUIRE(intersection->second == point2);

    auto outsideLine = Line(region.size.Magnitude() + 1, -45.0);
    auto noIntersection = outsideLine.Intersect(region);
    REQUIRE(!noIntersection);
}


TEST_CASE("Line from Hesse Normal Form", "[line2d]")
{
    using Point = tau::Point2d<int>;
    using Vector = tau::Vector2d<int>;
    using Line = tau::Line2d<double>;
    using Size = tau::Size<double>;
    using Region = tau::Region<double>;

    Line line1(10.0, 0.0);
    REQUIRE(line1.point.template Cast<int>() == Point(10, 0));
    REQUIRE(line1.vector.template Cast<int>() == Vector(0, 1));

    Line line2(10.0, 90.0);
    REQUIRE(line2.point.template Cast<int>() == Point(0, 10));
    REQUIRE(line2.vector.template Cast<int>() == Vector(-1, 0));

    Line line3(10.0, 180.0);
    REQUIRE(line3.point.template Cast<int>() == Point(-10, 0));
    REQUIRE(line3.vector.template Cast<int>() == Vector(0, 1));

    Line line4(-10.0, 180.0);
    REQUIRE(line4.point.template Cast<int>() == Point(10, 0));
    REQUIRE(line4.vector.template Cast<int>() == Vector(0, 1));

    auto region = Region{{Point(-20, -20), Size(40, 40)}};
    auto intersection1 = line1.Intersect(region);
    auto intersection2 = line2.Intersect(region);
    auto intersection3 = line3.Intersect(region);
    auto intersection4 = line4.Intersect(region);

    CHECK(!!intersection1);
    CHECK(!!intersection2);
    CHECK(!!intersection3);
    CHECK(!!intersection4);
}
