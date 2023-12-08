#include <catch2/catch.hpp>
#include <Eigen/Dense>
#include <tau/eigen.h>

#include <tau/pose.h>
#include <tau/intrinsics.h>
#include <tau/projection.h>
#include <tau/literals.h>

using namespace tau::literals;


using Pixel = tau::Point2d<float>;


TEST_CASE("Pose origin projection", "[pose]")
{
    tau::Intrinsics<float> intrinsics{{
        10_f,
        25_f,
        25_f,
        1920_f / 2_f,
        1080_f / 2_f,
        0}};

    // At origin, no rotation.
    // Camera is is facing the x-axis
    auto originPose = tau::Pose<float>();

    auto projectionFromWorld =
        tau::Projection<float>(intrinsics, originPose);

    auto pixel = Pixel{1920_f / 2_f, 1080_f / 2_f};
    auto lineFromWorld = projectionFromWorld.GetLine_m(pixel);
    auto expected = tau::Line3d<float>({0_f, 0_f, 0_f}, {1_f, 0_f, 0_f});

    REQUIRE(lineFromWorld.IsColinear(expected));

    auto lowerPixel = Pixel{1920_f / 2_f, 1079};

    std::cout << "lowerPixel: "
        << projectionFromWorld.GetLine(lowerPixel).GetAngleAboutY()
        << std::endl;

    auto upperPixel = Pixel{1920_f / 2_f, 0};

    std::cout << "upperPixel: "
        << projectionFromWorld.GetLine(upperPixel).GetAngleAboutY()
        << std::endl;

    auto leftPixel = Pixel{0_f, 1080_f / 2_f};

    std::cout << "leftPixel: "
        << projectionFromWorld.GetLine_m(leftPixel).GetAngleAboutZ()
        << std::endl;

    auto rightPixel = Pixel{1919_f, 1080_f / 2_f};

    std::cout << "rightPixel: "
        << projectionFromWorld.GetLine_m(rightPixel).GetAngleAboutZ()
        << std::endl;

    // TODO Add a REQUIRE
}


TEST_CASE("Pose shifted projection", "[pose]")
{
    tau::Intrinsics<float> intrinsics{{
        10_f,
        25_f,
        25_f,
        1920_f / 2_f,
        1080_f / 2_f,
        0_f}};

    // At origin, no rotation.
    // Camera is is facing the x-axis
    auto pose = tau::Pose<float>();
    pose.y_m = 2_f;

    auto projection = tau::Projection<float>(intrinsics, pose);

    auto pixel = Pixel{1920_f / 2_f, 1080_f / 2_f};
    auto lineFromPose = projection.GetLine_m(pixel);
    auto expected = tau::Line3d<float>({0_f, 2_f, 0_f}, {1_f, 0_f, 0_f});
    std::cout << "lineFromPose: " << lineFromPose << std::endl;
    std::cout << "expected: " << expected << std::endl;

    REQUIRE(lineFromPose.IsColinear(expected));
}


TEST_CASE("Pose rotated projection", "[pose]")
{
    tau::Intrinsics<float> intrinsics{{
        10_f,
        25_f,
        25_f,
        1920_f / 2_f,
        1080_f / 2_f,
        0}};

    auto pose = tau::Pose<float>(
        {
            45_f,
            0_f,
            0_f},
        0_f,
        0_f,
        0_f);

    auto projection = tau::Projection<float>(intrinsics, pose);

    auto pixel = Pixel{1920_f / 2_f, 1080_f / 2_f};
    auto lineFromPose = projection.GetLine(pixel);
    auto expectedAngle = 45_f;

    REQUIRE(lineFromPose.GetAngleAboutZ() == Approx(expectedAngle));
}
