#include <catch2/catch.hpp>

#include <jive/equal.h>
#include <jive/testing/generator_limits.h>
#include <iomanip>

#include <tau/vector3d.h>
#include <tau/angles.h>
#include <tau/rotation.h>


static constexpr double tolerance = 1e-12;


TEST_CASE("Compute angle between 3d vectors.", "[vector2d]")
{
    double yaw1_deg = 30.0;
    double yaw2_deg = 77.0;
    double pitch_deg = 0.0;

    double yaw1_rad = tau::ToRadians(yaw1_deg);
    double yaw2_rad = tau::ToRadians(yaw2_deg);
    double pitch_rad = tau::ToRadians(pitch_deg);

    auto z = std::sin(pitch_rad);

    auto x1 = std::cos(yaw1_rad);
    auto y1 = std::sin(yaw1_rad);

    auto x2 = std::cos(yaw2_rad);
    auto y2 = std::sin(yaw2_rad);

    tau::Vector3d<double> first(x1, y1, z);
    tau::Vector3d<double> second(x2, y2, z);

    auto expected = jive::Roughly(yaw2_deg - yaw1_deg, tolerance);

    REQUIRE(expected == first.GetAngle_deg(second));

    // Expect the angle between the vectors to remain unchanged when they are
    // rotated together.
    auto values = GENERATE(
        take(
            4,
            chunk(
                3,
                random(-180.0, 180.0))));

    auto rotation = tau::RotationAngles<double>(
        values.at(0),
        values.at(1),
        values.at(2));

    auto rotationMatrix = rotation.GetRotation();

    auto firstRotated =
        tau::Vector3d<double>(rotationMatrix * first.ToEigen());

    auto secondRotated =
        tau::Vector3d<double>(rotationMatrix * second.ToEigen());

    REQUIRE(expected == firstRotated.GetAngle_deg(secondRotated));
}
