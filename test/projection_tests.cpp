#include <catch2/catch.hpp>

#include <tau/literals.h>
#include <tau/projection.h>


using namespace tau::literals;


TEST_CASE("Camera to World to Camera round trip.", "[projection]")
{
    auto intrinsics = tau::Intrinsics<float>{};

    tau::Pose<float> pose(
        {
            -10_f, // rotation about z
            13.0_f, //  rotation about y
            0.0_f,
            tau::AxisOrder{2, 1, 0}}, // rotation about x
        1.0_f, // x
        -3_f, // y
        2_f); // z

    tau::Projection projection(intrinsics, pose);
    auto pixel = tau::Point2d<float>{1547_f, 567_f};
    auto world = projection.GetLine_m(pixel).ScaleToPoint(5);
    auto roundTrip = projection.WorldToCamera(world);

    REQUIRE(roundTrip.isApprox(pixel.GetHomogeneous()));
}


TEST_CASE("Trivial world to Camera", "[projection]")
{
    auto intrinsics = tau::Intrinsics<float>{};
    auto pose = tau::Pose<float>{};

    tau::Vector3<float> world(5.0_f, 0.0_f, 0.0_f);
    tau::Projection projection(intrinsics, pose);
    tau::Vector3<float> pixel = projection.WorldToCamera(world);

    REQUIRE(pixel(0) == Approx(960.0_f));
    REQUIRE(pixel(1) == Approx(540.0_f));
    REQUIRE(pixel(2) == Approx(1.0_f));
}


TEST_CASE("Shifted world to Camera", "[projection]")
{
    auto intrinsics = tau::Intrinsics<float>{};

    auto noShift = tau::Pose<float>{};

    auto pose = tau::Pose<float>{};
    pose.point_m.y = 1.0_f;

    tau::Vector3<float> world(5.0_f, 0.0_f, 0.0_f);
    tau::Projection noShiftProjection(intrinsics, noShift);
    tau::Projection projection(intrinsics, pose);

    tau::Vector3<float> noShiftPixel =
        noShiftProjection.WorldToCamera(world);

    tau::Vector3<float> pixel = projection.WorldToCamera(world);

    std::cout << "Shifted world to camera: " << pixel << std::endl;
    std::cout << "No Shift world to camera: " << noShiftPixel << std::endl;

    // TODO: Add a REQUIRE
}
