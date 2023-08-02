#include <catch2/catch.hpp>
#include <Eigen/Dense>


#include <tau/literals.h>
#include <tau/intrinsics.h>


using namespace tau::literals;


TEST_CASE("Meters To Pixel", "[intrinsics]")
{
    using Intrinsics = tau::Intrinsics<double>;
    Intrinsics intrinsics{};

    intrinsics.pixelSize_um = 10;

    double metersToPixel =
        intrinsics.pixelSize_um * Intrinsics::metersPerMicron;

    REQUIRE(intrinsics.MetersToPixel(0.0_d) == 0_d);
    REQUIRE(intrinsics.MetersToPixel(2.0_d) == 2.0_d / metersToPixel);
    REQUIRE(intrinsics.MetersToPixel(-2.0_d) == -2.0_d / metersToPixel);
}

TEST_CASE("Pixels To Meters", "[intrinsics]")
{
    using Intrinsics = tau::Intrinsics<double>;
    Intrinsics intrinsics{};

    intrinsics.pixelSize_um = 10;

    double metersToPixel =
        intrinsics.pixelSize_um * Intrinsics::metersPerMicron;

    REQUIRE(intrinsics.PixelsToMeters(0.0_d) == 0_d);
    REQUIRE(intrinsics.PixelsToMeters(2.0_d) == 2.0_d * metersToPixel);
    REQUIRE(intrinsics.PixelsToMeters(-2.0_d) == -2.0_d * metersToPixel);
}
