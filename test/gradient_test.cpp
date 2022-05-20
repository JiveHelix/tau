#include <catch2/catch.hpp>

#include "tau/color_maps/gradient.h"
#include "tau/color.h"


TEST_CASE("Create gradient along value", "[color]")
{
    auto green = tau::Vector<3, uint8_t>(100, 180, 85);
    auto first = tau::RgbToHsv<double>(green);
    auto last = first;

    first(tau::index::value) -= 0.1;

    auto rgbGradient =
        tau::gradient::MakeColormap<uint8_t>(10, first, last);
    
    REQUIRE(rgbGradient.rows() == 10);
    std::cout << rgbGradient << std::endl;

    REQUIRE(rgbGradient.row(9) == green.transpose());
}
