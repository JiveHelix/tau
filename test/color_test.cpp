#include <catch2/catch.hpp>

#include <jive/range.h>
#include "tau/planar.h"
#include "tau/color.h"


static constexpr size_t channels = 3;
constexpr size_t stride = 4;



template<typename Hsv>
Hsv MakeHsvInputs()
{
    Hsv hsv(4, 4);

    tau::GetHue(hsv) <<
        0, 0, 0, 120,
        240, 60, 180, 300,
        0, 0, 0, 60,
        120, 300, 180, 240;

    tau::GetSaturation(hsv) <<
        0, 0, 1, 1,
        1, 1, 1, 1,
        0, 0, 1, 1,
        1, 1, 1, 1;

    tau::GetValue(hsv) <<
        0, 1, 1, 1,
        1, 1, 1, 1,
        0.75, 0.5, 0.5, 0.5,
        0.5, 0.5, 0.5, 0.5;

    return hsv;
}


template<typename Rgb>
Rgb MakeRgbOutputs()
{
    Rgb rgb(4, 4);

    tau::GetRed(rgb) <<
        0, 255, 255, 0,
        0, 255, 0, 255,
        191, 128, 128, 128,
        0, 128, 0, 0;

    tau::GetGreen(rgb) <<
        0, 255, 0, 255,
        0, 255, 255, 0,
        191, 128, 0, 128,
        128, 0, 128, 0;

    tau::GetBlue(rgb) <<
        0, 255, 0, 0,
        255, 0, 255, 255,
        191, 128, 0, 0,
        0, 128, 128, 128;

    return rgb;
}


template<typename Hsv>
Hsv MakeHsvOutputs()
{
    Hsv hsv(4, 4);

    tau::GetHue(hsv) <<
        0, 0, 0, 120,
        240, 60, 180, 300,
        0, 0, 0, 60,
        120, 300, 180, 240;

    tau::GetSaturation(hsv) <<
        0, 0, 1, 1,
        1, 1, 1, 1,
        0, 0, 1, 1,
        1, 1, 1, 1;

    tau::GetValue(hsv) <<
        0, 1, 1, 1,
        1, 1, 1, 1,
        0.749, 0.502, 0.502, 0.502,
        0.502, 0.502, 0.502, 0.502;

    return hsv;
}


TEST_CASE("Convert column-major HSV to RGB 8", "[color]")
{
    using HsvT = tau::Planar<channels, double, Eigen::Dynamic, Eigen::Dynamic>;
    using RgbT = tau::Planar<channels, uint8_t, Eigen::Dynamic, Eigen::Dynamic>;

    auto hsv = MakeHsvInputs<HsvT>();
    auto expected = MakeRgbOutputs<RgbT>();

    auto result = tau::HsvToRgb<uint8_t>(hsv);

    auto interleaved = result.GetInterleaved();

    REQUIRE(interleaved.rows() == channels);
    REQUIRE(interleaved.cols() == 16);

    uint8_t *data = interleaved.data();

    for (auto j: jive::Range<Eigen::Index>(0, 4))
    {
        for (auto i: jive::Range<Eigen::Index>(0, 4))
        {
            REQUIRE(GetRed(result)(i, j) == GetRed(expected)(i, j));
            REQUIRE(GetGreen(result)(i, j) == GetGreen(expected)(i, j));
            REQUIRE(GetBlue(result)(i, j) == GetBlue(expected)(i, j));

            for (auto k: jive::Range<size_t>(0, channels))
            {
                auto rowIndex = static_cast<size_t>(j) * stride + static_cast<size_t>(i);

                auto colorIndex = rowIndex * channels + k;

                uint8_t *color = data + colorIndex;

                REQUIRE(static_cast<uint16_t>(*color) ==
                    static_cast<uint16_t>(expected.planes[k](i, j)));
            }
        }
    }
}


TEST_CASE("Convert row-major HSV to RGB 8", "[color]")
{
    using HsvT =
        tau::Planar
        <
            channels,
            double,
            Eigen::Dynamic,
            Eigen::Dynamic,
            Eigen::RowMajor
        >;

    using RgbT =
        tau::Planar
        <
            channels,
            uint8_t,
            Eigen::Dynamic,
            Eigen::Dynamic,
            Eigen::RowMajor
        >;

    auto hsv = MakeHsvInputs<HsvT>();
    auto expected = MakeRgbOutputs<RgbT>();

    auto result = tau::HsvToRgb<uint8_t>(hsv);

    auto interleaved = result.GetInterleaved();

    REQUIRE(interleaved.rows() == 16);
    REQUIRE(interleaved.cols() == channels);

    uint8_t *data = interleaved.data();

    for (auto i: jive::Range<Eigen::Index>(0, 4))
    {
        for (auto j: jive::Range<Eigen::Index>(0, 4))
        {
            REQUIRE(GetRed(result)(i, j) == GetRed(expected)(i, j));
            REQUIRE(GetGreen(result)(i, j) == GetGreen(expected)(i, j));
            REQUIRE(GetBlue(result)(i, j) == GetBlue(expected)(i, j));

            for (auto k: jive::Range<size_t>(0, channels))
            {
                auto columnIndex =
                    static_cast<size_t>(i) * stride + static_cast<size_t>(j);

                auto colorIndex = columnIndex * channels + k;

                uint8_t *color = data + colorIndex;

                REQUIRE(static_cast<uint16_t>(*color) ==
                    static_cast<uint16_t>(expected.planes[k](i, j)));
            }
        }
    }
}


TEST_CASE("Convert single Planar HSV to RGB 8", "[color]")
{
    using HsvMatrix =
        tau::Planar<channels, double, Eigen::Dynamic, Eigen::Dynamic>;

    using RgbMatrix =
        tau::Planar<channels, uint8_t, Eigen::Dynamic, Eigen::Dynamic>;

    auto hsv = MakeHsvInputs<HsvMatrix>();
    auto expected = MakeRgbOutputs<RgbMatrix>();

    for (auto j: jive::Range<Eigen::Index>(0, 4))
    {
        for (auto i: jive::Range<Eigen::Index>(0, 4))
        {
            auto value = hsv(i, j);
            auto result = tau::HsvToRgb<uint8_t>(value);
            auto interleaved = result.GetInterleaved();

            REQUIRE(interleaved.rows() == channels);
            REQUIRE(interleaved.cols() == 1);

            REQUIRE(GetRed(result)(0, 0) == GetRed(expected)(i, j));
            REQUIRE(GetGreen(result)(0, 0) == GetGreen(expected)(i, j));
            REQUIRE(GetBlue(result)(0, 0) == GetBlue(expected)(i, j));
        }
    }
}


TEST_CASE("Convert single Vector HSV to RGB 8", "[color]")
{
    using HsvMatrix =
        tau::Planar<channels, double, Eigen::Dynamic, Eigen::Dynamic>;

    using RgbMatrix =
        tau::Planar<channels, uint8_t, Eigen::Dynamic, Eigen::Dynamic>;

    auto hsvInputs = MakeHsvInputs<HsvMatrix>();
    auto expected = MakeRgbOutputs<RgbMatrix>();
    auto hsvOutputs = MakeHsvOutputs<HsvMatrix>();

    for (auto j: jive::Range<Eigen::Index>(0, 4))
    {
        for (auto i: jive::Range<Eigen::Index>(0, 4))
        {
            auto value = hsvInputs.GetVector(i, j);
            auto rgb = tau::HsvToRgb<uint8_t>(value);
            auto backToHsv = tau::RgbToHsv<double>(rgb);

            REQUIRE(rgb == expected.GetVector(i, j));
            REQUIRE(backToHsv == hsvOutputs.GetVector(i, j));
        }
    }
}


TEST_CASE("Convert RGB 8 to HSV", "[color]")
{
    using HsvT = tau::Planar<channels, double, Eigen::Dynamic, Eigen::Dynamic>;
    using RgbT = tau::Planar<channels, uint8_t, Eigen::Dynamic, Eigen::Dynamic>;

    auto rgb = MakeRgbOutputs<RgbT>();
    auto expected = MakeHsvOutputs<HsvT>();

    auto hsv = tau::RgbToHsv<double>(rgb);

    auto interleaved = hsv.GetInterleaved();

    REQUIRE(interleaved.rows() == channels);
    REQUIRE(interleaved.cols() == 16);

    for (auto j: jive::Range<Eigen::Index>(0, 4))
    {
        for (auto i: jive::Range<Eigen::Index>(0, 4))
        {
            REQUIRE(GetHue(hsv)(i, j) == GetHue(expected)(i, j));
            REQUIRE(GetSaturation(hsv)(i, j) == GetSaturation(expected)(i, j));
            REQUIRE(GetValue(hsv)(i, j) == GetValue(expected)(i, j));
        }
    }
}


TEST_CASE("HasAlpha", "[color]")
{
    STATIC_REQUIRE(tau::HasAlpha<tau::Rgba<uint8_t>>);
    STATIC_REQUIRE(!tau::HasAlpha<tau::Rgb<uint8_t>>);

    STATIC_REQUIRE(tau::HasAlpha<tau::Hsva<double>>);
    STATIC_REQUIRE(!tau::HasAlpha<tau::Hsv<double>>);
}
