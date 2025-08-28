#pragma once


#include <tau/rotation.h>


namespace tau
{


/**
 ** The position of pixel (0, 0) when looking at the sensor.
 **
 **
 ** Top-left                      Top-right
 **
 **  z (into screen/sensor                z (out of screen/sensor)
 **    *-----> x                   x <-----•
 **    |                                   |
 **    |                                   |
 **    |                                   |
 **  y v                                   v y
 **
 **
 ** Bottom-left                   Bottom-right
 **
 **    y                                   y
 **    ^                                   ^
 **    |                                   |
 **    |                                   |
 **    |                                   |
 **    •-----> x                  x <------*
 **  z (out of screen/sensor)             z (into screen/sensor)
 **
 **/
enum class PixelOrigin
{
    bottomLeft,
    topLeft,
    bottomRight,
    topRight
};


std::string ToString(PixelOrigin);

PixelOrigin ToValue(fields::Tag<PixelOrigin>, std::string_view asString);


struct PixelOriginConverter
{
    static std::string ToString(PixelOrigin);

    static PixelOrigin ToValue(const std::string &asString);
};


struct PixelOriginChoices
{
    using Type = PixelOrigin;
    static std::vector<PixelOrigin> GetChoices();
    using Converter = PixelOriginConverter;
};

using PixelOriginSelect = pex::MakeSelect<PixelOriginChoices>;

std::string GetPixelOriginsString();

std::ostream & operator<<(std::ostream &, PixelOrigin);


/* Convert between sensor and world coordinates
 * World Coordinate System
 *      X positive Forward (roll axis)
 *      Y positive to left (pitch axis)
 *      Z up (yaw axis)
 */
template<typename T>
RotationMatrix<T> SensorRelativeToWorld(PixelOrigin pixelOrigin)
{
    // To move from world coordinates to sensor coordinates:

    switch (pixelOrigin)
    {
        case PixelOrigin::bottomLeft:
            return MakeYawPitchRoll(
                static_cast<T>(90),
                static_cast<T>(0),
                static_cast<T>(90));

        case PixelOrigin::topLeft:
            return MakeYawPitchRoll(
                static_cast<T>(90),
                static_cast<T>(0),
                static_cast<T>(-90));

        case PixelOrigin::bottomRight:
            return MakeYawPitchRoll(
                static_cast<T>(-90),
                static_cast<T>(0),
                static_cast<T>(90));

        case PixelOrigin::topRight:
            return MakeYawPitchRoll(
                static_cast<T>(-90),
                static_cast<T>(0),
                static_cast<T>(-90));

        default:
            throw std::logic_error("Unsupported pixel origin");
    }
}

template<typename T>
RotationMatrix<T> WorldRelativeToSensor(PixelOrigin pixelOrigin)
{
    // To move from sensor coordinates to world coordinates:

    switch (pixelOrigin)
    {
        case PixelOrigin::bottomLeft:
            return MakeYawPitchRoll(
                static_cast<T>(-90),
                static_cast<T>(-90),
                static_cast<T>(0));

        case PixelOrigin::topLeft:
            return MakeYawPitchRoll(
                static_cast<T>(-90),
                static_cast<T>(90),
                static_cast<T>(0));

        case PixelOrigin::bottomRight:
            return MakeYawPitchRoll(
                static_cast<T>(90),
                static_cast<T>(90),
                static_cast<T>(0));

        case PixelOrigin::topRight:
            return MakeYawPitchRoll(
                static_cast<T>(90),
                static_cast<T>(-90),
                static_cast<T>(0));

        default:
            throw std::logic_error("Unsupported pixel origin");
    }
}


} // end namespace tau
