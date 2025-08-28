#include <tau/pixel_origin.h>


namespace tau
{


std::map<PixelOrigin, std::string_view> sensorOriginStrings{
    {PixelOrigin::bottomLeft, "bottom-left"},
    {PixelOrigin::topLeft, "top-left"},
    {PixelOrigin::bottomRight, "bottom-right"},
    {PixelOrigin::topRight, "top-right"}};


std::vector<PixelOrigin> PixelOriginChoices::GetChoices()
{
    return {
        PixelOrigin::bottomLeft,
        PixelOrigin::topLeft,
        PixelOrigin::bottomRight,
        PixelOrigin::topRight};
}


std::unordered_map<std::string_view, PixelOrigin> GetOriginsByString()
{
    std::unordered_map<std::string_view, PixelOrigin> result;

    for (auto [key, value]: sensorOriginStrings)
    {
        result[value] = key;
    }

    return result;
}


std::string ToString(PixelOrigin pixelOrigin)
{
    return std::string(sensorOriginStrings.at(pixelOrigin));
}


PixelOrigin ToValue(fields::Tag<PixelOrigin>, std::string_view asString)
{
    static const auto originsByString = GetOriginsByString();

    return originsByString.at(asString);
}


std::string PixelOriginConverter::ToString(
    PixelOrigin pixelOrigin)
{
    return ::tau::ToString(pixelOrigin);
}


PixelOrigin PixelOriginConverter::ToValue(const std::string &asString)
{
    return ::tau::ToValue(fields::Tag<PixelOrigin>{}, asString);
}


std::ostream & operator<<(std::ostream &output, PixelOrigin value)
{
    return output << PixelOriginConverter::ToString(value);
}


std::string GetPixelOriginsString()
{
    auto origins = tau::PixelOriginChoices::GetChoices();

    std::vector<std::string> originStrings;

    for (auto origin: origins)
    {
        originStrings.push_back(tau::ToString(origin));
    }

    return jive::strings::Join(
        originStrings.begin(),
        originStrings.end(),
        ", ");
}


} // end namespace tau
