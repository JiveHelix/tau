#include "tau/rotation.h"


namespace tau
{


const std::array<std::string, 3>
AxisOrder::axisNames{"roll", "pitch", "yaw"};


std::string AxisOrderConverter::ToString(const AxisOrder &axisOrder)
{
    std::ostringstream outputStream;
    axisOrder.ToStream(outputStream);
    return outputStream.str();
}


AxisOrder AxisOrderConverter::ToValue(const std::string &asString)
{
    auto axisNames = jive::strings::Split(asString, '-');

    if (axisNames.size() != 3)
    {
        throw RotationError("Expected 3 axis names");
    }

    std::vector<size_t> axes;

    auto begin = std::begin(AxisOrder::axisNames);
    auto end = std::end(AxisOrder::axisNames);

    for (auto &name: axisNames)
    {
        auto found = std::find(begin, end, name);

        if (found == end)
        {
            throw RotationError("Unexpected axis name " + name);
        }

        axes.push_back(static_cast<size_t>(std::distance(begin, found)));
    }

    return {axes.at(0), axes.at(1), axes.at(2)};
}


std::ostream & AxisOrder::ToStream(std::ostream &outputStream) const
{
    return outputStream << axisNames.at(this->first) << "-"
        << axisNames.at(this->second) << "-"
        << axisNames.at(this->third);
}


std::ostream & operator<<(
    std::ostream &outputStream,
    const AxisOrder &axisOrder)
{
    return axisOrder.ToStream(outputStream);
}


template struct RotationAnglesModel<float>;
template struct RotationAnglesModel<double>;


} // namespace tau


template struct pex::Group
    <
        tau::RotationAnglesFields,
        tau::RotationAnglesTemplate<float>::template Template,
        tau::RotationAngles<float>
    >;

template struct pex::Group
    <
        tau::RotationAnglesFields,
        tau::RotationAnglesTemplate<double>::template Template,
        tau::RotationAngles<double>
    >;
