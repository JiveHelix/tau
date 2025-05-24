#include "tau/dxf.h"
#include <fstream>
#include <jive/to_integer.h>
#include <jive/to_float.h>


namespace tau
{


Point3d<double> ImportPoint(std::istream &input)
{
    Point3d<double> result;
    std::string label;
    std::string value;

    for (int i = 0; i < 3; ++i)
    {
        std::getline(input, label, '\n');
        std::getline(input, value, '\n');

        label = jive::strings::Trim(label);
        value = jive::strings::Trim(value);

        auto key = jive::ToInteger<int>(label);

        switch (key)
        {
            case 10:
                result.x = jive::ToFloat<double>(value);
                break;

            case 20:
                result.y = jive::ToFloat<double>(value);
                break;

            case 30:
                result.z = jive::ToFloat<double>(value);
                break;
            
            default:
                throw std::runtime_error("Unexpected label");
        }
    }

    return result;
}


std::vector<Point3d<double>> ImportDxfPoints(const std::string &fileName)
{
    std::vector<Point3d<double>> result;

    auto input = std::ifstream(fileName);

    std::string line;

    while (input)
    {
        std::getline(input, line, '\n');

        if (!input)
        {
            break;
        }

        if (line.starts_with("AcDbPoint"))
        {
            result.push_back(ImportPoint(input));
        }
    }

    return result;
}


} // end namespace tau
