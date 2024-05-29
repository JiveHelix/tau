#include <iostream>
#include <tau/dxf.h>



int main(int count, char **args)
{
    if (count != 2)
    {
        std::cerr << "Usage: " << args[0] << " file.dxf" << std::endl;
        return 1;
    }

    auto points = tau::ImportDxfPoints(args[1]);

    std::cout << "Found " << points.size() << " points." << std::endl;

    for (size_t i = 0; i < points.size(); ++i)
    {
        std::cout << i + 1 << ": " << fields::Describe(points[i]) << std::endl;
    }

    return 0;
}
