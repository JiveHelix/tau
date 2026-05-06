#include <iostream>
#include <tau/dxf.h>
#include <tau/csv.h>
#include <filesystem>



int main(int count, char **args)
{
    if (count != 2)
    {
        std::cerr << "Usage: " << args[0] << " file.dxf" << std::endl;
        return 1;
    }

    auto points = tau::ImportDxfPoints(args[1]);

    std::cout << "Found " << points.size() << " points." << std::endl;

    tau::Csv csvOutput({"Point", "X", "Y", "Z"});

    for (size_t i = 0; i < points.size(); ++i)
    {
        const auto &point = points[i];
        std::cout << i + 1 << ": " << fields::Describe(point) << std::endl;
        auto index = Eigen::Index(i);

        csvOutput.AssignCell(index, 0, i + 1);
        csvOutput.AssignCell(index, 1, point.x);
        csvOutput.AssignCell(index, 2, point.y);
        csvOutput.AssignCell(index, 3, point.z);
    }

    std::filesystem::path inputPath(args[1]);
    csvOutput.ToFile(inputPath.replace_extension(".csv"));

    return 0;
}
