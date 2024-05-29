#pragma once


#include <vector>
#include "tau/vector3d.h"


namespace tau
{


Point3d<double> ImportPoint(std::istream &input);

std::vector<Point3d<double>> ImportDxfPoints(const std::string &fileName);


}
