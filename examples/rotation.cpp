#include <tau/rotation.h>
#include <iostream>


int main()
{
    using namespace tau;

    RotationAngles<double> yawPitchRoll(20, -30, 45);
    std::cout << "yawPitchRoll:\n" << yawPitchRoll << std::endl;
    auto r = yawPitchRoll.GetRotation();
    std::cout << "r:\n" << r << std::endl;
    RotationAngles<double> recovered(r, yawPitchRoll.axisOrder);
    std::cout << "recovered:\n" << recovered << std::endl;


    auto test = MakePitchYawRoll<double>(-30, 20, 0);
    RotationAngles<double> asPitchYawRoll(-30, 20, 0, {1, 2, 0});
    RotationAngles<double> converted(asPitchYawRoll.GetRotation(), {2, 1, 0});

    std::cout << "test:\n" << test << std::endl;
    std::cout << "asPitchYawRoll:\n" << asPitchYawRoll << std::endl;
    std::cout << "converted:\n" << converted << std::endl;

    std::cout << "converted.GetRotation():\n" << converted.GetRotation()
        << std::endl;

    return 0;
}
