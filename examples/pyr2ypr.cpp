#include <jive/to_float.h>
#include <tau/rotation.h>
#include <iostream>


int main(int count, char **args)
{
    if (count != 4)
    {
        std::cerr << "Usage: " << args[0] << "pitch yaw roll" << std::endl;
        return -1;
    }

    using namespace tau;

    RotationAngles<double> pitchYawRoll(
        jive::ToFloat<double>(args[1]),
        jive::ToFloat<double>(args[2]),
        jive::ToFloat<double>(args[3]),
        AxisOrder{1, 2, 0});

    std::cout << "entered: " << pitchYawRoll << std::endl;

    RotationAngles<double> yawPitchRoll(
        pitchYawRoll.GetRotation(),
        AxisOrder{2, 1, 0});

    if (!yawPitchRoll.GetRotation().isApprox(pitchYawRoll.GetRotation()))
    {
        std::cerr << "Failed to convert axis order to yaw-pitch-roll"
            << std::endl;

        return -1;
    }

    std::cout << "converted: " << yawPitchRoll << std::endl;

    std::cout << "pitch-yaw-roll matrix:\n" << pitchYawRoll.GetRotation() << std::endl;

    std::cout << "yaw-pitch-roll matrix:\n" << yawPitchRoll.GetRotation() << std::endl;

    RotationAngles<double> backToPitchYawRoll(
        yawPitchRoll.GetRotation(),
        AxisOrder{1, 2, 0});

    std::cout << "backToPitchYawRoll: " << backToPitchYawRoll << std::endl;

    return 0;
}
