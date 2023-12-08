#include <catch2/catch.hpp>

#include <jive/testing/generator_limits.h>
#include <jive/equal.h>
#include <tau/eigen.h>
#include <tau/random.h>

#include <tau/vector3d.h>
#include <tau/rotation.h>


template<typename T>
struct Constants
{
    static constexpr auto _0 = static_cast<T>(0);
    static constexpr auto _1 = static_cast<T>(1);
    static constexpr auto _90 = static_cast<T>(90);
    static constexpr auto _180 = static_cast<T>(180);
    static constexpr auto _360 = static_cast<T>(360);
};


TEMPLATE_TEST_CASE(
    "Rotations have determinant of 1.0",
    "[rotation]",
    float,
    double)
{
    using C = Constants<TestType>;
    static constexpr auto chunkSize = 5;
    static constexpr auto testCount = 100;
    static constexpr auto lowerBound = static_cast<TestType>(C::_0);
    static constexpr auto upperBound = static_cast<TestType>(C::_360);

    auto values = GENERATE(
        take(testCount, chunk(chunkSize, random(lowerBound, upperBound))));

    // By definition, the determinant of a rotation matrix about a single axis
    // will always be sin^2(angle) + cos^2(angle) == 1
    using namespace tau;

    for (auto value: values)
    {
        REQUIRE(
            AboutX(value).determinant()
            == Approx(C::_1));

        REQUIRE(
            AboutY(value).determinant()
            == Approx(C::_1));

        REQUIRE(
            AboutZ(value).determinant()
            == Approx(C::_1));
    }
}


TEMPLATE_TEST_CASE("Rotation about X", "[rotation]", float, double)
{
    using C = Constants<TestType>;
    auto y = tau::Vector<3>(C::_0, C::_1, C::_0);
    tau::Vector3<TestType> rotated = tau::AboutX(C::_90) * y;
    auto expected = tau::Vector<3>(C::_0, C::_0, C::_1);

    REQUIRE(expected.isApprox(rotated));
}

TEMPLATE_TEST_CASE("Rotation about Y", "[rotation]", float, double)
{
    using C = Constants<TestType>;
    auto x = tau::Vector<3>(C::_1, C::_0, C::_0);
    tau::Vector3<TestType> rotated = tau::AboutY(C::_90) * x;
    auto expected = tau::Vector<3>(C::_0, C::_0, -C::_1);

    REQUIRE(expected.isApprox(rotated));
}

TEMPLATE_TEST_CASE("Rotation about Z", "[rotation]", float, double)
{
    using C = Constants<TestType>;
    auto x = tau::Vector<3>(C::_1, C::_0, C::_0);
    tau::Vector3<TestType> rotated = tau::AboutZ(C::_90) * x;
    auto expected = tau::Vector<3>(C::_0, C::_1, C::_0);

    REQUIRE(expected.isApprox(rotated));
}


TEMPLATE_TEST_CASE(
    "Tait-Bryan Angles",
    "[rotation]",
    float,
    double)
{
    using C = Constants<TestType>;

    auto seed = GENERATE(
        take(16, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    tau::UniformRandom<TestType> uniformRandom{seed};
    uniformRandom.SetRange(-C::_180, C::_180);

    using tau::Vector3;

    auto thetaX = uniformRandom();
    auto thetaY = uniformRandom();
    auto thetaZ = uniformRandom();

    Vector3<TestType> degrees{thetaZ, thetaY, thetaX};

    using Matrix = Eigen::Matrix<TestType, 3, 3>;

    auto rotationX = tau::AboutX(thetaX);
    auto rotationY = tau::AboutY(thetaY);
    auto rotationZ = tau::AboutZ(thetaZ);

    Matrix rotation = rotationZ * rotationY * rotationX;

    REQUIRE(
        rotation.isApprox(
            tau::MakeYawPitchRoll(thetaZ, thetaY, thetaX)));

    Vector3<TestType> radians = tau::ToRadians(degrees);
    Vector3<TestType> c = radians.array().cos();
    Vector3<TestType> s = radians.array().sin();

    auto c1 = c(0);
    auto c2 = c(1);
    auto c3 = c(2);
    auto s1 = s(0);
    auto s2 = s(1);
    auto s3 = s(2);

    auto taitBryanZYX = Matrix
        {
            {
                c1 * c2,
                c1 * s2 * s3 - c3 * s1,
                s1 * s3 + c1 * c3 * s2},
            {
                c2 * s1,
                c1 * c3 + s1 * s2 * s3,
                c3 * s1 * s2 - c1 * s3},
            {
                -s2,
                c2 * s3,
                c2 * c3}};

    REQUIRE(rotation.isApprox(taitBryanZYX));
}


TEMPLATE_TEST_CASE(
    "Rotation order equivalence",
    "[rotation]",
    float,
    double)
{
    using C = Constants<TestType>;

    auto seed = GENERATE(
        take(16, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    tau::UniformRandom<TestType> uniformRandom{seed};
    uniformRandom.SetRange(-C::_180, C::_180);

    using tau::Vector3;

    auto thetaX = uniformRandom();
    auto thetaY = uniformRandom();
    auto thetaZ = uniformRandom();

    using Matrix = Eigen::Matrix<TestType, 3, 3>;

    auto rotationX = tau::AboutX(thetaX);
    auto rotationY = tau::AboutY(thetaY);
    auto rotationZ = tau::AboutZ(thetaZ);

    Matrix rotation = rotationZ * rotationY * rotationX;

    // Make a random vector
    uniformRandom.SetRange(-C::_1, C::_1);
    Vector3<TestType> p{uniformRandom(), uniformRandom(), uniformRandom()};

    // Verify that the matrix that combines all three rotations yields the
    // equivalent vector to breaking the rotation down into steps for each
    // axis.
    Vector3<TestType> stepped = rotationX * p;
    stepped = rotationY * stepped;
    stepped = rotationZ * stepped;

    Vector3<TestType> rotated = rotation * p;

    REQUIRE(rotated.isApprox(stepped));

    Vector3<TestType> steppedSwapped = p.transpose() * rotationZ;
    steppedSwapped = steppedSwapped.transpose() * rotationY;
    steppedSwapped = steppedSwapped.transpose() * rotationX;

    Vector3<TestType> swapped = p.transpose() * rotation;
    REQUIRE(swapped.isApprox(steppedSwapped));
}

/*
TEMPLATE_TEST_CASE(
    "Swap rotation order",
    "[rotation]",
    float,
    double)
{
    using C = Constants<TestType>;

    auto seed = GENERATE(
        take(16, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    tau::UniformRandom<TestType> uniformRandom{seed};
    uniformRandom.SetRange(-C::_90, C::_90);

    using TestAngles = tau::RotationAngles<TestType>;

    auto thetaX = uniformRandom();
    auto thetaY = uniformRandom();
    auto thetaZ = uniformRandom();

    auto yawPitchRoll =
        TestAngles(thetaZ, thetaY, thetaX, tau::AxisOrder{2, 1, 0});

    auto yawPitchRollMatrix = yawPitchRoll.GetRotation();

    auto pitchYawRoll =
        TestAngles(yawPitchRollMatrix, tau::AxisOrder{1, 2, 0});

    auto recoveredYawPitchRoll =
        TestAngles(pitchYawRoll.GetRotation(), tau::AxisOrder{2, 1, 0});

    REQUIRE(recoveredYawPitchRoll.yaw == Approx(thetaZ));
    REQUIRE(recoveredYawPitchRoll.pitch == Approx(thetaY));
    REQUIRE(recoveredYawPitchRoll.roll == Approx(thetaX));
}
*/


TEST_CASE("Simple swap rotation order", "[rotation]")
{
    auto thetaX = 0.0;
    auto thetaY = -30.0;
    auto thetaZ = 20.0;

    using TestAngles = tau::RotationAngles<double>;

    auto yawPitchRoll =
        TestAngles(thetaZ, thetaY, thetaX, tau::AxisOrder{2, 1, 0});

    auto yawPitchRollMatrix = yawPitchRoll.GetRotation();

    auto pitchYawRoll =
        TestAngles(yawPitchRollMatrix, tau::AxisOrder{1, 2, 0});

    auto recoveredYawPitchRoll =
        TestAngles(pitchYawRoll.GetRotation(), tau::AxisOrder{2, 1, 0});

    REQUIRE(jive::Roughly(recoveredYawPitchRoll.yaw, 1e-4) == thetaZ);
    REQUIRE(jive::Roughly(recoveredYawPitchRoll.pitch, 1e-4) == thetaY);
    REQUIRE(jive::Roughly(recoveredYawPitchRoll.roll, 1e-4) == thetaX);
}


TEST_CASE("Rotate Vector and RowVector", "[rotation]")
{
    auto thetaX = 0.0;
    auto thetaY = -30.0;
    auto thetaZ = 20.0;

    using TestAngles = tau::RotationAngles<double>;

    auto yawPitchRoll =
        TestAngles(thetaZ, thetaY, thetaX, tau::AxisOrder{2, 1, 0});

    auto yawPitchRollMatrix = yawPitchRoll.GetRotation();

    tau::Vector3<double> p(1, 0, 0);

    std::cout << yawPitchRollMatrix * p << std::endl;
    std::cout << p.transpose() * yawPitchRollMatrix << std::endl;

    // TODO: Add a REQUIRE
}
