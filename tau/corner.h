#pragma once


#include <fields/fields.h>
#include <pex/interface.h>
#include <pex/range.h>
#include <tau/eigen.h>
#include <tau/vector2d.h>
#include <tau/image.h>


namespace tau
{


template<typename T>
struct CornerFields
{
    static constexpr auto fields = std::make_tuple(
        fields::Field(&T::window, "window"),
        fields::Field(&T::count, "count"));
};


template<template<typename> typename T>
struct CornerTemplate
{
    using WindowLow = pex::Limit<3>;
    using WindowHigh = pex::Limit<32>;

    using CountLow = pex::Limit<1>;
    using CountHigh = pex::Limit<4>;

    T<pex::MakeRange<Eigen::Index, WindowLow, WindowHigh>> window;
    T<pex::MakeRange<Eigen::Index, CountLow, CountHigh>> count;

    static constexpr auto fields =
        CornerFields<CornerTemplate>::fields;

    static constexpr auto fieldsTypeName = "Corner";
};


struct CornerSettings
    :
    public CornerTemplate<pex::Identity>
{
    static CornerSettings Default()
    {
        static constexpr Eigen::Index defaultWindow = 12;
        static constexpr Eigen::Index defaultCount = 4;

        return {{defaultWindow, defaultCount}};
    }
};


struct CornerPoint
{
    Point2d<double> point;
    double count;

    CornerPoint(double x, double y, double count_);

    bool operator>(const CornerPoint &other) const;

    bool operator<(const CornerPoint &other) const;

    // For the purpose of determining unique corners, corners with the same
    // point compare equal, even if their counts differ.
    bool operator==(const CornerPoint &other) const;
};


using CornerPointsCollection = std::vector<CornerPoint>;

CornerPointsCollection GetPoints(const ImageMatrixFloat &input);


namespace internal
{


class CornerCollector
{
public:
    using Index = typename Eigen::Index;

    CornerCollector(size_t windowSize, size_t count);

    const CornerPointsCollection & GetCorners();

    void CollectFromWindow(
        ImageMatrixFloat &input,
        Eigen::Index windowRow,
        Eigen::Index windowColumn);

private:
    size_t windowSize_;
    size_t count_;
    Point2dCollection<double> points_;
    CornerPointsCollection corners_;
};


} // end namespace internal


class Corner
{
public:
    Corner(const CornerSettings &settings);

    CornerPointsCollection Filter(const ImageMatrixFloat &input);

private:
    size_t count_;
    Eigen::Index windowSize_;
};


} // end namespace tau
