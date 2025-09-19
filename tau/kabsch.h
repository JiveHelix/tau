#pragma once


#include <tau/eigen.h>
#include <vector>
#include <tau/error.h>


namespace tau
{


template<typename T, int dimensions>
struct KabschResult
{
    Eigen::Matrix<T, dimensions, dimensions> rotation;
    Eigen::Vector<T, dimensions> translation;
};


template<typename T, int dimensions>
using KabschPoints = Eigen::Matrix<T, dimensions, Eigen::Dynamic>;


template<typename T, int dimensions>
KabschResult<T, dimensions> SolveKabsch(
    const KabschPoints<T, dimensions> &localPoints,
    const KabschPoints<T, dimensions> &worldPoints)
{
    using PointMatrix = KabschPoints<T, dimensions>;
    using Vector = Eigen::Vector<T, dimensions>;

    if (localPoints.cols() != worldPoints.cols())
    {
        throw tau::TauError("Requires same point count.");
    }

    static_assert(dimensions > 0);

    if (localPoints.size() < static_cast<long>(dimensions))
    {
        throw tau::TauError("Insufficient point count.");
    }

    // Compute the centroid of each set of points.
    const auto count = static_cast<T>(localPoints.cols());

    Vector localMean = localPoints.rowwise().sum() / count;
    Vector worldMean = worldPoints.rowwise().sum() / count;

    // Center each group around 0.
    PointMatrix translatedLocal = localPoints.colwise() - localMean;
    PointMatrix translatedWorld = worldPoints.colwise() - worldMean;

    // Build covariance matrix
    using Inner = Eigen::Matrix<T, dimensions, dimensions>;
    Inner covariance = translatedWorld * translatedLocal.transpose();

    Eigen::JacobiSVD<Inner> svd(
        covariance,
        Eigen::ComputeFullU | Eigen::ComputeFullV);

    Inner U = svd.matrixU();
    Inner V = svd.matrixV();
    Inner S = Inner::Identity();

    if ((U * V.transpose()).determinant() < 0.0)
    {
        // The orthogonal matrices contain a reflection.
        S(dimensions - 1, dimensions - 1) = -1.0;
    }

    Inner rotation = U * S * V.transpose();
    Vector translation = worldMean - rotation * localMean;

    return {rotation, translation};
}


} // end namespace
