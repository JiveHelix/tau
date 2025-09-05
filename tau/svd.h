#pragma once


#include <iostream>
#include <tau/eigen_shim.h>
#include <jive/equal.h>
#include <tau/error.h>


namespace tau
{


template<typename Derived>
Eigen::VectorX<typename Derived::Scalar>
SvdSolve(Eigen::MatrixBase<Derived> &factors)
{
    using Scalar = typename Derived::Scalar;
    using Result = Eigen::VectorX<Scalar>;

#ifdef __GNUG__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-enum-enum-conversion"
#endif

    using Svd = Eigen::JacobiSVD<Derived>;

    Svd svd;

    svd.compute(
        factors.derived(),
        Eigen::ColPivHouseholderQRPreconditioner
            | Eigen::ComputeFullU
            | Eigen::ComputeFullV);

#ifdef __GNUG__
#pragma GCC diagnostic pop
#endif

    Result nullSpace;

    Eigen::MatrixX<Scalar> values = svd.matrixV();

    Eigen::Index solutionIndex = values.cols() - 1;

    while (solutionIndex > 0)
    {
        nullSpace = svd.matrixV()(Eigen::all, solutionIndex);
        Scalar magnitude = nullSpace.transpose() * nullSpace;

        if (jive::Roughly(magnitude, 1e-6) == 1)
        {
            return nullSpace;
        }
        else
        {
            // std::cout << "magnitude not 1: " << magnitude << std::endl;
            // std::cout << "solutionIndex: " << solutionIndex << std::endl;
        }

        --solutionIndex;
    }

    throw TauError("Unable to find solution with magnitude 1");
}


} // end namespace tau
