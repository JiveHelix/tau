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
    using Matrix = typename Derived::PlainObject;

    using Svd = Eigen::JacobiSVD<Matrix>;

    Svd svd(factors, Eigen::ComputeFullV);

    // Return the last column of V.
    return svd.matrixV().col(svd.matrixV().cols() - 1);
}


} // end namespace tau
