#pragma once

#include <iostream>
#include "tau/eigen.h"


namespace tau
{


template<typename Derived, typename Kernel>
struct Borders
{
    using Index = typename Eigen::Index;

    Borders(
        const Eigen::MatrixBase<Derived> &input,
        const Eigen::MatrixBase<Kernel> &kernel)
        :
        kernelRows(kernel.rows()),
        kernelColumns(kernel.cols()),
        firstRow(this->kernelRows / 2),
        firstColumn(this->kernelColumns / 2),
        limitRow(input.rows() - this->firstRow),
        limitColumn(input.cols() - this->firstColumn),
        rows(this->limitRow - this->firstRow),
        columns(this->limitColumn - this->firstColumn)
    {

    }

    Index kernelRows;
    Index kernelColumns;
    Index firstRow;
    Index firstColumn;
    Index limitRow;
    Index limitColumn;
    Index rows;
    Index columns;
};


template<typename Derived, typename Kernel>
Derived DoConvolve(
    const Eigen::MatrixBase<Derived> &input,
    const Eigen::MatrixBase<Kernel> &reversedKernel)
{
    using Eigen::Index;

    Derived output = input;

    typedef typename Derived::Scalar Scalar;

    // Leave the borders untouched
    auto borders = Borders(input, reversedKernel);

    for (Index row = borders.firstRow; row < borders.limitRow; ++row)
    {
        Index rowBegin = row - borders.firstRow;

        for (
            Index column = borders.firstColumn;
            column < borders.limitColumn;
            ++column)
        {
            Index columnBegin = column - borders.firstColumn;

            Scalar b = (
                static_cast<Kernel>(
                    input.block(
                            rowBegin,
                            columnBegin,
                            borders.kernelRows,
                            borders.kernelColumns))
                        .cwiseProduct(reversedKernel)).sum();

            output.coeffRef(row, column) = b;
        }
    }

    return output;
}


template<typename Derived, typename Kernel0, typename Kernel1>
Derived DoConvolve(
    const Eigen::MatrixBase<Derived> &input,
    const Eigen::MatrixBase<Kernel0> &kernel0,
    const Eigen::MatrixBase<Kernel1> &kernel1)
{
    Derived result = DoConvolve(input, kernel0);
    return DoConvolve(result, kernel1);
}


template<typename Derived, typename Kernel>
Derived Convolve(
    const Eigen::MatrixBase<Derived> &input,
    const Eigen::MatrixBase<Kernel> &kernel)
{
    Kernel preparedKernel = kernel.reverse();
    return DoConvolve(input, preparedKernel);
}


template<typename Derived, typename Kernel>
Derived Normalize(
    const Eigen::MatrixBase<Derived> &input,
    const Eigen::MatrixBase<Kernel> &kernel)
{
    typedef typename Derived::Scalar Scalar;
    Scalar sum = kernel.sum();

    if ((sum == 0) || (sum == 1))
    {
        return input;
    }

    auto borders = Borders(input, kernel);

    if constexpr (std::is_integral_v<Scalar>)
    {
        Eigen::MatrixX<double> asFloats = input.template cast<double>();

        asFloats.array().block(
            borders.firstRow,
            borders.firstColumn,
            borders.rows,
            borders.columns) /= static_cast<double>(sum);

        return asFloats.array().round().template cast<Scalar>();
    }
    else
    {
        Derived asFloats = input;

        asFloats.array().block(
            borders.firstRow,
            borders.firstColumn,
            borders.rows,
            borders.columns) /= sum;

        return asFloats;
    }
}


} // end namespace tau
