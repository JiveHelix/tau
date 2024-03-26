#pragma once

#include <iostream>
#include "tau/eigen.h"


namespace tau
{


template<typename Derived>
struct Borders
{
    using Index = Eigen::Index;

    Borders(
        const Eigen::MatrixBase<Derived> &input,
        Eigen::Index kernelRows_,
        Eigen::Index kernelColumns_)
        :
        kernelRows(kernelRows_),
        kernelColumns(kernelColumns_),
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
Borders<Derived> BordersFromKernel(
    const Eigen::MatrixBase<Derived> &input,
    const Eigen::MatrixBase<Kernel> &kernel)
{
    return Borders<Derived>(input, kernel.rows(), kernel.cols());
}


template<typename Derived, typename Kernel>
Derived DoConvolve2d(
    const Eigen::MatrixBase<Derived> &input,
    const Eigen::MatrixBase<Kernel> &reversedKernel)
{
    using Eigen::Index;

    Derived output = input.derived();

    typedef typename Derived::Scalar Scalar;

    // Leave the borders untouched
    auto borders = BordersFromKernel(input, reversedKernel);

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
Derived DoConvolve2d(
    const Eigen::MatrixBase<Derived> &input,
    const Eigen::MatrixBase<Kernel0> &kernel0,
    const Eigen::MatrixBase<Kernel1> &kernel1)
{
    Derived result = DoConvolve2d(input, kernel0);
    return DoConvolve2d(result, kernel1);
}


template<typename Derived, typename Kernel>
Derived Convolve2d(
    const Eigen::MatrixBase<Derived> &input,
    const Eigen::MatrixBase<Kernel> &kernel)
{
    Kernel preparedKernel = kernel.reverse();
    return DoConvolve2d(input, preparedKernel);
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

    auto borders = BordersFromKernel(input, kernel);

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


template<typename Derived>
struct Extended_
{
    using Type = Eigen::MatrixX<typename Derived::Scalar>;
};

template<typename Derived>
using Extended = typename Extended_<Derived>::Type;


template <typename Derived>
Extended<Derived> Extend(
    const Eigen::DenseBase<Derived> &data,
    Eigen::Index rowExtend,
    Eigen::Index columnExtend)
{
    using Eigen::last;

    if (rowExtend < 0 || columnExtend < 0)
    {
        throw std::logic_error(
            "This method only extends. "
            "To shrink a matrix, use the block API.");
    }

    const auto &derived = data.derived();
    auto rows = derived.rows();
    auto columns = derived.cols();

    Extended<Derived> result(rows + 2 * rowExtend, columns + 2 * columnExtend);

    // Copy the original matrix
    result.block(rowExtend, columnExtend, rows, columns) = derived;

    // Replicate top and bottom regions.
    result.block(0, columnExtend, rowExtend, columns) =
        derived.row(0).replicate(rowExtend, 1);

    result.block(rowExtend + rows, columnExtend, rowExtend, columns) =
        derived.row(rows - 1).replicate(rowExtend, 1);

    // Replicate left and right regions.
    result.block(rowExtend, 0, rows, columnExtend) =
        derived.col(0).replicate(1, columnExtend);

    result.block(rowExtend, columnExtend + columns, rows, columnExtend) =
        derived.col(columns - 1).replicate(1, columnExtend);

    // Corner regions
    result.block(0, 0, rowExtend, columnExtend).array() = derived(0, 0);

    result.block(rowExtend + rows, 0, rowExtend, columnExtend).array() =
        derived(last, 0);

    result.block(0, columnExtend + columns, rowExtend, columnExtend).array() =
        derived(0, last);

    result.block(
        rowExtend + rows,
        columnExtend + columns,
        rowExtend,
        columnExtend).array() = derived(last, last);

    return result;
}


} // end namespace tau
