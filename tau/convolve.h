#pragma once

#include <iostream>
#include <tau/eigen.h>
#include <tau/view.h>
#include <tau/margins.h>


// #define TAU_CONVOLVE_TRANSPOSE_MAJOR


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


template<typename KernelDerived, typename Derived>
Borders<Derived> BordersFromKernel(
    const Eigen::MatrixBase<KernelDerived> &kernel,
    const Eigen::MatrixBase<Derived> &input)
{
    return Borders<Derived>(input, kernel.rows(), kernel.cols());
}


namespace detail
{


template<typename Derived>
struct AddressRange_
{
    using PointerType = typename Derived::Scalar *;
    using Type = std::pair<const PointerType, const PointerType>;
};

template<typename Derived>
using AddressRange = typename AddressRange_<Derived>::Type;


template<typename Derived>
auto GetAddressRange(const Eigen::MatrixBase<Derived> &matrix)
    -> AddressRange<Derived>
{
    static_assert(
        (Derived::Flags & Eigen::DirectAccessBit) != 0,
        "matrix must have direct access");

    using Scalar = typename Derived::Scalar;

    if (matrix.size() == 0)
    {
        return {nullptr, nullptr};
    }

    const auto rowCount = static_cast<std::ptrdiff_t>(matrix.rows());
    const auto columnCount = static_cast<std::ptrdiff_t>(matrix.cols());
    const auto innerStride = static_cast<std::ptrdiff_t>(matrix.innerStride());
    const auto outerStride = static_cast<std::ptrdiff_t>(matrix.outerStride());

    // Compute offsets
    std::ptrdiff_t topRight;
    std::ptrdiff_t bottomLeft;

    if constexpr (Derived::IsRowMajor)
    {
        topRight = (columnCount - 1) * innerStride;
        bottomLeft = (rowCount - 1) * outerStride;
    }
    else
    {
        // Column-major
        topRight = (columnCount - 1) * outerStride;
        bottomLeft = (rowCount - 1) * innerStride;
    }

    const std::ptrdiff_t topLeft = 0;
    const std::ptrdiff_t bottomRight = bottomLeft + topRight;

    const std::ptrdiff_t minimum = std::min(
        std::min(topLeft, topRight),
        std::min(bottomLeft, bottomRight));

    const std::ptrdiff_t maximum = std::max(
        std::max(topLeft, topRight),
        std::max(bottomLeft, bottomRight));

    return {matrix.data() + minimum, matrix.data() + maximum};
}


template<typename InputDerived, typename OutputDerived>
bool IsAliased(
    const Eigen::MatrixBase<InputDerived> &input,
    const Eigen::MatrixBase<OutputDerived> &output)
{
    auto [inputMinimum, inputMaximum] = GetAddressRange(input);
    auto [outputMinimum, outputMaximum] = GetAddressRange(output);

    /*
    [.....]
            <.....>

      [.....]
            <.....>

         [.....]
            <.....>

            [.....]
            <.....>

                [.....]
            <.....>

                  [.....]
            <.....>

                      [.....]
            <.....>
    */

    return (inputMinimum <= outputMaximum) && (inputMaximum >= outputMinimum);
}


template<typename KernelDerived, typename Derived>
void DoCorrelate2d(
    const Eigen::MatrixBase<KernelDerived> &kernel,
    const Eigen::MatrixBase<Derived> &input,
    Eigen::MatrixBase<Derived> &output)
{
    using Eigen::Index;

    assert(output.rows() == input.rows());
    assert(output.cols() == input.cols());

    auto borders = BordersFromKernel(kernel, input);

    for (Index row = borders.firstRow; row < borders.limitRow; ++row)
    {
        Index rowBegin = row - borders.firstRow;

        for (
            Index column = borders.firstColumn;
            column < borders.limitColumn;
            ++column)
        {
            Index columnBegin = column - borders.firstColumn;

            auto b = input.block(
                rowBegin,
                columnBegin,
                borders.kernelRows,
                borders.kernelColumns).reshaped().dot(kernel.reshaped());

            output.coeffRef(row, column) = b;
        }
    }
}


template<typename KernelDerived, typename Derived>
void DoCorrelateRows(
    const Eigen::MatrixBase<KernelDerived> &kernel,
    Eigen::MatrixBase<Derived> &inOut)
{
    static_assert(
        KernelDerived::ColsAtCompileTime == 1
            || KernelDerived::RowsAtCompileTime == 1,
        "Expects a 1-dimensional kernel");

    static_assert(
        (Derived::Flags & Eigen::LvalueBit) != 0,
        "inOut requires a writable (lvalue) matrix or block");

    using Eigen::Index;
    using Scalar = typename Derived::Scalar;
    using Scratch = Eigen::RowVector<Scalar, Eigen::Dynamic>;

    Index rowKernelLength = kernel.size();
    assert(inOut.cols() > rowKernelLength);

    /**
    Add one here to allow the convolution overlap to extend to the far edge
    of the input buffer.
         0 1 2 3 4
             c b a
    **/
    Index columnLimit = inOut.cols() - rowKernelLength + 1;

    Index rowKernelRadius = rowKernelLength / 2;
    Index rowLimit = inOut.rows();

    Scratch scratch(columnLimit);

    for (Index row = 0; row < rowLimit; ++row)
    {
        for (Index column = 0; column < columnLimit; ++column)
        {
            auto inputView = inOut.row(row).segment(column, rowKernelLength);
            scratch(column) = inputView.dot(kernel);
        }

        inOut.row(row).segment(rowKernelRadius, columnLimit) = scratch;
    }
}


template
<
    bool isAliased,
    typename KernelDerived,
    typename InputDerived,
    typename OutputDerived
>
void DoCorrelateRows_(
    const Eigen::MatrixBase<KernelDerived> &kernel,
    const Eigen::MatrixBase<InputDerived> &input,
    Eigen::MatrixBase<OutputDerived> &output)
{
    static_assert(
        KernelDerived::ColsAtCompileTime == 1
            || KernelDerived::RowsAtCompileTime == 1,
        "Expects a 1-dimensional kernel");

    static_assert(
        (OutputDerived::Flags & Eigen::LvalueBit) != 0,
        "inOut requires a writable (lvalue) matrix or block");

    using Eigen::Index;
    using Scalar = typename OutputDerived::Scalar;
    using Scratch = Eigen::RowVector<Scalar, Eigen::Dynamic>;

    Index rowKernelLength = kernel.size();
    assert(output.rows() == input.rows());
    assert(output.cols() == input.cols());
    assert(input.cols() > rowKernelLength);

    /**
    Add one here to allow the convolution overlap to extend to the far edge
    of the input buffer.
         0 1 2 3 4
             c b a
    **/
    Index columnLimit = input.cols() - rowKernelLength + 1;

    Index rowKernelRadius = rowKernelLength / 2;
    Index rowLimit = input.rows();

    if constexpr (isAliased)
    {
        Scratch scratch(columnLimit);

        for (Index row = 0; row < rowLimit; ++row)
        {
            auto rowView = input.row(row);

            for (Index column = 0; column < columnLimit; ++column)
            {
                auto inputView = rowView.segment(column, rowKernelLength);
                scratch(column) = inputView.dot(kernel);
            }

            output.row(row).segment(rowKernelRadius, columnLimit) = scratch;
        }
    }
    else
    {
        for (Index row = 0; row < rowLimit; ++row)
        {
            auto outputView =
                output.row(row).segment(rowKernelRadius, columnLimit);

            auto rowView = input.row(row);

            for (Index column = 0; column < columnLimit; ++column)
            {
                auto inputView = rowView.segment(column, rowKernelLength);
                outputView(column) = inputView.dot(kernel);
            }
        }
    }
}


template<typename KernelDerived, typename InputDerived, typename OutputDerived>
void DoCorrelateRows(
    const Eigen::MatrixBase<KernelDerived> &kernel,
    const Eigen::MatrixBase<InputDerived> &input,
    Eigen::MatrixBase<OutputDerived> &output)
{
    static_assert(
        KernelDerived::ColsAtCompileTime == 1
            || KernelDerived::RowsAtCompileTime == 1,
        "Expects a 1-dimensional kernel");

    static_assert(
        (OutputDerived::Flags & Eigen::LvalueBit) != 0,
        "inOut requires a writable (lvalue) matrix or block");

    if constexpr ((InputDerived::Flags & Eigen::DirectAccessBit) != 0)
    {
        // Assume that input could alias output.
        DoCorrelateRows_<true>(kernel, input, output);
    }
    else
    {
        // Check for aliasing.
        if (IsAliased(input, output))
        {
            DoCorrelateRows_<true>(kernel, input, output);
        }
        else
        {
            DoCorrelateRows_<false>(kernel, input, output);
        }
    }
}


template<typename KernelDerived, typename Derived>
void DoCorrelateColumns(
    const Eigen::MatrixBase<KernelDerived> &kernel,
    Eigen::MatrixBase<Derived> &inOut)
{
    static_assert(
        KernelDerived::ColsAtCompileTime == 1
            || KernelDerived::RowsAtCompileTime == 1,
        "Expects a 1-dimensional kernel");

    static_assert(
        (Derived::Flags & Eigen::LvalueBit) != 0,
        "inOut requires a writable (lvalue) matrix or block");

    using Eigen::Index;
    using Scalar = typename Derived::Scalar;
    using Scratch = Eigen::Vector<Scalar, Eigen::Dynamic>;

    Index columnKernelLength = kernel.size();
    assert(inOut.rows() > columnKernelLength);

    /**
    Add one here to allow the convolution overlap to extend to the far edge
    of the input buffer.
         0 1 2 3 4
             c b a
    **/
    Index rowLimit = inOut.rows() - columnKernelLength + 1;
    Index columnKernelRadius = columnKernelLength / 2;
    Index columnLimit = inOut.cols();

    Scratch scratch(rowLimit);

    for (Index column = 0; column < columnLimit; ++column)
    {
        for (Index row = 0; row < rowLimit; ++row)
        {
            auto inputView = inOut.col(column).segment(row, columnKernelLength);
            scratch(row) = inputView.dot(kernel);
        }

        inOut.col(column).segment(columnKernelRadius, rowLimit) = scratch;
    }
}


template
<
    bool isAliased,
    typename KernelDerived,
    typename InputDerived,
    typename OutputDerived
>
void DoCorrelateColumns_(
    const Eigen::MatrixBase<KernelDerived> &kernel,
    const Eigen::MatrixBase<InputDerived> &input,
    Eigen::MatrixBase<OutputDerived> &output)
{
    static_assert(
        KernelDerived::ColsAtCompileTime == 1
            || KernelDerived::RowsAtCompileTime == 1,
        "Expects a 1-dimensional kernel");

    static_assert(
        (OutputDerived::Flags & Eigen::LvalueBit) != 0,
        "output requires a writable (lvalue) matrix or block");

    static_assert(
        std::same_as
        <
            typename InputDerived::Scalar,
            typename OutputDerived::Scalar
        >);


    using Eigen::Index;
    using Scalar = typename OutputDerived::Scalar;
    using Scratch = Eigen::Vector<Scalar, Eigen::Dynamic>;

    Index columnKernelLength = kernel.size();
    assert(output.rows() == input.rows());
    assert(output.cols() == input.cols());
    assert(input.rows() > columnKernelLength);

    /**
    Add one here to allow the convolution overlap to extend to the far edge
    of the input buffer.
         0 1 2 3 4
             c b a
    **/
    Index rowLimit = input.rows() - columnKernelLength + 1;
    Index columnKernelRadius = columnKernelLength / 2;
    Index columnLimit = input.cols();

    if constexpr (isAliased)
    {
        Scratch scratch(rowLimit);

        for (Index column = 0; column < columnLimit; ++column)
        {
            auto columnView = input.col(column);

            for (Index row = 0; row < rowLimit; ++row)
            {
                auto inputView = columnView.segment(row, columnKernelLength);
                scratch(row) = inputView.dot(kernel);
            }

            output.col(column).segment(columnKernelRadius, rowLimit) = scratch;
        }
    }
    else
    {
        for (Index column = 0; column < columnLimit; ++column)
        {
            auto outputView =
                output.col(column).segment(columnKernelRadius, rowLimit);

            auto columnView = input.col(column);

            for (Index row = 0; row < rowLimit; ++row)
            {
                auto inputView = columnView.segment(row, columnKernelLength);
                outputView(row) = inputView.dot(kernel);
            }
        }
    }
}


template<typename KernelDerived, typename InputDerived, typename OutputDerived>
void DoCorrelateColumns(
    const Eigen::MatrixBase<KernelDerived> &kernel,
    const Eigen::MatrixBase<InputDerived> &input,
    Eigen::MatrixBase<OutputDerived> &output)
{
    static_assert(
        KernelDerived::ColsAtCompileTime == 1
            || KernelDerived::RowsAtCompileTime == 1,
        "Expects a 1-dimensional kernel");

    static_assert(
        (OutputDerived::Flags & Eigen::LvalueBit) != 0,
        "inOut requires a writable (lvalue) matrix or block");

    if constexpr ((InputDerived::Flags & Eigen::DirectAccessBit) != 0)
    {
        // Assume that input could alias output.
        DoCorrelateColumns_<true>(kernel, input, output);
    }
    else
    {
        // Check for aliasing.
        if (IsAliased(input, output))
        {
            DoCorrelateColumns_<true>(kernel, input, output);
        }
        else
        {
            DoCorrelateColumns_<false>(kernel, input, output);
        }
    }
}


} // end namespace detail


template
<
    typename InputDerived,
    typename OutputDerived
>
bool IsSameMatrix(
    const Eigen::MatrixBase<InputDerived> &input,
    const Eigen::MatrixBase<OutputDerived> &output)
{
    if constexpr (!std::same_as<OutputDerived, InputDerived>)
    {
        return false;
    }
    else if constexpr (
        ((InputDerived::Flags & Eigen::DirectAccessBit) != 0)
        || ((OutputDerived::Flags & Eigen::DirectAccessBit) != 0))
    {
        return false;
    }
    else
    {
        return (
            (input.rows() == output.rows())
            && (input.cols() == output.cols())
            && (input.data() == output.data()));
    }
}


template<typename KernelDerived, typename Derived>
void CorrelateRows(
    const Eigen::MatrixBase<KernelDerived> &kernel,
    Eigen::MatrixBase<Derived> &inOut)
{
    static_assert(
        KernelDerived::ColsAtCompileTime == 1
            || KernelDerived::RowsAtCompileTime == 1,
        "Expects a 1-dimensional kernel");

    static_assert(
        (Derived::Flags & Eigen::LvalueBit) != 0,
        "inOut requires a writable (lvalue) matrix or block");

#ifndef TAU_CONVOLVE_TRANSPOSE_MAJOR
    detail::DoCorrelateRows(kernel, inOut);
#else
    if constexpr (Derived::IsRowMajor)
    {
        detail::DoCorrelateRows(kernel, inOut);
    }
    else
    {
        // inOut is column-major.
        // For cache efficiency, tranpose to place the row values in the
        // columns, in column-major order.
        auto transposed = inOut.transpose().eval();

        // The rows we need are now in the columns.
        // DoCorrelateColumns, but with the row kernel.
        detail::DoCorrelateColumns(kernel, transposed);

        // Copy the values back
        inOut = transposed.transpose();
    }
#endif
}


struct ViewTag {};


template<typename KernelDerived, IsEigenRef InOut>
void CorrelateRows(
    ViewTag,
    const Eigen::MatrixBase<KernelDerived> &kernel,
    InOut inOut)
{
    CorrelateRows(kernel, inOut);
}


template<typename KernelDerived, typename InputDerived, typename OutputDerived>
void CorrelateRows(
    const Eigen::MatrixBase<KernelDerived> &kernel,
    const Eigen::MatrixBase<InputDerived> &input,
    Eigen::MatrixBase<OutputDerived> &output)
{
    static_assert(
        KernelDerived::ColsAtCompileTime == 1
            || KernelDerived::RowsAtCompileTime == 1,
        "Expects a 1-dimensional kernel");

    static_assert(
        (OutputDerived::Flags & Eigen::LvalueBit) != 0,
        "inOut requires a writable (lvalue) matrix or block");

    if (IsSameMatrix(input, output))
    {
        CorrelateRows(kernel, output);

        return;
    }

#ifndef TAU_CONVOLVE_TRANSPOSE_MAJOR
    detail::DoCorrelateRows(kernel, input, output);
#else
    if constexpr (InputDerived::IsRowMajor && OutputDerived::IsRowMajor)
    {
        detail::DoCorrelateRows(kernel, input, output);
    }
    else
    {
        // input and output are column-major.
        // For cache efficiency, tranpose to place the row values in the
        // columns, in column-major order.
        auto transposedInput = input.transpose().eval();

        // Rows and Columns are swapped
        using TransposedOutput =
            Eigen::Matrix
            <
                typename OutputDerived::Scalar,
                OutputDerived::ColsAtCompileTime,
                OutputDerived::RowsAtCompileTime,
                Eigen::ColMajor
            >;

        TransposedOutput transposedOutput(output.cols(), output.rows());

        // The rows we need are now in the columns.
        // DoCorrelateColumns, but with the row kernel.
        detail::DoCorrelateColumns_<false>(
            kernel,
            transposedInput,
            transposedOutput);

        // Copy the values back
        output = transposedOutput.transpose();
    }
#endif
}


template<typename KernelDerived, IsEigenConstRef Input, IsEigenRef Output>
void CorrelateRows(
    ViewTag,
    const Eigen::MatrixBase<KernelDerived> &kernel,
    const Input &input,
    Output output)
{
    CorrelateRows(kernel, input, output);
}


template<typename KernelDerived, typename Derived>
void CorrelateColumns(
    const Eigen::MatrixBase<KernelDerived> &kernel,
    Eigen::MatrixBase<Derived> &inOut)
{
    static_assert(
        KernelDerived::ColsAtCompileTime == 1
            || KernelDerived::RowsAtCompileTime == 1,
        "Expects a 1-dimensional kernel");

    static_assert(
        (Derived::Flags & Eigen::LvalueBit) != 0,
        "inOut requires a writable (lvalue) matrix or block");

#ifndef TAU_CONVOLVE_TRANSPOSE_MAJOR
    detail::DoCorrelateColumns(kernel, inOut);
#else
    if constexpr (!Derived::IsRowMajor)
    {
        // Already in column-major order.
        detail::DoCorrelateColumns(kernel, inOut);
    }
    else
    {
        // inOut is row-major.
        // For cache efficiency, tranpose to place the column values in the
        // rows, in row-major order.
        auto tranposed = inOut.transpose().eval();

        // The columns we need are now in the rows.
        // DoCorrelateRows, but with the column kernel.
        detail::DoCorrelateRows(kernel, tranposed);

        // Copy the values back
        inOut = tranposed.transpose();
    }
#endif
}


template<typename KernelDerived, IsEigenRef InOut>
void CorrelateColumns(
    ViewTag,
    const Eigen::MatrixBase<KernelDerived> &kernel,
    InOut inOut)
{
    CorrelateColumns(kernel, inOut);
}


template
<
    typename KernelDerived,
    typename InputDerived,
    typename OutputDerived
>
void CorrelateColumns(
    const Eigen::MatrixBase<KernelDerived> &kernel,
    const Eigen::MatrixBase<InputDerived> &input,
    Eigen::MatrixBase<OutputDerived> &output)
{
    static_assert(
        KernelDerived::ColsAtCompileTime == 1
            || KernelDerived::RowsAtCompileTime == 1,
        "Expects a 1-dimensional kernel");

    static_assert(
        (OutputDerived::Flags & Eigen::LvalueBit) != 0,
        "inOut requires a writable (lvalue) matrix or block");

    if (IsSameMatrix(input, output))
    {
        CorrelateColumns(kernel, output);

        return;
    }

#ifndef TAU_CONVOLVE_TRANSPOSE_MAJOR
    detail::DoCorrelateColumns(kernel, input, output);
#else
    if constexpr (!InputDerived::IsRowMajor && !OutputDerived::IsRowMajor)
    {
        // Already in column-major order.
        detail::DoCorrelateColumns(kernel, input, output);
    }
    else
    {
        // inOut is row-major.
        // For cache efficiency, tranpose to place the column values in the
        // rows, in row-major order.
        auto transposedInput = input.transpose().eval();

        // Rows and Columns are swapped
        using TransposedOutput =
            Eigen::Matrix
            <
                typename OutputDerived::Scalar,
                OutputDerived::ColsAtCompileTime,
                OutputDerived::RowsAtCompileTime,
                Eigen::RowMajor
            >;

        TransposedOutput transposedOutput(output.cols(), output.rows());

        // The columns we need are now in the rows.
        // DoCorrelateRows, but with the column kernel.
        detail::DoCorrelateRows_<false>(
            kernel,
            transposedInput,
            transposedOutput);

        // Copy the values back
        output = transposedOutput.transpose();
    }
#endif
}


template
<
    typename KernelDerived,
    IsEigenConstRef Input,
    IsEigenRef Output
>
void CorrelateColumns(
    ViewTag,
    const Eigen::MatrixBase<KernelDerived> &kernel,
    const Input &input,
    Output output)
{
    CorrelateColumns(kernel, input, output);
}


template
<
    typename RowKernel,
    typename ColumnKernel,
    typename InputDerived,
    typename OutputDerived
>
void CorrelateSeparable(
    const Eigen::MatrixBase<RowKernel> &rowKernel,
    const Eigen::MatrixBase<ColumnKernel> &columnKernel,
    const Eigen::MatrixBase<InputDerived> &input,
    Eigen::MatrixBase<OutputDerived> &output)
{
    CorrelateRows(rowKernel, input, output);
    CorrelateColumns(columnKernel, input, output);
}


template
<
    typename RowKernel,
    typename ColumnKernel,
    IsEigenConstRef Input,
    IsEigenRef Output
>
void CorrelateSeparable(
    ViewTag,
    const Eigen::MatrixBase<RowKernel> &rowKernel,
    const Eigen::MatrixBase<ColumnKernel> &columnKernel,
    const Input &input,
    Output output)
{
    CorrelateRows(rowKernel, input, output);
    CorrelateColumns(columnKernel, input, output);
}


template
<
    typename RowKernel,
    typename ColumnKernel,
    typename Derived
>
void CorrelateSeparable(
    const Eigen::MatrixBase<RowKernel> &rowKernel,
    const Eigen::MatrixBase<ColumnKernel> &columnKernel,
    Eigen::MatrixBase<Derived> &inOut)
{
    CorrelateRows(rowKernel, inOut);
    CorrelateColumns(columnKernel, inOut);
}


template
<
    typename RowKernel,
    typename ColumnKernel,
    IsEigenRef InOut
>
void CorrelateSeparable(
    ViewTag,
    const Eigen::MatrixBase<RowKernel> &rowKernel,
    const Eigen::MatrixBase<ColumnKernel> &columnKernel,
    InOut inOut)
{
    CorrelateRows(rowKernel, inOut);
    CorrelateColumns(columnKernel, inOut);
}


template<typename Derived>
struct Evaluated_
{
    using Type =
        Eigen::Matrix
        <
            typename Derived::Scalar,
            Derived::RowsAtCompileTime,
            Derived::ColsAtCompileTime,
            Derived::IsRowMajor ? Eigen::RowMajor : Eigen::ColMajor
        >;
};


template<typename Derived>
using Evaluated = typename Evaluated_<Derived>::Type;


template<typename Derived>
struct Kernel
{
    Evaluated<Derived> natural;
    Evaluated<Derived> flipped;

    Kernel(const Eigen::MatrixBase<Derived> &natural_)
        :
        natural(natural_),
        flipped(natural.reverse())
    {

    }
};


template<typename KernelDerived, typename Derived>
void ConvolveRows(
    const Kernel<KernelDerived> &kernel,
    Eigen::MatrixBase<Derived> &inOut)
{
    CorrelateRows(kernel.flipped, inOut);
}


template<typename KernelDerived, IsEigenRef InOut>
void ConvolveRows(
    ViewTag,
    const Kernel<KernelDerived> &kernel,
    InOut &inOut)
{
    CorrelateRows(kernel.flipped, inOut);
}




template<typename KernelDerived, typename InputDerived, typename OutputDerived>
void ConvolveRows(
    const Kernel<KernelDerived> &kernel,
    const Eigen::MatrixBase<InputDerived> &input,
    Eigen::MatrixBase<OutputDerived> &output)
{
    CorrelateRows(kernel.flipped, input, output);
}

template<typename KernelDerived, IsEigenConstRef Input, IsEigenRef Output>
void ConvolveRows(
    ViewTag,
    const Kernel<KernelDerived> &kernel,
    const Input &input,
    Output output)
{
    CorrelateRows(kernel.flipped, input, output);
}





template<typename KernelDerived, typename Derived>
void ConvolveColumns(
    const Kernel<KernelDerived> &kernel,
    Eigen::MatrixBase<Derived> &inOut)
{
    CorrelateColumns(kernel.flipped, inOut);
}

template<typename KernelDerived, IsEigenRef InOut>
void ConvolveColumns(
    ViewTag,
    const Kernel<KernelDerived> &kernel,
    InOut inOut)
{
    CorrelateColumns(kernel.flipped, inOut);
}





template<typename KernelDerived, typename InputDerived, typename OutputDerived>
void ConvolveColumns(
    const Kernel<KernelDerived> &kernel,
    const Eigen::MatrixBase<InputDerived> &input,
    Eigen::MatrixBase<OutputDerived> &output)
{
    CorrelateColumns(kernel.flipped, input, output);
}

template<typename KernelDerived, IsEigenConstRef Input, IsEigenRef Output>
void ConvolveColumns(
    ViewTag,
    const Kernel<KernelDerived> &kernel,
    const Input &input,
    Output output)
{
    CorrelateColumns(kernel.flipped, input, output);
}




template<typename RowKernel, typename ColumnKernel, typename Derived>
void ConvolveSeparable(
    const Kernel<RowKernel> &rowKernel,
    const Kernel<ColumnKernel> &columnKernel,
    Eigen::MatrixBase<Derived> &inOut)
{
    ConvolveRows(rowKernel, inOut);
    ConvolveColumns(columnKernel, inOut);
}


template<typename RowKernel, typename ColumnKernel, IsEigenRef InOut>
void ConvolveSeparable(
    ViewTag,
    const Kernel<RowKernel> &rowKernel,
    const Kernel<ColumnKernel> &columnKernel,
    InOut inOut)
{
    ConvolveRows(rowKernel, inOut);
    ConvolveColumns(columnKernel, inOut);
}






template
<
    typename RowKernel,
    typename ColumnKernel,
    typename InputDerived,
    typename OutputDerived
>
void ConvolveSeparable(
    const Kernel<RowKernel> &rowKernel,
    const Kernel<ColumnKernel> &columnKernel,
    const Eigen::MatrixBase<InputDerived> &input,
    Eigen::MatrixBase<OutputDerived> &output)
{
    ConvolveRows(rowKernel, input, output);
    ConvolveColumns(columnKernel, output);
}


template
<
    typename RowKernel,
    typename ColumnKernel,
    IsEigenConstRef Input,
    IsEigenRef Output
>
void ConvolveSeparable(
    ViewTag,
    const Kernel<RowKernel> &rowKernel,
    const Kernel<ColumnKernel> &columnKernel,
    const Input &input,
    Output output)
{
    ConvolveRows(rowKernel, input, output);
    ConvolveColumns(columnKernel, output);
}





template<typename Kernel, typename Derived>
void Convolve2d(
    const Eigen::MatrixBase<Kernel> &kernel,
    const Eigen::MatrixBase<Derived> &input,
    Eigen::MatrixBase<Derived> &output)
{
    Kernel preparedKernel = kernel.reverse();
    detail::DoCorrelate2d(preparedKernel, input, output);
}


template<typename Kernel, typename Derived>
Derived Convolve2d(
    const Eigen::MatrixBase<Kernel> &kernel,
    const Eigen::MatrixBase<Derived> &input)
{
    Derived output(input.rows(), input.cols());
    Convolve2d(kernel, input, output);

    return output;
}


// For floating-point, it is faster to normalize the kernel prior to
// convolution. Pre-normalization of an integral kernel loses precision.
template<typename Kernel, typename Derived>
Dynamic<Derived> Normalize(
    const Eigen::MatrixBase<Kernel> &kernel,
    const Eigen::MatrixBase<Derived> &input)
{
    typedef typename Derived::Scalar Scalar;
    auto sum = static_cast<Scalar>(kernel.sum());

    if ((sum == 0) || (sum == 1))
    {
        return input.derived();
    }

    auto borders = BordersFromKernel(kernel, input);

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
        Derived asFloats = input.derived();

        asFloats.array().block(
            borders.firstRow,
            borders.firstColumn,
            borders.rows,
            borders.columns) /= sum;

        return asFloats;
    }
}


} // end namespace tau
