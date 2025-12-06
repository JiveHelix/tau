#pragma once


#include <tau/eigen.h>
#include <tau/size.h>


namespace tau
{


template<typename Derived>
struct Dynamic_
{
    using Type =
        Eigen::Matrix
        <
            typename Derived::Scalar,
            Eigen::Dynamic,
            Eigen::Dynamic,
            Derived::IsRowMajor ? Eigen::RowMajor : Eigen::ColMajor
        >;
};


template<typename Derived>
using Dynamic = typename Dynamic_<Derived>::Type;


struct MatrixSize
{
    using Index = Eigen::Index;
    Index rows;
    Index columns;

    bool operator==(const Size<Index> &other)
    {
        return this->rows == other.height && this->columns == other.width;
    }

    bool operator!=(const Size<Index> &other)
    {
        return !this->operator==(other);
    }
};


inline MatrixSize GetExtendedSize(
    const tau::Size<Eigen::Index> &size,
    Eigen::Index rowExtend,
    Eigen::Index columnExtend)
{
    return {
        size.height + (2 * rowExtend),
        size.width + (2 * columnExtend)};
}


template<typename Derived>
MatrixSize GetExtendedSize(
    const Eigen::MatrixBase<Derived> &data,
    Eigen::Index rowExtend,
    Eigen::Index columnExtend)
{
    return {
        data.rows() + (2 * rowExtend),
        data.cols() + (2 * columnExtend)};
}


// Returned matrix will always by dynamically sized because the padding
// arguments are runtime values.
template<typename Derived>
Dynamic<Derived> Extend(
    const Eigen::MatrixBase<Derived> &data,
    Eigen::Index rowExtend,
    Eigen::Index columnExtend)
{
    using Result = Dynamic<Derived>;

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

    auto extendedSize = GetExtendedSize(derived, rowExtend, columnExtend);
    auto result = Result(extendedSize.rows, extendedSize.columns);

    // Copy the original matrix
    result.block(rowExtend, columnExtend, rows, columns) = derived;

    return result;
}


template<typename Derived>
void Replicate(
    Eigen::MatrixBase<Derived> &data,
    Eigen::Index rowExtend,
    Eigen::Index columnExtend)
{
    using Eigen::last;

    auto rows = data.rows() - (2 * rowExtend);
    auto columns = data.cols() - (2 * columnExtend);
    auto validView = data.block(rowExtend, columnExtend, rows, columns);

    // Replicate top and bottom regions.
    data.block(0, columnExtend, rowExtend, columns) =
        validView.row(0).replicate(rowExtend, 1);

    data.block(rowExtend + rows, columnExtend, rowExtend, columns) =
        validView.row(rows - 1).replicate(rowExtend, 1);

    // Replicate left and right regions.
    data.block(rowExtend, 0, rows, columnExtend) =
        validView.col(0).replicate(1, columnExtend);

    data.block(rowExtend, columnExtend + columns, rows, columnExtend) =
        validView.col(columns - 1).replicate(1, columnExtend);

    // Corner regions
    data.block(0, 0, rowExtend, columnExtend).array() = validView(0, 0);

    data.block(rowExtend + rows, 0, rowExtend, columnExtend).array() =
        validView(last, 0);

    data.block(0, columnExtend + columns, rowExtend, columnExtend).array() =
        validView(0, last);

    data.block(
        rowExtend + rows,
        columnExtend + columns,
        rowExtend,
        columnExtend).array() = validView(last, last);
}


struct Margins
{
    using Index = Eigen::Index;
    Index horizontalMargin;
    Index verticalMargin;

    template<typename... Kernels>
    static Margins Create(Kernels &&...kernels)
    {
        using Eigen::Index;
        Index maximumColumns = std::max({kernels.cols()...});
        Index maximumRows = std::max({kernels.rows()...});

        return {maximumColumns / 2, maximumRows / 2};
    }

    static Margins Create(Eigen::Index radius)
    {
        return {radius, radius};
    }

    bool Contains(const Margins &other) const
    {
        return this->horizontalMargin >= other.horizontalMargin
            && this->verticalMargin >= other.verticalMargin;
    }

    bool HasMargin() const
    {
        return this->horizontalMargin > 0
            || this->verticalMargin > 0;
    }

    template<typename Derived>
    Dynamic<Derived> AddMargin(const Eigen::MatrixBase<Derived> &data) const
    {
        if (this->HasMargin())
        {
            auto extended = Extend(
                data,
                this->verticalMargin,
                this->horizontalMargin);

            Replicate(
                extended,
                this->verticalMargin,
                this->horizontalMargin);

            return extended;
        }

        return data;
    }

    template<typename Derived>
    MatrixSize GetValidSize(const Eigen::MatrixBase<Derived> &data) const
    {
        return {
            data.rows() - (2 * this->verticalMargin),
            data.cols() - (2 * this->horizontalMargin)};
    }

    template<typename Derived>
    Dynamic<Derived> RemoveMargin(const Eigen::MatrixBase<Derived> &data) const
    {
        if (!this->HasMargin())
        {
            return data;
        }

        auto validSize = this->GetValidSize(data);

        return data.block(
            this->verticalMargin,
            this->horizontalMargin,
            validSize.rows,
            validSize.columns);
    }
};


template<typename T>
concept HasAddMargin = requires(T t)
{
    { t.AddMargin(std::declval<Margins>()) };
};

template<typename T>
concept HasRemoveMargin = requires(T t)
{
    { t.RemoveMargin(std::declval<Margins>()) };
};


} // end namespace tau
