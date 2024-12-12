#pragma once

#include <jive/power.h>
#include <jive/range.h>
#include "tau/eigen.h"
#include "tau/size.h"


namespace tau
{


CREATE_EXCEPTION(PlanarError, TauError);


template
<
    size_t count_,
    typename T,
    int rows_,
    int columns_,
    int options_ = Eigen::ColMajor
>
class Planar
{
public:
    static constexpr auto count = count_;
    using Type = T;
    static constexpr auto rows = rows_;
    static constexpr auto columns = columns_;
    static constexpr auto options = options_;

    using Matrix = Eigen::Matrix<T, rows, columns, options>;
    using Index = Eigen::Index;

    static_assert(
        count < std::numeric_limits<int>::max(),
        "count will be cast to int for use in Eigen template expressions.");

    template<size_t, typename, int, int, int>
    friend class Planar;

    std::array<Matrix, count> planes;

    Planar()
        :
        planes{}
    {

    }

    Planar([[maybe_unused]] Index rowCount, [[maybe_unused]] Index columnCount)
        :
        planes{}
    {
        if constexpr (rows == Eigen::Dynamic || columns == Eigen::Dynamic)
        {
            for (auto &plane: this->planes)
            {
                plane = Matrix(rowCount, columnCount);
            }
        }
        else
        {
            assert(rowCount == rows);
            assert(columnCount == columns);
        }
    }

    Planar([[maybe_unused]] const Size<Index> &size)
        :
        Planar(size.height, size.width)
    {

    }

    using Value = Planar<count, T, 1, 1, options>;

    Value operator()(Index row, Index column) const
    {
        Value result{};

#if defined _MSC_VER
        // MSVC is confused by correct C++ syntax...
        this->template Planar::GetValue_(
            result,
            row,
            column,
            std::make_index_sequence<count>{});
#else
        // Clang and GCC are not
        this->template GetValue_(
            result,
            row,
            column,
            std::make_index_sequence<count>{});
#endif

        return result;
    }

    Eigen::Vector<T, int(count)> GetVector(Index row, Index column) const
    {
        Eigen::Vector<T, int(count)> result;

#if defined _MSC_VER
        // MSVC is confused by correct C++ syntax...
        this->template Planar::GetVector_(
            result,
            row,
            column,
            std::make_index_sequence<count>{});
#else
        // Clang and GCC are not
        this->template GetVector_(
            result,
            row,
            column,
            std::make_index_sequence<count>{});
#endif

        return result;
    }

    template<size_t...I>
    Eigen::Vector<T, int(sizeof...(I))> GetVector(
        Index row,
        Index column,
        std::index_sequence<I...> indices) const
    {
        Eigen::Vector<T, int(sizeof...(I))> result;

#if defined _MSC_VER
        // MSVC is confused by correct C++ syntax...
        this->template Planar::GetVector_(
            result,
            row,
            column,
            indices);
#else
        // Clang and GCC are not
        this->template GetVector_(
            result,
            row,
            column,
            indices);
#endif

        return result;
    }

    // First plane contains minima, and second plane contains maxima.
    using Extrema = Planar<2, T, rows, columns, options>;

    // First plane contains the plane index of the minima.
    // Second plane contains the plane index of the maxima.
    using ExtremaIndices = Planar<2, Index, rows, columns, options>;

    template<size_t...I>
    void GetCoreSampleExtrema(
        Extrema &result,
        Index row,
        Index column,
        ExtremaIndices *indices,
        std::index_sequence<I...> planeIndices) const
    {
        Eigen::Vector<T, int(sizeof...(I))> coreSample =
            this->GetVector(row, column, planeIndices);

        if (indices)
        {
            result.planes[0](row, column) =
                coreSample.minCoeff(&indices->planes[0](row, column));

            result.planes[1](row, column) =
                coreSample.maxCoeff(&indices->planes[1](row, column));
        }
        else
        {
            result.planes[0](row, column) = coreSample.minCoeff();
            result.planes[1](row, column) = coreSample.maxCoeff();
        }
    }

    void GetCoreSampleExtrema(
        Extrema &result,
        Index row,
        Index column,
        ExtremaIndices *indices) const
    {
        this->GetCoreSampleExtrema(
            result,
            row,
            column,
            indices,
            std::make_index_sequence<count>());
    }

    Extrema GetExtrema(ExtremaIndices *indices = nullptr) const
    {
        return this->GetExtrema(std::make_index_sequence<count>(), indices);
    }

    template<size_t...I>
    Extrema GetExtrema(
        std::index_sequence<I...> planeIndices,
        ExtremaIndices *indices = nullptr) const
    {
        Extrema result(this->GetRowCount(), this->GetColumnCount());

        if constexpr (options == Eigen::RowMajor)
        {
            for (Index row = 0; row < this->GetRowCount(); ++row)
            {
                for (
                    Index column = 0;
                    column < this->GetColumnCount();
                    ++column)
                {
                    this->GetCoreSampleExtrema(
                        result,
                        row,
                        column,
                        indices,
                        planeIndices);
                }
            }
        }
        else
        {
            // Column-major

            for (Index column = 0; column < this->GetColumnCount(); ++column)
            {
                for ( Index row = 0; row < this->GetRowCount(); ++row)
                {
                    this->GetCoreSampleExtrema(
                        result,
                        row,
                        column,
                        indices,
                        planeIndices);
                }
            }
        }

        return result;
    }

    template<size_t precision>
    void Round()
    {
        this->template DoRound_<precision>(std::make_index_sequence<count>());
    }

    Index GetRowCount() const
    {
        return std::get<0>(this->planes).rows();
    }

    Index GetColumnCount() const
    {
        return std::get<0>(this->planes).cols();
    }

    template<typename Derived>
    static Planar FromInterleaved(
        const Eigen::DenseBase<Derived> &interleaved,
        Index rowCount = rows,
        Index columnCount = columns)
    {
        static_assert(std::is_same_v<typename Derived::Scalar, T>);

        if (rows == Eigen::Dynamic)
        {
            if (rowCount <= 0)
            {
                throw PlanarError("Dynamic rowCount must be specified");
            }
        }

        if (columns == Eigen::Dynamic)
        {
            if (columnCount <= 0)
            {
                throw PlanarError("Dynamic columnCount must be specified");
            }
        }

        Index channelCount;
        Index pixelCount;
        bool channelsInColumns;

        if (interleaved.rows() > interleaved.cols())
        {
            pixelCount = interleaved.rows();
            channelCount = interleaved.cols();
            channelsInColumns = true;
        }
        else
        {
            pixelCount = interleaved.cols();
            channelCount = interleaved.rows();
            channelsInColumns = false;
        }

        Index expectedPixelCount = rowCount * columnCount;

        if (expectedPixelCount != pixelCount)
        {
            throw PlanarError("pixel count mismatch");
        }

        if (channelCount != Index(count))
        {
            throw PlanarError("interleaved channelCount mismatch");
        }

        Planar result(rowCount, columnCount);

        if (channelsInColumns)
        {
            for (Index i = 0; i < channelCount; ++i)
            {
                result.planes.at(size_t(i)) =
                    interleaved.col(i)
                        .template reshaped<options>(rowCount, columnCount);
            }
        }
        else
        {
            // channels are in rows
            for (Index i = 0; i < channelCount; ++i)
            {
                result.planes.at(size_t(i)) =
                    interleaved.row(i)
                        .template reshaped<options>(rowCount, columnCount);
            }
        }

        return result;
    }

    template<int resultOptions = options>
    auto GetInterleaved() const
    {
        auto size = std::get<0>(this->planes).size();

        for (auto &plane: planes)
        {
            if (plane.size() != size)
            {
                throw PlanarError(
                    "Planes to interleave have mismatched sizes.");
            }
        }

        static constexpr bool isColumnMajor =
            (resultOptions == Eigen::ColMajor);

        using Traits = MatrixTraits<Matrix>;

        static constexpr int resultRows = static_cast<int>(
            isColumnMajor ? static_cast<int>(count) : Traits::size);

        static constexpr int resultColumns = static_cast<int>(
            isColumnMajor ? Traits::size : static_cast<int>(count));

        using Result =
            Eigen::Matrix<T, resultRows, resultColumns, resultOptions>;

        Result result;

        if constexpr (isColumnMajor)
        {
            result = Result(count, size);
        }
        else
        {
            result = Result(size, count);
        }

#if defined _MSC_VER
        // MSVC is confused by correct C++ syntax...
        this->template Planar::Interleave_(
            result,
            std::make_index_sequence<count>{});
#else
        // Clang and GCC are not
        this->template Interleave_(result, std::make_index_sequence<count>{});
#endif

        return result;
    }

    template<typename U>
    auto Cast() const
    {
        return this->template Cast_<U>(std::make_index_sequence<count>{});
    }

    Planar & operator *=(T scalar)
    {
        return this->template MultiplyAssign_(
            std::make_index_sequence<count>{},
            scalar);
    }

    Planar & operator /=(T scalar)
    {
        return this->template DivideAssign_(
            std::make_index_sequence<count>{},
            scalar);
    }

    Planar & operator +=(T scalar)
    {
        return this->template AddAssign_(
            std::make_index_sequence<count>{},
            scalar);
    }

    Planar & operator -=(T scalar)
    {
        return this->template SubtractAssign_(
            std::make_index_sequence<count>{},
            scalar);
    }

    Planar operator *(T scalar) const
    {
        Planar result(*this);
        result *= scalar;

        return result;
    }

    Planar operator /(T scalar) const
    {
        Planar result(*this);
        result /= scalar;

        return result;
    }

    Planar operator +(T scalar) const
    {
        Planar result(*this);
        result += scalar;

        return result;
    }

    Planar operator -(T scalar) const
    {
        Planar result(*this);
        result -= scalar;

        return result;
    }

    template<size_t index, typename Other>
    void AssignCast(const Other &other)
    {
        std::get<index>(this->planes) =
            std::get<index>(other.planes).template cast<T>();
    }

    template<size_t index>
    void MultiplyAssign(T scalar)
    {
        std::get<index>(this->planes).array() *= scalar;
    }

    template<size_t index>
    void DivideAssign(T scalar)
    {
        std::get<index>(this->planes).array() /= scalar;
    }

    template<size_t index>
    void AddAssign(T scalar)
    {
        std::get<index>(this->planes).array() += scalar;
    }

    template<size_t index>
    void ZeroPlane(const Size<Index> &size)
    {
        std::get<index>(this->planes) = Matrix::Zero(size.height, size.width);
    }

    template<size_t index>
    void AssignMiddle(const Planar &source, Index startRow, Index startColumn)
    {
        auto size = source.GetSize();

        std::get<index>(this->planes)
            .block(startRow, startColumn, size.height, size.width)
                = std::get<index>(source.planes);
    }

    template<size_t index>
    void SubtractAssign(T scalar)
    {
        std::get<index>(this->planes).array() -= scalar;
    }

    template<size_t index>
    void ConstrainPlane(T minimum, T maximum)
    {
        ::tau::Constrain(std::get<index>(this->planes), minimum, maximum);
    }

    static Planar Zero(const Size<Index> &size)
    {
        Planar result;

        result.template Zero_(std::make_index_sequence<count>{}, size);

        return result;
    }

    static Planar Zero(Index height, Index width)
    {
        return Zero({height, width});
    }

    template<typename U>
    using PlanarLike = Planar<count, U, rows, columns, options>;

    template<typename U = T>
    PlanarLike<U> PadZeros(const Size<Index> &paddedSize) const
    {
        // We are growing the rows and columns.
        // This could be down with compile-time sizes, but has been omitted
        // here for simplicity.
        // Run-time sizes only.
        static_assert(rows == Eigen::Dynamic);
        static_assert(columns == Eigen::Dynamic);

        using Result = PlanarLike<U>;
        auto size = this->GetSize();
        assert(paddedSize.height >= size.height);
        assert(paddedSize.width >= size.width);
        auto offsets = (paddedSize - size).ToPoint() / 2;

        Result result = Result::Zero(paddedSize);

        if constexpr (!std::is_same_v<T, U>)
        {
            result.template AssignMiddle_(
                std::make_index_sequence<count>{},
                this->template Cast<U>(),
                offsets.y,
                offsets.x);
        }
        else
        {
            result.template AssignMiddle_(
                std::make_index_sequence<count>{},
                *this,
                offsets.y,
                offsets.x);
        }

        return result;
    }

    template<typename U = T>
    PlanarLike<U> PadZeros(Index height, Index width)
    {
        return this->template PadZeros<U>(Size<Index>(width, height));
    }

    void Constrain(T minimum, T maximum)
    {
        this->template Constrain_(
            std::make_index_sequence<count>{},
            minimum,
            maximum);
    }

    std::ostream & ToStream(std::ostream &outputStream) const
    {
        for (auto i: jive::Range<size_t>(0, this->planes.size()))
        {
            outputStream << "plane " << i << ":\n"
                << this->planes[i] << std::endl;
        }

        return outputStream;
    }

    Size<Index> GetSize() const
    {
        return {{
            std::get<0>(this->planes).cols(),
            std::get<0>(this->planes).rows()}};
    }

private:

    template<typename U, size_t ... I>
    auto Cast_(std::index_sequence<I ...>) const
    {
        using Result = Planar<count, U, rows, columns, options>;

        Result result;

        (result.template AssignCast<I>(*this), ...);

        return result;
    }

    template<size_t ... I>
    Planar & MultiplyAssign_(std::index_sequence<I ...>, T scalar)
    {
        (this->template MultiplyAssign<I>(scalar), ...);

        return *this;
    }

    template<size_t ... I>
    Planar & DivideAssign_(std::index_sequence<I ...>, T scalar)
    {
        (this->template DivideAssign<I>(scalar), ...);

        return *this;
    }

    template<size_t ... I>
    Planar & AddAssign_(std::index_sequence<I ...>, T scalar)
    {
        (this->template AddAssign<I>(scalar), ...);

        return *this;
    }

    template<size_t ... I>
    Planar & Zero_(
        std::index_sequence<I ...>,
        const Size<Index> size)
    {
        (this->template ZeroPlane<I>(size), ...);

        return *this;
    }

    template<size_t ... I>
    Planar & AssignMiddle_(
        std::index_sequence<I ...>,
        const Planar &source,
        Index startRow,
        Index startColumn)
    {
        (this->template AssignMiddle<I>(source, startRow, startColumn), ...);

        return *this;
    }

    template<size_t ... I>
    Planar & SubtractAssign_(std::index_sequence<I ...>, T scalar) const
    {
        (this->template SubtractAssign<I>(scalar), ...);

        return *this;
    }

    template<size_t ... I>
    void Constrain_(
        std::index_sequence<I ...>,
        T minimum,
        T maximum)
    {
        (this->template ConstrainPlane<I>(minimum, maximum), ...);
    }

    template<typename Result, size_t...I>
    void Interleave_(Result &result, std::index_sequence<I...>) const
    {
        using Traits = MatrixTraits<Result>;

        auto size = std::get<0>(this->planes).size();

        if constexpr (Traits::isColumnMajor)
        {
            assert(result.rows() == count);
            assert(result.cols() == size);

            Fill(
                result,
                std::get<I>(this->planes)
                    .template reshaped<Eigen::AutoOrder>(1, size)...);
        }
        else
        {
            assert(result.rows() == size);
            assert(result.cols() == count);

            Fill(
                result,
                std::get<I>(this->planes)
                    .template reshaped<Eigen::AutoOrder>(size, 1)...);
        }
    }

    template<typename Result, size_t...I>
    void GetVector_(
        Result &result,
        Index row,
        Index column,
        std::index_sequence<I...>) const
    {
        Fill(
            result,
            std::get<I>(this->planes)(row, column)...);
    }

    template<typename Result, size_t...I>
    void GetValue_(
        Result &result,
        Index row,
        Index column,
        std::index_sequence<I...>) const
    {
        ((std::get<I>(result.planes)(0, 0) =
            std::get<I>(this->planes)(row, column)), ...);
    }

    template<size_t precision, size_t...I>
    void DoRound_(std::index_sequence<I...>)
    {
        static_assert(
            jive::Power<10, precision>() <= std::numeric_limits<T>::max());

        static constexpr T rounder =
            static_cast<T>(jive::Power<10, precision>());

        ((
            std::get<I>(this->planes) =
                (std::get<I>(this->planes).array() * rounder).round()
                    / rounder),
             ...);
    }
};


template
<
    size_t count,
    typename T,
    int rows,
    int columns,
    int options
>
std::ostream & operator<<(
    std::ostream &outputStream,
    const Planar<count, T, rows, columns, options> &planar)
{
    return planar.ToStream(outputStream);
}


template<typename P, typename U>
using PlanarLike = Planar<P::count, U, P::rows, P::columns, P::options>;


template<typename T>
struct IsPlanar_: std::false_type {};


template
<
    size_t count_,
    typename T,
    int rows_,
    int columns_
>
struct IsPlanar_<Planar<count_, T, rows_, columns_>>
    :
    std::true_type
{

};


template
<
    size_t count_,
    typename T,
    int rows_,
    int columns_,
    int options_
>
struct IsPlanar_<Planar<count_, T, rows_, columns_, options_>>
    :
    std::true_type
{

};


template<typename T>
inline constexpr bool IsPlanar = IsPlanar_<T>::value;


} // end namespace tau
