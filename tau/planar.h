#pragma once

#include <jive/power.h>
#include <jive/range.h>
#include "tau/eigen.h"


namespace tau
{


template
<
    size_t count,
    typename T,
    int rows,
    int columns,
    int options = Eigen::ColMajor
>
class Planar
{
public:
    using Matrix = Eigen::Matrix<T, rows, columns, options>;
    using Index = Eigen::Index;

    std::array<Matrix, count> planes;

    Planar() = default;

    Planar(Index rowCount, Index columnCount)
    {
        for (auto &plane: this->planes)
        {
            plane = Matrix(rowCount, columnCount);
        }
    }

    using Value = Planar<count, T, 1, 1, options>;

    Value operator()(Index row, Index column) const
    {
        Value result{};

        this->template GetValue_(
            result,
            row,
            column,
            std::make_index_sequence<count>{});

        return result;
    }

    Eigen::Vector<T, count> GetVector(Index row, Index column) const
    {
        Eigen::Vector<T, count> result;

        this->template GetVector_(
            result,
            row,
            column,
            std::make_index_sequence<count>{});

        return result;
    }

    // First plane contains minima, and second plane contains maxima.
    using Extrema = Planar<2, T, rows, columns, options>;

    // First plane contains the plane index of the minima.
    // Second plane contains the plane index of the maxima.
    using ExtremaIndices = Planar<2, Index, rows, columns, options>;

    void GetCoreSampleExtrema(
        Extrema &result,
        Index row,
        Index column,
        ExtremaIndices *indices) const
    {
        Eigen::Vector<T, count> coreSample = GetVector(row, column);

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

    Extrema GetExtrema(ExtremaIndices *indices = nullptr) const
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
                    GetCoreSampleExtrema(result, row, column, indices);
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
                    GetCoreSampleExtrema(result, row, column, indices);
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

    Eigen::Index GetRowCount() const
    {
        return std::get<0>(this->planes).rows();
    }

    Eigen::Index GetColumnCount() const
    {
        return std::get<0>(this->planes).cols();
    }

    template<int resultOptions = options>
    auto GetInterleaved() const
    {
        auto size = std::get<0>(this->planes).size();

        for (auto &plane: planes)
        {
            if (plane.size() != size)
            {
                throw TauError("Planes to interleave have mismatched sizes.");
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

#if defined _MSC_VER && _MSC_VER <= 1933
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

    template<size_t index, typename Other>
    void AssignCast(const Other &other)
    {
        std::get<index>(this->planes) =
            std::get<index>(other.planes).template cast<T>();
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

private:

    template<typename U, size_t ... I>
    auto Cast_(std::index_sequence<I ...>) const
    {
        using Result = Planar<count, U, rows, columns, options>;

        Result result;
        
        (result.template AssignCast<I>(*this), ...);

        return result;
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
        static constexpr T rounder = jive::Power<10, precision>();

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


} // end namespace tau
