/**
  * @file eigen.h
  *
  * @brief A few convenience functions for working with Eigen.
  *
  * @author Jive Helix (jivehelix@gmail.com)
  * @date 19 May 2020
  * Licensed under the MIT license. See LICENSE file.
**/

#pragma once

#include <type_traits>
#include <numeric>
#include "Eigen/Dense"
#include "jive/type_traits.h"
#include "jive/future.h"


namespace tau
{


template<typename>
struct IsMatrix: std::false_type {};

template<typename T, int rows, int columns>
struct IsMatrix<Eigen::Matrix<T, rows, columns>>: std::true_type {};

// An Eigen::Map behaves like a Matrix.
// Use MatrixTraits to test whether it is an Eigen::Matrix or Eigen::Map.
template<typename PlainMatrix, int mapOptions, typename Stride>
struct IsMatrix<Eigen::Map<PlainMatrix, mapOptions, Stride>>: std::true_type {};


template<typename T, typename Enable = void>
struct ValueType
{
    using type = T;
};


template<typename T>
struct ValueType<T, std::enable_if_t<jive::IsValueContainer<T>::value>>
{
    using type = typename T::value_type;
};


template<typename T_>
struct MatrixTraits
{
    using T = std::remove_cvref_t<T_>;

    static_assert(
        std::is_arithmetic_v<T> || jive::IsValueContainer<T>::value,
        "Expected an arithmetic type or a value container.");

    static constexpr bool isMatrix = false;
    static constexpr bool isMap = false;
    static constexpr bool isDynamic = jive::IsValueContainer<T>::value;
    static constexpr bool isFullDynamic = isDynamic;
    using type = typename ValueType<T>::type;
};


template<
    typename T_,
    int rows_,
    int columns_,
    int options_,
    int maxRows_,
    int maxColumns_>
struct MatrixTraits<
    Eigen::Matrix<T_, rows_, columns_, options_, maxRows_, maxColumns_>>
{
    using type = T_;
    static constexpr int rows = rows_;
    static constexpr int columns = columns_;
    static constexpr int options = options_;
    static constexpr int maxRows = maxRows_;
    static constexpr int maxColumns = maxColumns_;
    static constexpr bool isDynamic = (
        columns == Eigen::Dynamic
        || rows == Eigen::Dynamic);

    static constexpr bool isFullDynamic = (
        columns == Eigen::Dynamic
        && rows == Eigen::Dynamic);

    static constexpr bool isVector = (rows == 1) || (columns == 1);
    static constexpr bool isRowVector = (rows == 1);
    static constexpr bool isColumnVector = (columns == 1);
    static constexpr bool isRowMajor = (options & Eigen::RowMajor); 
    static constexpr bool isColumnMajor = !(options & Eigen::RowMajor); 
    static constexpr int size = isDynamic ? -1 : rows * columns;

    static constexpr bool isMatrix = true;
    static constexpr bool isMap = false;
};


template<typename PlainMatrix, int mapOptions, typename Stride>
struct MatrixTraits<Eigen::Map<PlainMatrix, mapOptions, Stride>>
    : MatrixTraits<PlainMatrix>
{
    static constexpr bool isMap = true;
};


template<typename Derived>
struct MatrixTraits<Eigen::DenseBase<Derived>>: MatrixTraits<Derived> {};


template<typename T, typename MatrixType>
using MatrixLike = Eigen::Matrix<
    T,
    MatrixTraits<MatrixType>::rows,
    MatrixTraits<MatrixType>::columns,
    MatrixTraits<MatrixType>::options,
    MatrixTraits<MatrixType>::maxRows,
    MatrixTraits<MatrixType>::maxColumns>;

/** Eigen 3.4.0 does not support compile time checking of fixed-size array
 ** initialization. Brace and comma initialization
 ** are unsafe because they do not fail until runtime when Eigen discovers an
 ** incorrect number of arguments.
 **
 ** This helper allows comma initialization to be used as a one-liner, and
 ** provides a compile time check on the argument count.
 **/
template<int rows, int columns, typename T, typename ...Values>
Eigen::Matrix<std::remove_cvref_t<T>, rows, columns>
Matrix(T &&first, Values &&...values)
{
    static_assert(sizeof...(Values) + 1 == rows * columns);

    using type = std::remove_cvref_t<T>;

    using Common =
        std::common_type_t<
            type,
            std::remove_cvref_t<Values>...>;

    Eigen::Matrix<Common, rows, columns> result;
    ((result << std::forward<T>(first)), ..., std::forward<Values>(values));
    return result.template cast<type>();
}


template<int items, typename T, typename ...Values>
Eigen::Matrix<std::remove_cvref_t<T>, items, 1>
Vector(T &&first, Values &&...values)
{
    return Matrix<items, 1, T>(
        std::forward<T>(first),
        std::forward<Values>(values)...);
}


template<typename... Values>
auto Vector(Values &&...values)
{
    using T = std::common_type_t<Values...>;
    return Matrix<sizeof...(Values), 1, T>(std::forward<Values>(values)...);
}


template<int items, typename T, typename ...Values>
Eigen::Matrix<std::remove_cvref_t<T>, 1, items>
RowVector(T &&first, Values &&...values)
{
    return Matrix<1, items, T>(
        std::forward<T>(first),
        std::forward<Values>(values)...);
}


template<typename... Values>
auto RowVector(Values &&...values)
{
    using T = std::common_type_t<Values...>;
    return Matrix<1, sizeof...(Values), T>(std::forward<Values>(values)...);
}


template<typename T>
Eigen::VectorX<std::remove_cvref_t<T>> VectorRange(T start, size_t count)
{
    using ValueType = std::remove_cvref_t<T>;
    Eigen::VectorX<ValueType> result(count);
    std::iota(result.begin(), result.end(), start);
    return result;
}


template<typename T>
bool IsRowVector(const T &matrix)
{
    using traits = MatrixTraits<T>;
    
    if constexpr (traits::isDynamic)
    {
        return matrix.rows() == 1;
    }
    else
    {
        return traits::isRowVector;
    }
}


template<typename T>
bool IsColumnVector(const T &matrix)
{
    using traits = MatrixTraits<T>;
    
    if constexpr (traits::isDynamic)
    {
        return matrix.cols() == 1;
    }
    else
    {
        return traits::isColumnVector;
    }
}


template<typename T>
bool IsVector(const T &matrix)
{
    using traits = MatrixTraits<T>;
    
    if constexpr (traits::isDynamic)
    {
        return matrix.cols() == 1 || matrix.rows() == 1;
    }
    else
    {
        return traits::isVector;
    }
}




/**
 ** Numpy, Matlab, and others support fancy indexing, where
 **     myArray[myArray > 2.0] = 2.0
 ** will replace any values in myArray that are greater than 2 with 2.
 **
 ** To achieve the same in Eigen requires
 **     myArray = (myArray > 2.0).select(2.0, myArray);
 ** which feels awkward to use.
 **
 ** This adapter doesn't match the common syntax exactly, but expresses the
 ** intent more clearly:
 **     Select(myArray) > 2.0 = 2.0;
 **
 ** All six comparison operators are supported. Perfect forwarding is used for
 ** zero overhead.
 **/
template<typename MatrixType>
class Select
{
public:
    Select(MatrixType &matrix): matrix_(matrix) {}

    template<typename Value>
    auto operator>(Value &&value)
    {
        return this->MakeAssigner(
            this->matrix_.array() > std::forward<Value>(value));
    }

    template<typename Value>
    auto operator<(Value &&value)
    {
        return this->MakeAssigner(
            this->matrix_.array() < std::forward<Value>(value));
    }

    template<typename Value>
    auto operator<=(Value &&value)
    {
        return this->MakeAssigner(
            this->matrix_.array() <= std::forward<Value>(value));
    }

    template<typename Value>
    auto operator>=(Value &&value)
    {
        return this->MakeAssigner(
            this->matrix_.array() >= std::forward<Value>(value));
    }

    template<typename Value>
    auto operator==(Value &&value)
    {
        return this->MakeAssigner(
            this->matrix_.array() == std::forward<Value>(value));
    }

    template<typename Value>
    auto operator!=(Value &&value)
    {
        return this->MakeAssigner(
            this->matrix_.array() != std::forward<Value>(value));
    }

private:
    template<typename Compare>
    class Assigner
    {
    public:
        // Add rvalue referent to Compare to force the forwarding reference.
        // Constructors do not have template arguments, so we cannot use
        // Compare &&compare
        // like we would in a templated function.
        Assigner(
                MatrixType &matrix,
                std::add_rvalue_reference_t<Compare> compare)
            :
            matrix_(matrix),
            compare_(std::forward<Compare>(compare))
        {

        }

        template<typename Value>
        MatrixType & operator=(Value && value)
        {
            return this->matrix_ = this->compare_.select(
                std::forward<Value>(value),
                this->matrix_);
        }

        MatrixType &matrix_;
        Compare compare_;
    };

    template<typename Compare>
    Assigner<Compare> MakeAssigner(Compare &&compare)
    {
        return Assigner<Compare>(this->matrix_, std::forward<Compare>(compare));
    }

private:
    MatrixType &matrix_;
};


template<size_t alignment>
constexpr auto GetEigenAlignment()
{
    if constexpr (alignment == 8)
    {
        return Eigen::Aligned8;
    }
    else if constexpr (alignment == 16)
    {
        return Eigen::Aligned16;
    }
    else if constexpr (alignment == 32)
    {
        return Eigen::Aligned32;
    }
    else if constexpr (alignment == 64)
    {
        return Eigen::Aligned64;
    }
    else if constexpr (alignment == 128)
    {
        return Eigen::Aligned128;
    }
    else
    {
        return Eigen::Unaligned;
    }
}


} // end namespace tau
