/**
  * @file stack.h
  *
  * @brief Stack matrices vertically or horizontally.
  *
  * @author Jive Helix (jivehelix@gmail.com)
  * @date 23 May 2022
  * @copyright Jive Helix
  * Licensed under the MIT license. See LICENSE file.
**/

#pragma once

#include "tau/eigen.h"


namespace tau
{


template<typename Left, typename Right>
auto HorizontalStack(const Left &left, const Right &right)
{
    assert(left.rows() == right.rows());

    using LeftTraits = tau::MatrixTraits<Left>;
    using RightTraits = tau::MatrixTraits<Right>;

    if constexpr (
            LeftTraits::rows != Eigen::Dynamic
            && RightTraits::rows != Eigen::Dynamic)
    {
        static_assert(LeftTraits::rows == RightTraits::rows);
    }

    using Result =
        Eigen::Matrix
        <
            typename LeftTraits::type,
            LeftTraits::rows,
            (LeftTraits::columns == Eigen::Dynamic
                || RightTraits::columns == Eigen::Dynamic)
                ? Eigen::Dynamic
                : LeftTraits::columns + RightTraits::columns,
            LeftTraits::options
        >;

    Result result(left.rows(), left.cols() + right.cols());
    result << left, right;

    return result;
}


template<typename Top, typename Bottom>
auto VerticalStack(const Top &top, const Bottom &bottom)
{
    assert(top.cols() == bottom.cols());

    using TopTraits = tau::MatrixTraits<Top>;
    using BottomTraits = tau::MatrixTraits<Bottom>;

    if constexpr (
            TopTraits::rows != Eigen::Dynamic
            && BottomTraits::rows != Eigen::Dynamic)
    {
        static_assert(TopTraits::columns == BottomTraits::columns);
    }

    using Result =
        Eigen::Matrix
        <
            typename TopTraits::type,
            (TopTraits::rows == Eigen::Dynamic
                || BottomTraits::rows == Eigen::Dynamic)
                ? Eigen::Dynamic
                : TopTraits::rows + BottomTraits::rows,
            TopTraits::columns,
            TopTraits::options
        >;

    Result result(top.rows() + bottom.rows(), top.cols());
    result << top, bottom;

    return result;
}


} // end namespace tau
