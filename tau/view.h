#pragma once


#include <tau/eigen.h>
#include <concepts>
#include <type_traits>


namespace tau
{


template<typename T>
using Clean = std::remove_cvref_t<T>;


template<typename Derived>
using View = Eigen::Ref
<
    Eigen::Matrix
    <
        typename Derived::Scalar,
        Derived::RowsAtCompileTime,
        Derived::ColsAtCompileTime,
        Derived::IsRowMajor ? Eigen::RowMajor : Eigen::ColMajor
    >
>;

template<typename Derived>
using ConstView = Eigen::Ref
<
    const Eigen::Matrix
    <
        typename Derived::Scalar,
        Derived::RowsAtCompileTime,
        Derived::ColsAtCompileTime,
        Derived::IsRowMajor ? Eigen::RowMajor : Eigen::ColMajor
    >
>;


template<typename T>
struct IsEigenRef_: std::false_type {};

template<typename Derived>
struct IsEigenRef_<Eigen::Ref<Derived>>: std::true_type {};

template<typename T>
concept IsEigenRef = IsEigenRef_<Clean<T>>::value;


template<typename T>
struct RefTraits;


template<typename PlainObjectType_, int Options_, typename StrideType_>
struct RefTraits<Eigen::Ref<PlainObjectType_, Options_, StrideType_>>
    :
    MatrixTraits<std::remove_cvref_t<PlainObjectType_>>
{
    using PlainObjectType = PlainObjectType_;
    using Scalar = typename PlainObjectType::Scalar;
    static constexpr int Options = Options_;
    using StrideType = StrideType_;
};


template<typename T>
concept IsEigenConstRef =
    IsEigenRef<T>
    && std::is_const_v<typename RefTraits<T>::PlainObjectType>;


template<IsEigenRef T>
using RefScalar = typename RefTraits<T>::Scalar;


template<typename T>
concept Viewable =
    std::derived_from<Clean<T>, Eigen::MatrixBase<Clean<T>>> &&
    (std::constructible_from<View<Clean<T>>, T>
     || std::constructible_from<ConstView<Clean<T>>, T>);


template<typename Expr, typename = void>
struct IsWritableExpr_ : std::false_type {};

template<typename Expr>
struct IsWritableExpr_
<
    Expr,
    std::void_t
    <
        decltype(
            std::declval<Expr>().coeffRef(
                Eigen::Index{0},
                Eigen::Index{0}
            ) = std::declval<typename Clean<Expr>::Scalar>()
        )
    >
> : std::true_type {};

template<typename Expr>
inline constexpr bool IsWritableExpr = IsWritableExpr_<Expr>::value;


template<Viewable Expr>
auto MakeView(Expr &&expr)
{
    // Don't forward expr: we want it as an lvalue for Eigen::Ref

    using C = Clean<Expr>;

    if constexpr (IsWritableExpr<Expr>)
    {
        return View<C>(expr);
    }
    else
    {
        return ConstView<C>(expr);
    }
}


} // end namespace tau
