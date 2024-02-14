#pragma once


namespace tau
{


template<typename T, typename Enable = void>
struct HasHorizontal_: std::false_type {};

template<typename T>
struct HasHorizontal_
<
    T,
    std::void_t<decltype(std::declval<T>().Horizontal())>
>: std::true_type {};

template<typename T>
inline constexpr bool HasHorizontal = HasHorizontal_<T>::value;


template<typename T, typename Enable = void>
struct HasVertical_: std::false_type {};

template<typename T>
struct HasVertical_
<
    T,
    std::void_t<decltype(std::declval<T>().Vertical())>
>: std::true_type {};

template<typename T>
inline constexpr bool HasVertical = HasVertical_<T>::value;


template<typename T, typename Enable = void>
struct HasOrthogonals_: std::false_type {};

template<typename T>
struct HasOrthogonals_
<
    T,
    std::enable_if_t<HasHorizontal<T> && HasVertical<T>>
>: std::true_type {};

template<typename T>
inline constexpr bool HasOrthogonals =
    HasOrthogonals_<T>::value;


template<typename Left, typename Right>
inline constexpr bool HaveOrthogonals =
    HasOrthogonals<Left> && HasOrthogonals<Right>;


template
<
    typename T,
    typename Left,
    typename Right
>
std::enable_if_t<(HaveOrthogonals<Left, Right>), Left &>
operator*=(
    BasicArithmetic<T, Left> &left,
    const BasicArithmetic<T, Right> &right)
{
    auto &result = left.Upcast();
    const auto &rightDerived = right.Upcast();

    result.Horizontal() *= rightDerived.Horizontal();
    result.Vertical() *= rightDerived.Vertical();

    return result;
}


template
<
    typename T,
    typename Left,
    typename Right
>
std::enable_if_t<(HaveOrthogonals<Left, Right>), Left &>
operator/=(
    BasicArithmetic<T, Left> &left,
    const BasicArithmetic<T, Right> &right)
{
    auto &result = left.Upcast();
    const auto &rightDerived = right.Upcast();

    assert(rightDerived.Vertical() != 0);
    assert(rightDerived.Horizontal() != 0);

    result.Horizontal() /= rightDerived.Horizontal();
    result.Vertical() /= rightDerived.Vertical();

    return result;
}


template
<
    typename T,
    typename Left,
    typename Right
>
std::enable_if_t<(HaveOrthogonals<Left, Right>), Left &>
operator+=(
    BasicArithmetic<T, Left> &left,
    const BasicArithmetic<T, Right> &right)
{
    auto &result = left.Upcast();
    const auto &rightDerived = right.Upcast();

    result.Horizontal() += rightDerived.Horizontal();
    result.Vertical() += rightDerived.Vertical();

    return result;
}


template
<
    typename T,
    typename Left,
    typename Right
>
std::enable_if_t<(HaveOrthogonals<Left, Right>), Left &>
operator-=(
    BasicArithmetic<T, Left> &left,
    const BasicArithmetic<T, Right> &right)
{
    auto &result = left.Upcast();
    const auto &rightDerived = right.Upcast();

    result.Horizontal() -= rightDerived.Horizontal();
    result.Vertical() -= rightDerived.Vertical();

    return result;
}


template
<
    typename T,
    typename Left,
    typename Right
>
std::enable_if_t<(HaveOrthogonals<Left, Right>), Left>
operator*(
    const BasicArithmetic<T, Left> &left,
    const BasicArithmetic<T, Right> &right)
{
    // Make a copy
    auto leftDerived = left.Upcast();

    // Call the in-place operator
    leftDerived *= right;

    return leftDerived;
}


template
<
    typename T,
    typename Left,
    typename Right
>
std::enable_if_t<(HaveOrthogonals<Left, Right>), Left>
operator/(
    const BasicArithmetic<T, Left> &left,
    const BasicArithmetic<T, Right> &right)
{
    // Make a copy
    auto leftDerived = left.Upcast();

    // Call the in-place operator
    leftDerived /= right;

    return leftDerived;
}


template
<
    typename T,
    typename Left,
    typename Right
>
std::enable_if_t<(HaveOrthogonals<Left, Right>), Left>
operator+(
    const BasicArithmetic<T, Left> &left,
    const BasicArithmetic<T, Right> &right)
{
    // Make a copy
    auto leftDerived = left.Upcast();

    // Call the in-place operator
    leftDerived += right;

    return leftDerived;
}


template
<
    typename T,
    typename Left,
    typename Right
>
std::enable_if_t<(HaveOrthogonals<Left, Right>), Left>
operator-(
    const BasicArithmetic<T, Left> &left,
    const BasicArithmetic<T, Right> &right)
{
    // Make a copy
    auto leftDerived = left.Upcast();

    // Call the in-place operator
    leftDerived -= right;

    return leftDerived;
}


} // end namespace tau
