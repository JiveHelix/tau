#pragma once


namespace tau
{


namespace literals
{


constexpr float operator"" _f(long double value)
{
    return static_cast<float>(value);
}

constexpr float operator"" _f(unsigned long long value)
{
    return static_cast<float>(value);
}

constexpr double operator"" _d(long double value)
{
    return static_cast<double>(value);
}

constexpr double operator"" _d(unsigned long long value)
{
    return static_cast<double>(value);
}

constexpr long double operator"" _ld(long double value)
{
    return value;
}

constexpr long double operator"" _ld(unsigned long long value)
{
    return static_cast<long double>(value);
}


} // end namespace literals


} // end namespace tau
