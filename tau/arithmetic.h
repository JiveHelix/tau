#pragma once

#include <type_traits>
#include <cmath>
#include <fields/fields.h>
#include <jive/for_each.h>
#include <jive/zip_apply.h>
#include <jive/overflow.h>


namespace tau
{

struct Round {};
struct Floor {};
struct Ceil {};

template<typename T, typename V, typename Style>
std::enable_if_t<std::is_integral_v<T>, T>
DoConvert(V value)
{
    assert(jive::CheckConvertible<T>(value));

    if constexpr (std::is_floating_point_v<V>)
    {
        if constexpr (std::is_same_v<Style, Round>)
        {
            return static_cast<T>(std::round(value));
        }
        else if constexpr (std::is_same_v<Style, Floor>)
        {
            return static_cast<T>(std::floor(value));
        }
        else if constexpr (std::is_same_v<Style, Ceil>)
        {
            return static_cast<T>(std::ceil(value));
        }
        else
        {
            return static_cast<T>(value);
        }
    }
    else
    {
        return static_cast<T>(value);
    }
}


template<typename T, typename V, typename>
std::enable_if_t<std::is_floating_point_v<T>, T>
DoConvert(V value)
{
    // Converting to a floating-point value.
    // No rounding is necessary.
    assert(jive::CheckConvertible<T>(value));

    return static_cast<T>(value);
}


template<typename T, typename V>
T Convert(V value)
{
    return DoConvert<T, V, Round>(value);
}


template
<
    template<typename> typename Fields,
    typename Result,
    typename T,
    typename Style,
    typename Source
>
Result ConvertFields(const Source &source)
{
    Result result;

    auto convert = [&result, &source] (auto sourceField, auto resultField)
    {
        using Member = std::remove_reference_t<
            decltype(source.*(sourceField.member))>;

        result.*(resultField.member) =
            DoConvert<T, Member, Style>(source.*(sourceField.member));
    };

    jive::ZipApply(convert, Fields<Source>::fields, Fields<Result>::fields);

    return result;
}


template
<
    typename T,
    template<typename> typename Fields,
    template<typename> typename Derived
>
struct Arithmetic
{
    using This = Derived<T>;

    This & Upcast()
    {
        return *static_cast<This *>(this);
    }

    const This & Upcast() const
    {
        return *static_cast<const This *>(this);
    }

    template<typename U, typename Style = Round>
    auto Convert() const
    {
        return ConvertFields<Fields, Derived<U>, U, Style>(this->Upcast());
    }

    /***** Element-wise operators *****/
    // TODO: Add assertions (or exceptions?) when operators cause overflow.
    This & operator+=(const This &other)
    {
        auto & self = this->Upcast();

        auto add = [&self, &other] (auto field)
        {
            self.*(field.member) += other.*(field.member);
        };

        jive::ForEach(
            Fields<This>::fields,
            add);

        return self;
    }

    This operator+(const This &other) const
    {
        auto result = this->Upcast();
        return result += other;
    }

    This & operator-=(const This &other)
    {
        auto & self = this->Upcast();

        auto subtract = [&self, &other] (auto field)
        {
            self.*(field.member) -= other.*(field.member);
        };

        jive::ForEach(
            Fields<This>::fields,
            subtract);

        return self;
    }

    This operator-(const This &other) const
    {
        auto result = this->Upcast();
        return result -= other;
    }

    This & operator*=(const This &other)
    {
        auto & self = this->Upcast();

        auto multiply = [&self, &other] (auto field)
        {
            self.*(field.member) *= other.*(field.member);
        };

        jive::ForEach(
            Fields<This>::fields,
            multiply);

        return self;
    }

    This operator*(const This &other) const
    {
        auto result = this->Upcast();
        return result *= other;
    }

    This & operator/=(const This &other)
    {
        auto & self = this->Upcast();

        auto divide = [&self, &other] (auto field)
        {
            self.*(field.member) /= other.*(field.member);
        };

        jive::ForEach(
            Fields<This>::fields,
            divide);

        return self;
    }

    This operator/(const This &other) const
    {
        auto result = this->Upcast();
        return result /= other;
    }


    /***** Scalar operators *****/

    This & operator+=(T scalar)
    {
        auto & self = this->Upcast();

        auto add = [&self, scalar] (auto field)
        {
            self.*(field.member) += scalar;
        };

        jive::ForEach(
            Fields<This>::fields,
            add);

        return self;
    }

    This operator+(T scalar) const
    {
        auto result = this->Upcast();
        return result += scalar;
    }

    This & operator-=(T scalar)
    {
        auto & self = this->Upcast();

        auto subtract = [&self, scalar] (auto field)
        {
            self.*(field.member) -= scalar;
        };

        jive::ForEach(
            Fields<This>::fields,
            subtract);

        return self;
    }

    This operator-(T scalar) const
    {
        auto result = this->Upcast();
        return result -= scalar;
    }

    This & operator*=(T scalar)
    {
        auto & self = this->Upcast();

        auto multiply = [&self, scalar] (auto field)
        {
            self.*(field.member) *= scalar;
        };

        jive::ForEach(
            Fields<This>::fields,
            multiply);

        return self;
    }

    This operator*(T scalar) const
    {
        auto result = this->Upcast();
        return result *= scalar;
    }

    This & operator/=(T scalar)
    {
        auto & self = this->Upcast();

        auto divide = [&self, scalar] (auto field)
        {
            self.*(field.member) /= scalar;
        };

        jive::ForEach(
            Fields<This>::fields,
            divide);

        return self;
    }

    This operator/(T scalar) const
    {
        auto result = this->Upcast();
        return result /= scalar;
    }

    T Squared() const
    {
        T result = 0;

        const auto & self = this->Upcast();

        auto square = [&self, &result] (auto field)
        {
            auto member = self.*(field.member);
            auto memberSquared = member * member;
            assert(memberSquared + result <= std::numeric_limits<T>::max());
            result = static_cast<T>(result + memberSquared);
        };

        jive::ForEach(
            Fields<This>::fields,
            square);

        return result;
    }

    T SquaredDistance(const This &other) const
    {
        return (other - this->Upcast()).Squared();
    }

    friend bool operator==(const This &left, const This &right)
    {
        return fields::ComparisonTuple(left, Fields<This>::fields)
            == fields::ComparisonTuple(right, Fields<This>::fields);
    }

    friend bool operator!=(const This &left, const This &right)
    {
        return fields::ComparisonTuple(left, Fields<This>::fields)
            != fields::ComparisonTuple(right, Fields<This>::fields);
    }

    /*
        fields::ComparisonTuple uses std::tuple to perform a lexicographical
        compare, which returns after the first 'true' comparison.

        For arithmetic types, we only want a 'true' result if ALL elements
        compare true.
    */

    template<template<typename> typename Operator>
    static bool Compare(const This &left, const This &right)
    {
        bool result = true;

        // Check the logical AND of all comparisons.
        auto compare = [&result, &left, &right](auto field) -> void
        {
            using MemberType = typename std::remove_reference_t
                <
                    decltype(left.*(field.member))
                >;

            if (result)
            {
                result = Operator<MemberType>{}(
                    left.*(field.member),
                    right.*(field.member));
            }
        };

        jive::ForEach(Fields<This>::fields, compare);

        return result;
    }

    friend bool operator<(const This &left, const This &right)
    {
        return This::template Compare<std::less>(left, right);
    }

    friend bool operator>(const This &left, const This &right)
    {
        return This::template Compare<std::greater>(left, right);
    }

    friend bool operator<=(const This &left, const This &right)
    {
        return This::template Compare<std::less_equal>(left, right);
    }

    friend bool operator>=(const This &left, const This &right)
    {
        return This::template Compare<std::greater_equal>(left, right);
    }
};


} // end namespace tau
