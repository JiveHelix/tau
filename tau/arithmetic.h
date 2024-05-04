#pragma once

#include <type_traits>
#include <cmath>
#include <fields/fields.h>
#include <jive/for_each.h>
#include <jive/zip_apply.h>
#include <jive/overflow.h>

#include "tau/eigen.h"


namespace tau
{

struct Round {};
struct Floor {};
struct Ceil {};

template<typename T, typename V, typename Style>
std::enable_if_t<std::is_integral_v<T>, T>
DoCast(V value)
{
    // Casting to an integral type.
    assert(jive::CheckConvertible<T>(value));

    if constexpr (std::is_floating_point_v<V>)
    {
        // Source value is floating point.
        // Apply selected conversion style.
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
DoCast(V value)
{
    // Casting to a floating-point value.
    // No rounding is necessary.
    assert(jive::CheckConvertible<T>(value));

    return static_cast<T>(value);
}


// Default to Round
template<typename T, typename V>
T Cast(V value)
{
    return DoCast<T, V, Round>(value);
}


template<typename T, typename = void>
struct HasCast_: std::false_type {};

template<typename T>
struct HasCast_
<
    T,
    std::enable_if_t
    <
        !std::is_same_v
        <
            void,
            decltype(std::declval<T>().template Cast<int>())
        >
    >
>
: std::true_type {};

template<typename T>
inline constexpr bool HasCast = HasCast_<T>::value;


template<typename T, typename = void>
struct HasEigenCast_: std::false_type {};

template<typename T>
struct HasEigenCast_
<
    T,
    std::enable_if_t
    <
        !std::is_same_v
        <
            void,
            decltype(std::declval<T>().template cast<int>())
        >
    >
>
: std::true_type {};

template<typename T>
inline constexpr bool HasEigenCast = HasEigenCast_<T>::value;


template<typename T, typename = void>
struct HasStyleCast_: std::false_type {};

template<typename T>
struct HasStyleCast_
<
    T,
    std::enable_if_t
    <
        !std::is_same_v
        <
            void,
            decltype(std::declval<T>().template Cast<int, Round>())
        >
    >
>
: std::true_type {};

template<typename T>
inline constexpr bool HasStyleCast = HasStyleCast_<T>::value;


template
<
    typename Result,
    typename T,
    typename Style,
    typename Source
>
Result CastFields(const Source &source)
{
    if constexpr (std::is_same_v<Result, Source>)
    {
        return source;
    }

    Result result;

    auto convert = [&result, &source] (auto sourceField, auto resultField)
    {
        using Member = std::remove_reference_t<
            decltype(source.*(sourceField.member))>;

        if constexpr (HasStyleCast<Member>)
        {
            result.*(resultField.member) =
                (source.*(sourceField.member)).template Cast<T, Style>();
        }
        else if constexpr (HasCast<Member> && std::is_same_v<Style, Round>)
        {
            // Member has Cast function without Style selection.
            // Allow only because the caller has requested the default style.
            result.*(resultField.member) =
                (source.*(sourceField.member)).template Cast<T>();
        }
        else if constexpr (HasEigenCast<Member>)
        {
            result.*(resultField.member) =
                (source.*(sourceField.member)).template cast<T>();
        }
        else if constexpr (std::is_compound_v<Member>)
        {
            // Compound members without Cast or cast functions will be copied
            // without any conversion.
            result.*(resultField.member) = source.*(sourceField.member);
        }
        else
        {
            result.*(resultField.member) =
                DoCast<T, Member, Style>(source.*(sourceField.member));
        }
    };

    jive::ZipApply(convert, Source::fields, Result::fields);

    return result;
}


template<typename T, typename Enable = void>
struct DefinesIsBasicArithmetic_: std::false_type {};

template<typename T>
struct DefinesIsBasicArithmetic_
<
    T,
    std::enable_if_t<T::isBasicArithmetic>
>
: std::true_type {};

template<typename T>
inline constexpr bool DefinesIsBasicArithmetic =
    DefinesIsBasicArithmetic_<T>::value;


template<typename T, typename Enable = void>
struct IsBasicArithmetic_: std::false_type {};

template<typename T>
struct IsBasicArithmetic_
<
    T,
    std::enable_if_t<DefinesIsBasicArithmetic<T>>
>: std::true_type {};

template<typename T>
inline constexpr bool IsBasicArithmetic = IsBasicArithmetic_<T>::value;


namespace op
{

struct Add;
struct Subtract;
struct Multiply;
struct Divide;

} // end namespace op


template<typename Target, typename Source>
void AssignCast(Target &target, const Source &source)
{
    if constexpr (std::is_scalar_v<Target>)
    {
        // Cast source to Target type,
        // and, in debug mode, check type bounds.
        target = Cast<Target, Source>(source);
    }
    else
    {
        target = source;
    }
}



template<typename Operator, typename Target, typename Operand>
void OpAssign(Target &target, const Operand &source)
{
    if constexpr (std::is_same_v<Operator, op::Add>)
    {
        AssignCast(target, target + source);
    }
    else if constexpr (std::is_same_v<Operator, op::Subtract>)
    {
        AssignCast(target, target - source);
    }
    else if constexpr (std::is_same_v<Operator, op::Multiply>)
    {
        AssignCast(target, target * source);
    }
    else if constexpr (std::is_same_v<Operator, op::Divide>)
    {
        AssignCast(target, target / source);
    }
}


template
<
    typename T,
    typename Derived
>
struct BasicArithmetic
{
    using This = Derived;

    static constexpr bool isBasicArithmetic = true;

    This & Upcast()
    {
        return *static_cast<This *>(this);
    }

    const This & Upcast() const
    {
        return *static_cast<const This *>(this);
    }

    /***** Element-wise operators *****/
    // TODO: Add assertions (or exceptions?) when operators cause overflow.
    This & operator+=(const This &other)
    {
        auto & self = this->Upcast();

        auto add = [&self, &other] (auto field)
        {
            OpAssign<op::Add>(self.*(field.member), other.*(field.member));
        };

        jive::ForEach(This::fields, add);

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
            OpAssign<op::Subtract>(self.*(field.member), other.*(field.member));
        };

        jive::ForEach(This::fields, subtract);

        return self;
    }

    This operator-(const This &other) const
    {
        auto result = this->Upcast();
        return result.operator-=(other);
    }

    This & operator*=(const This &other)
    {
        auto & self = this->Upcast();

        auto multiply = [&self, &other] (auto field)
        {
            OpAssign<op::Multiply>(self.*(field.member), other.*(field.member));
        };

        jive::ForEach(This::fields, multiply);

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
            OpAssign<op::Divide>(self.*(field.member), other.*(field.member));
        };

        jive::ForEach(This::fields, divide);

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
            OpAssign<op::Add>(self.*(field.member), scalar);
        };

        jive::ForEach(This::fields, add);

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
            OpAssign<op::Subtract>(self.*(field.member), scalar);
        };

        jive::ForEach(This::fields, subtract);

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
            OpAssign<op::Multiply>(self.*(field.member), scalar);
        };

        jive::ForEach(This::fields, multiply);

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
            // /= would be convenient, but division converts to int,
            // which triggers a conversion warning.
            OpAssign<op::Divide>(self.*(field.member), scalar);
        };

        jive::ForEach(This::fields, divide);

        return self;
    }

    This operator/(T scalar) const
    {
        auto result = this->Upcast();
        return result /= scalar;
    }

    T SquaredSum() const
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

        jive::ForEach(This::fields, square);

        return result;
    }

    This Squared() const
    {
        auto self = this->Upcast();

        auto square = [&self](auto field) -> void
        {
            self.*(field.member) *= self.*(field.member);
        };

        jive::ForEach(This::fields, square);

        return self;
    }

    T Magnitude() const
    {
        return static_cast<T>(std::sqrt(this->SquaredSum()));
    }

    friend bool operator==(const This &left, const This &right)
    {
        return fields::ComparisonTuple(left)
            == fields::ComparisonTuple(right);
    }

    friend bool operator!=(const This &left, const This &right)
    {
        return fields::ComparisonTuple(left)
            != fields::ComparisonTuple(right);
    }

    /*
        fields::ComparisonTuple uses std::tuple to perform a lexicographical
        compare, which returns after the first 'true' comparison.
    */

    friend bool operator<(const This &left, const This &right)
    {
        return fields::ComparisonTuple(left)
            < fields::ComparisonTuple(right);
    }

    friend bool operator>(const This &left, const This &right)
    {
        return fields::ComparisonTuple(left)
            > fields::ComparisonTuple(right);
    }

    friend bool operator<=(const This &left, const This &right)
    {
        return fields::ComparisonTuple(left)
            <= fields::ComparisonTuple(right);
    }

    friend bool operator>=(const This &left, const This &right)
    {
        return fields::ComparisonTuple(left)
            >= fields::ComparisonTuple(right);
    }


    /*
        fields::ComparisonTuple uses std::tuple to perform a lexicographical
        compare, which returns after the first 'true' comparison.

        Sometimes it is necessary to check whether all members compare 'true';
        for example, when checking array bounds.
    */

    // Check the logical AND of all comparisons.
    template<template<typename> typename Operator>
    bool AndCompare(const This &other) const
    {
       bool result = true;

       const This &self = this->Upcast();

       auto compare = [&result, &self, &other](auto field) -> void
       {
           using MemberType = typename std::remove_reference_t
               <
                   decltype(self.*(field.member))
               >;

           if (result)
           {
               result = Operator<MemberType>{}(
                   self.*(field.member),
                   other.*(field.member));
           }
       };

       jive::ForEach(This::fields, compare);

       return result;
    }

    bool AndLess(const This &other) const
    {
        return this->template AndCompare<std::less>(other);
    }

    bool AndGreater(const This &other) const
    {
        return this->template AndCompare<std::greater>(other);
    }

    bool AndLessEqual(const This &other) const
    {
        return this->template AndCompare<std::less_equal>(other);
    }

    bool AndGreaterEqual(const This &other) const
    {
        return this->template AndCompare<std::greater_equal>(other);
    }
};





template
<
    typename T,
    template<typename> typename Derived
>
struct Arithmetic: public BasicArithmetic<T, Derived<T>>
{
    using Base = BasicArithmetic<T, Derived<T>>;

    template<typename U, typename Style = Round>
    auto Cast() const
    {
        return CastFields<Derived<U>, U, Style>(this->Upcast());
    }

    typename Base::This Normalize() const
    {
        return *this / this->Magnitude();
    }
};


template
<
    typename T,
    typename Derived
>
Derived operator*(T scalar, const BasicArithmetic<T, Derived> &arithmetic)
{
    return (arithmetic * scalar).Upcast();
}


template<typename Container>
std::optional<typename Container::iterator>
GetUniqueInsertion(
    Container &container,
    const typename Container::value_type &value)
{
    auto found = std::lower_bound(
        begin(container),
        end(container),
        value);

    if (found == end(container))
    {
        return found;
    }

    if (*found != value)
    {
        if (*found == value)
        {
            throw std::logic_error("Can't be true AND not true!");
        }

        // This value is not already in the list.
        return found;
    }

    return {};
}


} // end namespace tau
