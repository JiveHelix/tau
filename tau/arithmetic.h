#pragma once

#include <type_traits>
#include <cmath>
#include <fields/fields.h>
#include <jive/for_each.h>
#include <jive/zip_apply.h>


namespace tau
{

struct Round {};
struct Floor {};
struct Ceil {};

template<typename T, typename V, typename Style>
std::enable_if_t<std::is_integral_v<T>, T>
DoConvert(V value)
{
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
    return static_cast<T>(value);
}


template<typename T, typename V>
std::enable_if_t<std::is_integral_v<T>, T>
Convert(V value)
{
    return DoConvert<T, V, Round>(value);
}


template<typename T, typename V>
std::enable_if_t<std::is_floating_point_v<T>, T>
Convert(V value)
{
    return static_cast<T>(value);
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
            result += member * member;
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
};


} // end namespace tau
