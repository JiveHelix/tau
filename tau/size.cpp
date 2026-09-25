#include "tau/size.h"


namespace tau
{


template struct Size<int8_t>;
template struct Size<int16_t>;
template struct Size<int32_t>;
template struct Size<int64_t>;

template struct Size<uint8_t>;
template struct Size<uint16_t>;
template struct Size<uint32_t>;
template struct Size<uint64_t>;

template struct Size<float>;
template struct Size<double>;


} // end namespace tau


namespace pex
{


template struct Group
    <
        tau::SizeSchema<int8_t>::template Schema,
        tau::SizeFinisher<int8_t>
    >;

template struct Group
    <
        tau::SizeSchema<int16_t>::template Schema,
        tau::SizeFinisher<int16_t>
    >;

template struct Group
    <
        tau::SizeSchema<int32_t>::template Schema,
        tau::SizeFinisher<int32_t>
    >;

template struct Group
    <
        tau::SizeSchema<int64_t>::template Schema,
        tau::SizeFinisher<int64_t>
    >;

template struct Group
    <
        tau::SizeSchema<uint8_t>::template Schema,
        tau::SizeFinisher<uint8_t>
    >;

template struct Group
    <
        tau::SizeSchema<uint16_t>::template Schema,
        tau::SizeFinisher<uint16_t>
    >;

template struct Group
    <
        tau::SizeSchema<uint32_t>::template Schema,
        tau::SizeFinisher<uint32_t>
    >;

template struct Group
    <
        tau::SizeSchema<uint64_t>::template Schema,
        tau::SizeFinisher<uint64_t>
    >;

template struct Group
    <
        tau::SizeSchema<float>::template Schema,
        tau::SizeFinisher<float>
    >;

template struct Group
    <
        tau::SizeSchema<double>::template Schema,
        tau::SizeFinisher<double>
    >;


} // end namespace pex
