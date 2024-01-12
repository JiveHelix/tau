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
        tau::SizeFields,
        tau::SizeTemplate<int8_t>::template Template,
        pex::PlainT<tau::Size<int8_t>>
    >;

template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<int16_t>::template Template,
        pex::PlainT<tau::Size<int16_t>>
    >;

template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<int32_t>::template Template,
        pex::PlainT<tau::Size<int32_t>>
    >;

template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<int64_t>::template Template,
        pex::PlainT<tau::Size<int64_t>>
    >;

template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<uint8_t>::template Template,
        pex::PlainT<tau::Size<uint8_t>>
    >;

template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<uint16_t>::template Template,
        pex::PlainT<tau::Size<uint16_t>>
    >;

template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<uint32_t>::template Template,
        pex::PlainT<tau::Size<uint32_t>>
    >;

template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<uint64_t>::template Template,
        pex::PlainT<tau::Size<uint64_t>>
    >;

template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<float>::template Template,
        pex::PlainT<tau::Size<float>>
    >;

template struct Group
    <
        tau::SizeFields,
        tau::SizeTemplate<double>::template Template,
        pex::PlainT<tau::Size<double>>
    >;


} // end namespace pex
