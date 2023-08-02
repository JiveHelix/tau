#include "tau/vector2d.h"


namespace tau
{


template struct Point2d<int8_t>;
template struct Point2d<int16_t>;
template struct Point2d<int32_t>;
template struct Point2d<int64_t>;

template struct Point2d<uint8_t>;
template struct Point2d<uint16_t>;
template struct Point2d<uint32_t>;
template struct Point2d<uint64_t>;

template struct Point2d<float>;
template struct Point2d<double>;

template struct Vector2d<float>;
template struct Vector2d<double>;


} // end namespace tau
