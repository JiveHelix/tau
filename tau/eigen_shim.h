#pragma once


#ifdef _WIN32

#define EIGEN_SHIM_PUSH_IGNORES \
    __pragma(warning(push)) \
    __pragma(warning(disable:4127))

#define EIGEN_SHIM_POP_IGNORES \
    __pragma(warning(pop))

#else

#define EIGEN_SHIM_PUSH_IGNORES \
#define EIGEN_SHIM_POP_IGNORES \

#endif

EIGEN_SHIM_PUSH_IGNORES

#include <Eigen/Dense>

EIGEN_SHIM_POP_IGNORES
