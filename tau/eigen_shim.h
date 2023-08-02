#pragma once


#if defined _WIN32 && !(defined __MINGW32__ || defined __MINGW64__)

#define EIGEN_SHIM_PUSH_IGNORES \
    __pragma(warning(push)) \
    __pragma(warning(disable:4127)) \
    __pragma(warning(disable:4242)) \
    __pragma(warning(disable:5054))

#define EIGEN_SHIM_POP_IGNORES \
    __pragma(warning(pop))

#else

#define EIGEN_SHIM_PUSH_IGNORES
#define EIGEN_SHIM_POP_IGNORES

#endif

EIGEN_SHIM_PUSH_IGNORES

#include <Eigen/Dense>

EIGEN_SHIM_POP_IGNORES
