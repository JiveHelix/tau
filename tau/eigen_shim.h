#pragma once

#include <jive/compiler.h>

#if defined(MSC_COMPILER)

#define EIGEN_SHIM_PUSH_IGNORES \
    __pragma(warning(push)) \
    __pragma(warning(disable:4127)) \
    __pragma(warning(disable:4242)) \
    __pragma(warning(disable:5054)) \
    __pragma(warning(disable:26495))

#define EIGEN_SHIM_POP_IGNORES \
    __pragma(warning(pop))

#elif defined(GCC_COMPILER)

#define EIGEN_SHIM_PUSH_IGNORES \
    _Pragma("GCC diagnostic push") \
    _Pragma("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")
    _Pragma("GCC diagnostic ignored \"-Wnull-dereference\"")

#define EIGEN_SHIM_POP_IGNORES \
    _Pragma("GCC diagnostic pop")

#else

#define EIGEN_SHIM_PUSH_IGNORES
#define EIGEN_SHIM_POP_IGNORES

#endif

EIGEN_SHIM_PUSH_IGNORES

#include <Eigen/Dense>

EIGEN_SHIM_POP_IGNORES
