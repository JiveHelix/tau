#pragma once

#include <iostream>
#include "tau/eigen.h"


namespace tau
{

#if 0
Notes on padding with reflected values or zeros, without making an extra copy
of the input signal.

s is the signal to be convolved with k, the reversed kernel.
Reflection:

k0 k1 k2 k3
s3 s2 s1 s0 s1 s2 s3 s4 s5 s6
i0 i0 i0 i0
j0 j1 j2 j3
3   2  1  0

   k0 k1 k2 k3
s3 s2 s1 s0 s1 s2 s3 s4 s5 s6
   i1 i1 i1 i1
   j0 j1 j2 j3
    2  1  0  1

      k0 k1 k2 k3
s3 s2 s1 s0 s1 s2 s3 s4 s5 s6
      i2 i2 i2 i2
      j0 j1 j2 j3
       1  0  1  2


         k0 k1 k2 k3
s3 s2 s1 s0 s1 s2 s3 s4 s5 s6


                  k0 k1 k2 k3
         s0 s1 s2 s3 s4 s5 s6

                     k0 k1 k2 k3
         s0 s1 s2 s3 s4 s5 s6 s5
                     i0 i0 i0 i0
                     j0 j1 j2 j3
                     -2 -1  0 -1

                        k0 k1 k2 k3
         s0 s1 s2 s3 s4 s5 s6 s5 s4
                        i1 i1 i1 i1
                        j0 j1 j2 j3
                        -1  0 -1 -2

                           k0 k1 k2 k3
         s0 s1 s2 s3 s4 s5 s6 s5 s4 s3
                           i2 i2 i2 i2
                           j0 j1 j2 j3
                            0 -1 -2 -3

         lastIndex - abs(k_n - i - 2 - j)


Zero padded:

k0 k1 k2 k3
 0  0  0 s0 s1 s2 s3 s4 s5 s6
i0 i0 i0 i0
j0 j1 j2 j3
s[:1] * k[3:]

   k0 k1 k2 k3
 0  0  0 s0 s1 s2 s3 s4 s5 s6
   i1 i1 i1 i1
   j0 j1 j2 j3
s[:2] * k[2:]

      k0 k1 k2 k3
 0  0  0 s0 s1 s2 s3 s4 s5 s6
      i2 i2 i2 i2
      j0 j1 j2 j3
s[:3] * k[1:]


Valid:
         k0 k1 k2 k3
 0  0  0 s0 s1 s2 s3 s4 s5 s6


Padded Tail:
                  k0 k1 k2 k3
         s0 s1 s2 s3 s4 s5 s6

                     k0 k1 k2 k3
         s0 s1 s2 s3 s4 s5 s6  0
                     i0 i0 i0 i0
                     j0 j1 j2 j3
s[4:] * k[:3]
s[-3:] * k[:3]

                        k0 k1 k2 k3
         s0 s1 s2 s3 s4 s5 s6  0  0
                        i1 i1 i1 i1
                        j0 j1 j2 j3
s[5:] * k[:2]
s[-2:] * k[:2]

                           k0 k1 k2 k3
         s0 s1 s2 s3 s4 s5 s6  0  0  0
                           i2 i2 i2 i2
                           j0 j1 j2 j3
s[6:] * k[:1]
s[-1:] * k[:1]
#endif


// Expects kernel to be already reversed.
template<typename T, int InputSize, int KernelSize>
auto DoRowConvolve(
    const Eigen::RowVector<T, InputSize> &input,
    const Eigen::RowVector<T, KernelSize> &kernel,
    [[maybe_unused]] bool reflect = false)
{
    using Eigen::Index;
    using Eigen::Dynamic;
    using Eigen::seqN;

    constexpr bool isDynamic = InputSize == Dynamic || KernelSize == Dynamic;

    using Result = Eigen::RowVector
        <
            T,
            isDynamic
                ? Dynamic
                : InputSize + KernelSize - 1
        >;

    Index kernelSize = kernel.size();
    Index resultSize = input.size() + kernelSize - 1;
    Result result;

    if constexpr (isDynamic)
    {
        result = Result::Zero(resultSize);
    }
    else
    {
        // Compile-time size makes run-time size argument redundant.
        result = Result::Zero();
    }

    Index validStart = kernelSize - 1;
    Index validEnd = resultSize - validStart;
    Index validLength = validEnd - validStart;

    for (Index i = 0; i < validLength; ++i)
    {
        result(validStart + i) =
            (kernel.cwiseProduct(input(seqN(i, kernelSize)))).sum();
    }

    if (reflect)
    {
        // Compute the beginning/end using symmetric (mirrored) values from
        // the input
        Index lastInput = input.size() - 1;

        for (Index i = 0; i < validStart; ++i)
        {
            for (Index j = 0; j < kernelSize; ++j)
            {
                result(i) +=
                    kernel(j) * input(std::abs(kernelSize - j - i - 1));

                Index offsetFromEnd = std::abs(kernelSize - j - i - 2);

                result(validEnd + i) +=
                    kernel(j) * input(lastInput - offsetFromEnd);
            }
        }

        return result;
    }

    assert(!reflect);

    // Compute the beginning/end using zeros
    for (Index i = 0; i < validStart; ++i)
    {
        result(i) =
            kernel.tail(i + 1).cwiseProduct(input(seqN(0, i + 1))).sum();

        Index offset = kernelSize - 1 - i;

        result(validEnd + i) =
            kernel(seqN(0, offset)).cwiseProduct(input.tail(offset)).sum();
    }

    return result;
}


template<typename T, int InputSize, int KernelSize>
auto RowConvolve(
    const Eigen::RowVector<T, InputSize> &input,
    const Eigen::RowVector<T, KernelSize> &kernel,
    bool reflect = false)
{
    return DoRowConvolve(input, kernel.reverse().eval(), reflect);
}


} // end namespace tau
