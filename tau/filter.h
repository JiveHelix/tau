/**
  * @file filter.h
  * 
  * @brief Return an Array containing only values selected by the mask.
  * 
  * @author Jive Helix (jivehelix@gmail.com)
  * @date 24 May 2022
  * @copyright Jive Helix
  * Licensed under the MIT license. See LICENSE file.
**/

#pragma once


#include <vector>
#include "tau/eigen_shim.h"


namespace tau
{


template<typename Input, typename Mask>
auto Filter(const Input &input, const Mask &mask)
{
    std::vector<Eigen::Index> maskIndices;

    for (Eigen::Index i = 0; i < mask.size(); ++i)
    {
        if (mask(i))
        {
            maskIndices.push_back(i);
        }
    }

    return input.array()(maskIndices);
}


} // end namespace tau
