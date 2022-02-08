/**
  * @file to_csv.h
  * 
  * @brief Write Eigen::Matrix to csv file.
  * 
  * @author Jive Helix (jivehelix@gmail.com)
  * @date 07 Feb 2022
  * @copyright Jive Helix
  * Licensed under the MIT license. See LICENSE file.
**/

#pragma once


#include <string>
#include <ostream>
#include <fstream>
#include "Eigen/Dense"
#include "jive/create_exception.h"


namespace tau
{


CREATE_EXCEPTION(CsvError, std::runtime_error);


template<typename Derived>
void ToCsv(
    std::ostream &outputStream,
    const Eigen::MatrixBase<Derived> &matrix)
{
    for (Eigen::Index i = 0; i < matrix.rows(); ++i)
    {
        for (Eigen::Index j = 0; j < matrix.cols() - 1; ++j)
        {
            outputStream << matrix(i, j) << ',';
        }

        // Output the last value without the trailing comma.
        outputStream << matrix(i, matrix.cols() - 1);

        outputStream << '\n';
    }
}


template<typename Derived>
void ToCsv(
    const std::string &fileName,
    const Eigen::MatrixBase<Derived> &matrix)
{
    std::ofstream outputStream(fileName); 
    
    if (!outputStream)
    {
        throw CsvError("Unable to open file for writing: " + fileName);
    }

    ToCsv(outputStream, matrix);
}


} // end namespace tau
