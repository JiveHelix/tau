#pragma once


#include <string>
#include <fstream>
#include <unordered_map>
#include <jive/strings.h>
#include <jive/to_integer.h>
#include <jive/to_float.h>
#include <jive/precise_string.h>
#include <fmt/core.h>
#include "tau/size.h"
#include "tau/eigen_shim.h"


namespace tau
{


template<typename TokenT>
std::vector<std::string> ExtractLine(
    const std::string& input,
    const TokenT &token)
{
    size_t tokenSize;

    if constexpr (std::is_same_v<TokenT, char>)
    {
        tokenSize = 1;
    }
    else
    {
        tokenSize = token.size();
    }

    std::string::size_type position = 0;

    if (tokenSize == 0)
    {
        throw std::invalid_argument("token must have non-zero length.");
    }

    std::vector<std::string> result;
    std::string::size_type inputEnd = input.size();

    if (inputEnd == 0)
    {
        return {};
    }

    bool quoted = false;
    std::string cell;

    while (position < inputEnd)
    {
        if (quoted)
        {
            if (input[position] == '"')
            {
                if (
                    position + 1 != inputEnd
                    && input[position + 1] == '"')
                {
                    // This is a repeated quote.
                    // Place a literal quote in the cell.
                    cell.push_back('"');
                    ++position;
                }
                else
                {
                    // We found the end of the quoted cell.
                    quoted = false;
                }
            }
            else
            {
                cell.push_back(input[position]);
            }
        }
        else
        {
            if (input[position] == '"')
            {
                // begin quoted section.
                assert(cell.empty());
                quoted = true;
            }
            else if (input[position] == ',')
            {
                result.push_back(jive::strings::Trim(cell));
                cell.clear();
            }
            else
            {
                cell.push_back(input[position]);
            }
        }

        ++position;
    }

    result.push_back(jive::strings::Trim(cell));

    return result;
}


std::string FilterCell(std::string cell);


class Csv
{
public:
    using Index = Eigen::Index;
    using Size = tau::Size<Index>;
    using Cells = std::vector<std::vector<std::string>>;

    Csv();

    Csv(std::istream &&inputStream, bool hasHeaders);

    Csv(const std::string &fileName, bool hasHeaders);

    const std::vector<std::string> & GetHeaders() const;

    const std::unordered_map<std::string, Index> & GetHeaderMap() const;

    Size GetSize() const;

    std::string operator()(Index row, Index column) const;

    std::string operator()(const std::string &headerName, Index row) const;

    const Cells & GetCells() const;

    template<typename T>
    T GetNumber(Index row, Index column) const
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            return jive::ToFloat<T>(this->operator()(row, column));
        }
        else
        {
            return jive::ToInteger<T>(this->operator()(row, column));
        }
    }

    template<typename T>
    T GetNumber(const std::string &headerName, Index row) const
    {
        return this->GetNumber<T>(row, this->headerMap_.at(headerName));
    }

    void ToFile(const std::string &fileName);

    std::ostream & ToStream(std::ostream & outputStream);

    Index GetRowCount() const;

    Index GetColumnCount() const;

private:
    std::vector<std::string> headers_;
    std::unordered_map<std::string, Index> headerMap_;
    std::vector<std::vector<std::string>> cells_;
    Index rowCount_;
    Index columnCount_;
};


template<typename T, int options = Eigen::ColMajor>
class EigenCsv
{
public:
    using Index = Eigen::Index;
    using Size = tau::Size<Index>;
    using Data = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, options>;

    EigenCsv()
        :
        headers_{},
        headerMap_{},
        data_{}
    {

    }

    EigenCsv(
        const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, options> &data,
        const std::vector<std::string> &headers = {})
        :
        headers_(headers),
        headerMap_(),
        data_(data)
    {
        // Store in unordered_map to decrease search time when looking up
        // a column by header name.
        Index columnIndex = 0;

        for (auto &header: this->headers_)
        {
            // Clean up whitespace
            header = jive::strings::Trim(header);
            this->headerMap_[header] = columnIndex++;
        }
    }

    EigenCsv(std::istream &&inputStream, bool hasHeaders)
    {
        Csv csv(std::move(inputStream), hasHeaders);
        this->headers_ = csv.GetHeaders();
        this->headerMap_ = csv.GetHeaderMap();

        Index rowCount = csv.GetRowCount();
        Index columnCount = csv.GetColumnCount();

        this->data_ = Data::Zero(rowCount, columnCount);

        for (Index row = 0; row < rowCount; ++row)
        {
            for (Index column = 0; column < columnCount; ++column)
            {
                try
                {
                    this->data_(row, column) =
                        jive::ToFloat<T>(csv(row, column));
                }
                catch (std::invalid_argument &)
                {
                    std::cerr << "Failed to convert value in row " << row << ", column " << column << ": " << csv(row, column) << std::endl;

                    throw;
                }
            }
        }
    }

    EigenCsv(const std::string &fileName, bool hasHeaders)
        :
        EigenCsv(std::ifstream(fileName), hasHeaders)
    {

    }

    const std::vector<std::string> & GetHeaders() const
    {
        return this->headers_;
    }

    Size GetSize() const
    {
        return {this->data_.cols(), this->data_.rows()};
    }

    T operator()(Index row, Index column) const
    {
        return this->data_(row, column);
    }

    T operator()(const std::string &headerName, Index row) const
    {
        return this->data_(row, this->headerMap_.at(headerName));
    }

    Data GetData() const
    {
        return this->data_;
    }

    void ToFile(const std::string &fileName)
    {
        std::ofstream outputStream(fileName);

        if (!outputStream)
        {
            throw std::runtime_error(
                "Cannot open " + fileName + " for writing");
        }

        this->ToStream(outputStream);
    }

    std::ostream & ToStream(std::ostream & outputStream)
    {
        if (!this->headers_.empty())
        {
            outputStream <<
                jive::strings::Join(
                    std::begin(this->headers_),
                    std::end(this->headers_),
                    ',') << std::endl;
        }

        Index rowCount = this->data_.rows();
        Index columnCount = this->data_.cols();

        for (Index row = 0; row < rowCount; ++row)
        {
            for (Index column = 0; column < columnCount - 1; ++column)
            {
                outputStream
                    << jive::PreciseString(this->data_(row, column))
                    << ",";
            }

            // Append the last value without a comma.
            outputStream << jive::PreciseString(
                this->data_(row, columnCount - 1));

            outputStream << '\n';
        }

        return outputStream;
    }

private:
    std::vector<std::string> headers_;
    std::unordered_map<std::string, Index> headerMap_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, options> data_;
};


} // end namespace tau
