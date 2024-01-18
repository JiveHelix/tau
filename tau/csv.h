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

std::string FilterCell(std::string cell)
{
    auto comma = cell.find(',');

    bool requiresQuotes = false;

    if (comma != std::string::npos)
    {
        // Found a comma in the cell
        requiresQuotes = true;
    }

    auto literalQuote = cell.find('"');

    while (literalQuote != std::string::npos)
    {
        requiresQuotes = true;
        cell.replace(literalQuote, 1, "\"\"");
        literalQuote = cell.find('"', literalQuote + 2);
    }

    if (requiresQuotes)
    {
        return fmt::format("\"{}\"", cell);
    }

    return cell;
}


class Csv
{
public:
    using Index = Eigen::Index;
    using Size = tau::Size<Index>;
    using Cells = std::vector<std::vector<std::string>>;

    Csv()
        :
        headers_{},
        headerMap_{},
        cells_{},
        rowCount_(0),
        columnCount_(0)
    {

    }

    Csv(std::istream &&inputStream, bool hasHeaders)
    {
        if (!inputStream)
        {
            throw std::runtime_error("Bad input stream");
        }

        std::string line;

        uint8_t bom[4]{};

        inputStream.read(reinterpret_cast<char *>(&bom[0]), 4);

        if (!inputStream)
        {
            throw std::runtime_error("Unable to read stream");
        }

        if (
            (bom[0] == 0xFE)
            || (bom[0] == 0xFF)
            || (bom[0] == 0))
        {
            throw std::runtime_error(
                "Wide encodings not supported. Recreate the csv as utf8.");
        }

        if ((bom[0] == 0xEF) && (bom[1] == 0xBB) && (bom[2] == 0xBF))
        {
            // UTF8 encoding.
            inputStream.seekg(3);
        }
        else
        {
            // Treat the file as if there is no BOM.
            inputStream.seekg(0);
        }

        if (hasHeaders)
        {
            std::getline(inputStream, line, '\n');

            if (!inputStream)
            {
                throw std::runtime_error("File appears to be empty");
            }

            this->headers_ = jive::strings::Split(line, ',');

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

        size_t maximumColumnCount = 0;

        while (true)
        {
            std::getline(inputStream, line, '\n');

            if (!inputStream)
            {
                break;
            }

            auto extracted = ExtractLine(line, ',');

            bool lineIsEmpty = std::all_of(
                std::begin(extracted),
                std::end(extracted),
                [](const auto &entry) -> bool
                {
                    return entry.empty();
                });

            if (!lineIsEmpty)
            {
                bool someAreEmpty = std::any_of(
                    std::begin(extracted),
                    std::end(extracted),
                    [](const auto &entry) -> bool
                    {
                        return entry.empty();
                    });


                if (someAreEmpty)
                {
                    std::cerr << "Warning, empty cells in line: " << line << std::endl;
                }

                this->cells_.push_back(extracted);

                maximumColumnCount =
                    std::max(maximumColumnCount, extracted.size());
            }
        }

        this->rowCount_ = static_cast<Index>(this->cells_.size());
        this->columnCount_ = static_cast<Index>(maximumColumnCount);

        if (maximumColumnCount == 0)
        {
            throw std::runtime_error("File has no data");
        }

        // CSV files are required to have the same number of columns in every
        // row.
        auto checkCount = static_cast<size_t>(this->rowCount_);

        while (checkCount--)
        {
            if (
                static_cast<Index>(this->cells_[checkCount].size())
                    < this->columnCount_)
            {
                throw std::runtime_error(
                    "CSV is missing columns in row "
                    + std::to_string(checkCount));
            }
        }

    }

    Csv(const std::string &fileName, bool hasHeaders)
        :
        Csv(std::ifstream(fileName), hasHeaders)
    {

    }

    const std::vector<std::string> & GetHeaders() const
    {
        return this->headers_;
    }

    const std::unordered_map<std::string, Index> & GetHeaderMap() const
    {
        return this->headerMap_;
    }

    Size GetSize() const
    {
        return {this->columnCount_, this->rowCount_};
    }

    std::string operator()(Index row, Index column) const
    {
        return this->cells_.at(SizeCheck(row)).at(SizeCheck(column));
    }

    std::string operator()(const std::string &headerName, Index row) const
    {
        return this->cells_.at(SizeCheck(row))
            .at(SizeCheck(this->headerMap_.at(headerName)));
    }

    const Cells & GetCells() const
    {
        return this->cells_;
    }

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

        for (Index row = 0; row < this->rowCount_; ++row)
        {
            for (Index column = 0; column < this->columnCount_ - 1; ++column)
            {
                outputStream << FilterCell(this->operator()(row, column))
                    << ",";
            }

            // Append the last value without a comma.
            outputStream << FilterCell(
                this->operator()(row, this->columnCount_ - 1));

            outputStream << '\n';
        }

        return outputStream;
    }

    Index GetRowCount() const
    {
        return this->rowCount_;
    }

    Index GetColumnCount() const
    {
        return this->columnCount_;
    }

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
