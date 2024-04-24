#include "tau/csv.h"


namespace tau
{


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


Csv::Csv()
    :
    headers_{},
    headerMap_{},
    cells_{},
    rowCount_(0),
    columnCount_(0)
{

}


Csv::Csv(std::istream &&inputStream, bool hasHeaders)
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


Csv::Csv(const std::string &fileName, bool hasHeaders)
    :
    Csv(std::ifstream(fileName), hasHeaders)
{

}


const std::vector<std::string> & Csv::GetHeaders() const
{
    return this->headers_;
}


const std::unordered_map<std::string, Eigen::Index> & Csv::GetHeaderMap() const
{
    return this->headerMap_;
}


Csv::Size Csv::GetSize() const
{
    return {this->columnCount_, this->rowCount_};
}


std::string Csv::operator()(Eigen::Index row, Eigen::Index column) const
{
    return this->cells_.at(SizeCheck(row)).at(SizeCheck(column));
}


std::string Csv::operator()(
    const std::string &headerName,
    Eigen::Index row) const
{
    return this->cells_.at(SizeCheck(row))
        .at(SizeCheck(this->headerMap_.at(headerName)));
}


const Csv::Cells & Csv::GetCells() const
{
    return this->cells_;
}


void Csv::ToFile(const std::string &fileName)
{
    std::ofstream outputStream(fileName);

    if (!outputStream)
    {
        throw std::runtime_error(
            "Cannot open " + fileName + " for writing");
    }

    this->ToStream(outputStream);
}


std::ostream & Csv::ToStream(std::ostream & outputStream)
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


Eigen::Index Csv::GetRowCount() const
{
    return this->rowCount_;
}


Eigen::Index Csv::GetColumnCount() const
{
    return this->columnCount_;
}


} // end namespace tau
