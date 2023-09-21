#include <catch2/catch.hpp>

#include <tau/csv.h>
#include <sstream>

static const std::string csvFileName = "test_data/test.csv";

TEST_CASE("Import CSV", "[csv]")
{
    auto csv = tau::Csv(csvFileName, true);

    REQUIRE(csv("Note", 3) == "This, note has \"quotes, and\" commas");
    REQUIRE(csv.GetNumber<long>(2, 0) == 44);
    REQUIRE(csv.GetNumber<double>(3, 2) == 6.7);
}


TEST_CASE("Round-trip CSV", "[csv]")
{
    auto csv = tau::Csv(csvFileName, true);
    std::stringstream testStream;
    csv.ToStream(testStream);
    auto recovered = tau::Csv(std::move(testStream), true);

    REQUIRE(recovered("Note", 3) == "This, note has \"quotes, and\" commas");
    REQUIRE(recovered.GetNumber<long>(2, 0) == 44);
    REQUIRE(recovered.GetNumber<double>(3, 2) == 6.7);
}
