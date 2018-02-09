//
// Created by zac on 12/9/17.
//

#include "ds_util/ds_util.h"

#include <list>
#include <gtest/gtest.h>

// This test case should capture valid strings that we don't have parsers for
TEST(FofonoffTest, expectedValues)
{
  using testData = struct
  {
    double pressure_dbar;
    double latitude_deg;
    double depth;
  };

  // Test values sampled from the original text:
  // http://unesdoc.unesco.org/images/0005/000598/059832eb.pdf
  const auto test_data = std::list<testData>{ { 500, 30, 496.00 }, { 7000, 90, 6814.84 }, { 10000, 30, 9712.653 } };

  for (const auto data : test_data)
  {
    // Construct a ByteSequence message
    // Attempt to parse
    auto result = ds_util::fofonoff_depth(data.pressure_dbar, data.latitude_deg);

    // Need to use expect_near here because even for the original sources expect_float_equal fails.
    EXPECT_NEAR(data.depth, result, 0.1);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
