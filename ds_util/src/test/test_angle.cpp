#include "ds_util/ds_util.h"

#include <cmath>
#include <gtest/gtest.h>

// This test case should capture valid strings that we don't have parsers for
TEST(AngularSeperation, expectedValues)
{


  using testData = struct
  {
    double from_deg;
    double to_deg;
    double expected_deg;
  };

  // 0 separation
  auto from = 0;
  auto to = 0;
  auto expected = 0;
  auto result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // 0 separation
  from = 180;
  to = 180;
  expected = 0;
  result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  from = 0;
  to = 15;
  expected = 15;
  result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  from = 15;
  to = 30;
  expected = 15;
  result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Backwards should be negative
  from = 30;
  to = 15;
  expected = -15;
  result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Backwards around 0
  from = 10;
  to = -10;
  expected = -20;
  result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Forwards around 0
  from = -10;
  to = 10;
  expected = 20;
  result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Forwards around 0 using heading values
  from = 350;
  to = 10;
  expected = 20;
  result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Backwards around 0 using heading values
  from = 10;
  to = 350;
  expected = -20;
  result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Angle separations are in the interval (-pi, pi]
  from = 0;
  to = 270;
  expected = -90;
  result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Angle separations are in the interval (-pi, pi]
  from = 0;
  to = -270;
  expected = 90;
  result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Angle separations are in the interval (-pi, pi]
  from = 0;
  to = 180;
  expected = 180;
  result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  from = 180;
  to = 0;
  expected = -180;
  result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  from = 185;
  to = 180;
  expected = -5;
  result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  from = 175;
  to = 180;
  expected = 5;
  result = ds_util::angular_separation_radians(from * M_PI/180, to * M_PI/180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
