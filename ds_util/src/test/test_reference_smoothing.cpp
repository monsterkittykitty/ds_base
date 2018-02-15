#include "ds_util/ds_util.h"

#include <gtest/gtest.h>

// This test case should capture valid strings that we don't have parsers for
TEST(ReferenceSmoothing, sign)
{
  EXPECT_EQ(ds_util::sgn(3.45), 1.0);
  EXPECT_EQ(ds_util::sgn(-2.67), -1.0);
  EXPECT_EQ(ds_util::sgn(0.0), 1.0);
}

TEST(ReferenceSmoothing, trapezoidal)
{
  double pos, vel, acc;

  std::tie(pos, vel, acc) = ds_util::goal_trajectory_trapezoidal(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  EXPECT_EQ(pos, 0.0);
  EXPECT_EQ(vel, 0.0);
  EXPECT_EQ(acc, 0.0);

  std::tie(pos, vel, acc) = ds_util::goal_trajectory_trapezoidal(1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
  EXPECT_EQ(pos, 0.0);
  EXPECT_EQ(vel, 0.0);
  EXPECT_EQ(acc, 1.0);

  std::tie(pos, vel, acc) = ds_util::goal_trajectory_trapezoidal(1.0, pos, vel, acc, 1.0, 1.0, 1.0);
  EXPECT_EQ(pos, 1.0);
  EXPECT_EQ(vel, 0.0);
  EXPECT_EQ(acc, 0.0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
