//
// Created by jvaccaro on 10/4/18.
//

#include "ds_util/ds_util.h"

#include <gtest/gtest.h>

// Test fixture for parsing, needed because we want to call ros::Time::init() before our tests.
class Bcd2IntTest : public ::testing::Test
{
 public:
  // This method runs ONCE before a text fixture is run (not once-per-test-case)
  static void SetUpTestCase()
  {
  }
};

TEST_F(Bcd2IntTest, zero)
{
  uint16_t test = 0x00;
  EXPECT_EQ(0, ds_util::bcd_to_int(test));
}

TEST_F(Bcd2IntTest, try_35)
{
  uint16_t test = 0x35;
  EXPECT_EQ(35, ds_util::bcd_to_int(test));
}

TEST_F(Bcd2IntTest, try_max)
{
  int16_t test = 0x99;
  EXPECT_EQ(99, ds_util::bcd_to_int(test));
}

TEST_F(Bcd2IntTest, try_overflow)
{
  int16_t test = 0xAA;
  EXPECT_EQ(0, ds_util::bcd_to_int(test));
}

TEST_F(Bcd2IntTest, try_ones_overflow)
{
  int16_t test = 0x0A;
  EXPECT_EQ(0, ds_util::bcd_to_int(test));
}

TEST_F(Bcd2IntTest, try_tens_overflow)
{
  int16_t test = 0xA0;
  EXPECT_EQ(0, ds_util::bcd_to_int(test));
}



// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
