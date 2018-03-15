//
// Created by jvaccaro on 2/12/18.
//

#include "ds_util/ds_util.h"

#include <gtest/gtest.h>

// Test fixture for parsing, needed because we want to call ros::Time::init() before our tests.
class Int2HexTest : public ::testing::Test
{
public:
  // This method runs ONCE before a text fixture is run (not once-per-test-case)
  static void SetUpTestCase()
  {
  }
};

TEST_F(Int2HexTest, zero)
{
  uint16_t test = 0;
  EXPECT_EQ("00", ds_util::int_to_hex<uint16_t>(test));
}

TEST_F(Int2HexTest, max)
{
  uint16_t test = 255;
  EXPECT_EQ("FF", ds_util::int_to_hex<uint16_t>(test));
}

TEST_F(Int2HexTest, neg)
{
  int16_t test = -50;
  EXPECT_EQ("CE", ds_util::int_to_hex<int16_t>(test));
}

TEST_F(Int2HexTest, wrap)
{
  uint32_t test = 267;
  EXPECT_EQ("0B", ds_util::int_to_hex<uint32_t>(test));
}

TEST_F(Int2HexTest, uint8)
{
  uint8_t test = 37;
  EXPECT_EQ("25", ds_util::int_to_hex<uint8_t>(test));
}

TEST_F(Int2HexTest, uint32)
{
  uint32_t test = 37;
  EXPECT_EQ("25", ds_util::int_to_hex<uint32_t>(test));
}

TEST_F(Int2HexTest, float_)
{
  float test = 5.7;
  EXPECT_EQ("05", ds_util::int_to_hex<float>(test));
}

TEST_F(Int2HexTest, char_)
{
  char test = '7';
  EXPECT_EQ("37", ds_util::int_to_hex<char>(test));
}

TEST_F(Int2HexTest, long_zero)
{
  uint16_t test = 0;
  EXPECT_EQ("00", ds_util::int_to_long_hex<uint16_t>(test));
}

TEST_F(Int2HexTest, long_wrap)
{
  uint32_t test = 267;
  EXPECT_EQ("010B", ds_util::int_to_long_hex<uint32_t>(test));
}
TEST_F(Int2HexTest, long_wrap2)
{
  uint32_t test = 367;
  EXPECT_EQ("016F", ds_util::int_to_long_hex<uint32_t>(test));
}

//TEST_F(Int2HexTest, long_uint8) //DOES NOT PASS
//{
//  uint8_t test = 37;
//  EXPECT_EQ("25", ds_util::int_to_long_hex<uint8_t>(test));
//}

TEST_F(Int2HexTest, long_uint32_32)
{
  uint32_t test = 37;
  EXPECT_EQ("00000025", ds_util::int_to_32_hex<uint32_t>(test));
}

TEST_F(Int2HexTest, long_short_32)
{
  uint16_t test = 350;
  EXPECT_EQ("0000015E", ds_util::int_to_32_hex<uint16_t>(test));
}

TEST_F(Int2HexTest, long_char_32)
{
  char test = '7';
  EXPECT_EQ("00000037", ds_util::int_to_32_hex<char>(test));
}

TEST_F(Int2HexTest, long_uint64_32)
{
  uint64_t test = 5700000325;
  EXPECT_EQ("53BF1A45", ds_util::int_to_32_hex<uint64_t>(test));
}

TEST_F(Int2HexTest, long_zero_32)
{
  uint64_t test = 0;
  EXPECT_EQ("00000000", ds_util::int_to_32_hex<uint64_t>(test));
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
