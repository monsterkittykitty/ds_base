//
// Created by jvaccaro on 3/27/18.
//

#include "ds_util/ds_util.h"
#include "ds_util/float_round.h"

#include <gtest/gtest.h>

// Test fixture for parsing, needed because we want to call ros::Time::init() before our tests.
class FloatRoundTest : public ::testing::Test
{
public:
    // This method runs ONCE before a text fixture is run (not once-per-test-case)
    static void SetUpTestCase()
    {
    }
};

TEST_F(FloatRoundTest, round_zero)
{
float flt = 0.0;
EXPECT_FLOAT_EQ(0.0, ds_util::float_round(flt, 10));
}

TEST_F(FloatRoundTest, round_levels)
{
float flt = 1.1234;
EXPECT_FLOAT_EQ(1.1234, ds_util::float_round(flt, 4));
EXPECT_FLOAT_EQ(1.123, ds_util::float_round(flt, 3));
EXPECT_FLOAT_EQ(1.12, ds_util::float_round(flt, 2));
EXPECT_FLOAT_EQ(1.1, ds_util::float_round(flt, 1));
EXPECT_FLOAT_EQ(1, ds_util::float_round(flt, 0));
}

TEST_F(FloatRoundTest, round_up)
{
EXPECT_FLOAT_EQ(2, ds_util::float_round(1.5, 0));
EXPECT_FLOAT_EQ(1.2, ds_util::float_round(1.15, 1));
EXPECT_FLOAT_EQ(1.12, ds_util::float_round(1.115, 2));
EXPECT_FLOAT_EQ(2.0, ds_util::float_round(1.999, 2));
EXPECT_FLOAT_EQ(1.3, ds_util::float_round(1.25, 1));
}

TEST_F(FloatRoundTest, double_round)
{
EXPECT_FLOAT_EQ(2, ds_util::float_round(ds_util::float_round(1.45, 1), 0));
EXPECT_FLOAT_EQ(1.1, ds_util::float_round(ds_util::float_round(1.115, 2), 1));
EXPECT_FLOAT_EQ(1.46, ds_util::float_round(ds_util::float_round(1.4555, 3), 2));
EXPECT_FLOAT_EQ(1.192, ds_util::float_round(ds_util::float_round(1.1919995, 5), 4));
EXPECT_FLOAT_EQ(1.6869, ds_util::float_round(ds_util::float_round(1.68686841, 6), 4));
}


// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
