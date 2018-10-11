/**
* Copyright 2018 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
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
  auto result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // 0 separation
  from = 180;
  to = 180;
  expected = 0;
  result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  from = 0;
  to = 15;
  expected = 15;
  result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  from = 15;
  to = 30;
  expected = 15;
  result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Backwards should be negative
  from = 30;
  to = 15;
  expected = -15;
  result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Backwards around 0
  from = 10;
  to = -10;
  expected = -20;
  result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Forwards around 0
  from = -10;
  to = 10;
  expected = 20;
  result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Forwards around 0 using heading values
  from = 350;
  to = 10;
  expected = 20;
  result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Backwards around 0 using heading values
  from = 10;
  to = 350;
  expected = -20;
  result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Angle separations are in the interval (-pi, pi]
  from = 0;
  to = 270;
  expected = -90;
  result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Angle separations are in the interval (-pi, pi]
  from = 0;
  to = -270;
  expected = 90;
  result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  // Angle separations are in the interval (-pi, pi]
  from = 0;
  to = 180;
  expected = 180;
  result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  from = 180;
  to = 0;
  expected = -180;
  result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  from = 185;
  to = 180;
  expected = -5;
  result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);

  from = 175;
  to = 180;
  expected = 5;
  result = ds_util::angular_separation_radians(from * M_PI / 180, to * M_PI / 180);
  EXPECT_NEAR(expected, result * 180 / M_PI, 0.1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
