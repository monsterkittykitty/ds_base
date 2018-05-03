#include "ds_asio/ds_asio.h"
#include "ds_asio/ds_match_functions.h"
#include <gtest/gtest.h>
#include <ostream>
#include "boost/asio/streambuf.hpp"

class MatchFunctionsTest: public ::testing::Test
{
 protected:

  void SetUp() override
  {
  }

  void TearDown() override
  {
  }

};

TEST_F(MatchFunctionsTest, match_char)
{
  boost::asio::streambuf b;
  std::ostream os(&b);

  os << "Test matcher!\n";

  auto obj = match_char('\n');

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  iterator item;
  bool found;
  auto size = b.size();
  auto start = boost::asio::buffers_begin(b.data());
  auto end = boost::asio::buffers_end(b.data());
  std::tie(item, found) = obj(start, end);
  
  ASSERT_EQ(found, true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  ros::init(argc, argv, "serial_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
