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

TEST_F(MatchFunctionsTest, match_header_pd0)
{
  boost::asio::streambuf b;
  std::ostream os(&b);

  std::vector<unsigned char> myData;
  myData.push_back(0x7f);
  myData.push_back(0x7f);
  myData.push_back(0x04);
  myData.push_back(0x00);
  myData.push_back(0x00);
  myData.push_back(0x00);

  os.write((const char *) myData.data(), myData.size() * sizeof(unsigned char));

  auto obj = match_header_pd0();

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  iterator item;
  bool found;
  auto size = b.size();
  auto start = boost::asio::buffers_begin(b.data());
  auto end = boost::asio::buffers_end(b.data());
  std::tie(item, found) = obj(start, end);
  
  ASSERT_EQ(found, true);
}

TEST_F(MatchFunctionsTest, match_header_pd0_fail)
{
  boost::asio::streambuf b;
  std::ostream os(&b);

  std::vector<unsigned char> myData;
  myData.push_back(0x7f);
  myData.push_back(0x7f);
  myData.push_back(0x08);
  myData.push_back(0x00);
  myData.push_back(0x00);
  myData.push_back(0x00);

  os.write((const char *) myData.data(), myData.size() * sizeof(unsigned char));

  auto obj = match_header_pd0();

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  iterator item;
  bool found;
  auto size = b.size();
  auto start = boost::asio::buffers_begin(b.data());
  auto end = boost::asio::buffers_end(b.data());
  std::tie(item, found) = obj(start, end);
  
  ASSERT_EQ(found,false);
}

TEST_F(MatchFunctionsTest, match_header_pd0_0length)
{
  boost::asio::streambuf b;
  std::ostream os(&b);

  std::vector<unsigned char> myData;
  myData.push_back(0x7f);
  myData.push_back(0x7f);
  myData.push_back(0x00);
  myData.push_back(0x00);
  myData.push_back(0x00);
  myData.push_back(0x00);

  os.write((const char *) myData.data(), myData.size() * sizeof(unsigned char));

  auto obj = match_header_pd0();

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  iterator item;
  bool found;
  auto size = b.size();
  auto start = boost::asio::buffers_begin(b.data());
  auto end = boost::asio::buffers_end(b.data());
  std::tie(item, found) = obj(start, end);
  
  ASSERT_EQ(found, true);
}

TEST_F(MatchFunctionsTest, match_multi_header_length_pass) {

  std::vector<std::vector<unsigned char>> headers = {{0xaf, 0x02}, {0xae, 0x02}};
  std::vector<int> lengths = {5, 7};

  std::vector<std::vector<unsigned char>> myData =
      { {0xAF, 0x02, 0x03, 0x04, 0x05}
        , {0xAE, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}
        , {0xAF, 0x02, 0x03, 0x04, 0x05}
        , {0xAF, 0x02, 0x03, 0x04, 0x05}
        , {0xAF, 0x02, 0x03, 0x04, 0x05}
        , {0xAE, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}
        , {0x00, 0x00, 0x00, 0x00, 0x00, 0xAE, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}
      };

  auto obj = match_multi_header_length(headers, lengths);
  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  for (auto data : myData){
    boost::asio::streambuf b;
    std::ostream os(&b);
    ROS_INFO_STREAM("Data "<< data.data());
    os.write((const char *) data.data(), data.size() * sizeof(unsigned char));
    iterator item;
    bool found;
    auto size = b.size();
    auto start = boost::asio::buffers_begin(b.data());
    auto end = boost::asio::buffers_end(b.data());
    std::tie(item, found) = obj(start, end);
    ASSERT_EQ(found, true);
  }
}

TEST_F(MatchFunctionsTest, match_multi_header_length_poorlydefined) {

  std::vector<std::vector<unsigned char>> headers = {{}};
  std::vector<int> lengths = {5};

  std::vector<std::vector<unsigned char>> myData =
      { {0xAF, 0x02, 0x03, 0x04, 0x05}
      };

  auto obj = match_multi_header_length(headers, lengths);
  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  for (auto data : myData){
    boost::asio::streambuf b;
    std::ostream os(&b);
    ROS_INFO_STREAM("Data "<< data.data());
    os.write((const char *) data.data(), data.size() * sizeof(unsigned char));
    iterator item;
    bool found;
    auto size = b.size();
    auto start = boost::asio::buffers_begin(b.data());
    auto end = boost::asio::buffers_end(b.data());
    std::tie(item, found) = obj(start, end);
    ASSERT_EQ(found, false);
  }
}

TEST_F(MatchFunctionsTest, match_multi_header_length_fail) {

  std::vector<std::vector<unsigned char>> headers = {{0xaf}, {0xae}};
  std::vector<int> lengths = {5, 7};

  std::vector<std::vector<unsigned char>> myData =
      { {0xAD, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A} //wrong header
        , {0xAE}
        , {0xAF, 0x02, 0x03}
      };

  auto obj = match_multi_header_length(headers, lengths);
  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  for (auto data : myData){
    boost::asio::streambuf b;
    std::ostream os(&b);
    ROS_INFO_STREAM("Data "<< data.data());
    os.write((const char *) data.data(), data.size() * sizeof(unsigned char));
    iterator item;
    bool found;
    auto size = b.size();
    auto start = boost::asio::buffers_begin(b.data());
    auto end = boost::asio::buffers_end(b.data());
    std::tie(item, found) = obj(start, end);
    ASSERT_EQ(found, false);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  ros::init(argc, argv, "serial_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
