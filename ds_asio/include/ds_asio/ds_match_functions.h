#ifndef DS_MATCH_FUNCTIONS_H
#define DS_MATCH_FUNCTIONS_H

#include "boost/circular_buffer.hpp"

/// @brief Generates a matcher class for async_read_until. This class makes async_read_until return when the delimiter
/// matches the char c
///
/// @return iterator and true pair if the delimiter is identified, iterator and false pair if the delimiter is not
/// identified
///
class match_char
{
public:
  explicit match_char(char c) : c_(c)
  {
  }

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  template <typename Iterator>
  std::pair<Iterator, bool> operator()(Iterator begin, Iterator end) const
  {
    Iterator i = begin;
    while (i != end)
      if (c_ == *i++)
        return std::make_pair(i, true);
    return std::make_pair(i, false);
  }

private:
  char c_;
};

namespace boost
{
namespace asio
{
template <>
struct is_match_condition<match_char> : public boost::true_type
{
};
}  // namespace asio
}  // namespace boost

/// @brief Generates a matcher class for async_read_until. This class makes async_read_until return when the binary
/// header is identified and the binary data received matches the expected length
///
/// @return iterator and true pair if the data packet is complete, iterator and false pair otherwise
///
class match_header_length
{
public:
  explicit match_header_length(std::vector<unsigned char> header, int length)
    : header_(header), length_(length), len_(0), sync_(false), found_(header.size(), false), cb_(header.size())
  {
    ROS_INFO_STREAM("Matcher set " << cb_.capacity());
  }

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  template <typename Iterator>
  std::pair<Iterator, bool> operator()(Iterator begin, Iterator end)
  {
    Iterator i = begin;
    while (i != end)
    {
      cb_.push_back(*i++);
      // If the stream is not synchronized, access cb_ only if it's full i.e. size is equal to capacity
      if ((!sync_) && (cb_.full()))
      {
        // Update the found_ vector that stores matching header bytes status
        for (int j = 0; j < header_.size(); ++j)
        {
          found_[j] = (cb_[j] == header_[j] ? true : false);
        }
        // If all the found_ vector is true, then we are synchronized to the binary frame
        if (std::all_of(found_.begin(), found_.end(), [](bool v) { return v; }))
        {
          sync_ = true;
          // Increment the binary frame len_ that we already read by the size of the header
          len_ += header_.size();
        }
      }
      else if (sync_)
      {
        len_++;
        // We reached the expected length of the binary frame, tell async_read_until that we're done reading this frame
        if (len_ == length_)
        {
          ROS_INFO_STREAM("Buffering ended, length: " << len_);
          return std::make_pair(i, true);
        }
      }
    }
    return std::make_pair(i, false);
  }

private:
  std::vector<unsigned char> header_;
  boost::circular_buffer<unsigned char> cb_;
  std::vector<bool> found_;
  int length_;
  int len_;
  bool sync_;
};

namespace boost
{
namespace asio
{
template <>
struct is_match_condition<match_header_length> : public boost::true_type
{
};
}  // namespace asio
}  // namespace boost


class match_header_pd0
{
 public:
  explicit match_header_pd0() : length_(833), len_(0), sync_(false), found_(2, false), cb_(2)
    {
      ROS_INFO_STREAM("Matcher set " << cb_.capacity());
    }

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  template <typename Iterator>
  std::pair<Iterator, bool> operator()(Iterator begin, Iterator end)
    {
      Iterator i = begin;
      std::string hexAscii = "7F7F";
      std::vector<unsigned char> myHeader;
      for (int j = 0; j < hexAscii.length(); j += 2)
	{
	  std::string byteString = hexAscii.substr(j, 2);
	  unsigned int myByte;
	  sscanf(byteString.c_str(), "%X", &myByte);
	  myHeader.push_back((unsigned char)myByte);
	}
      header_ = myHeader;
      while (i != end)
	{
	  cb_.push_back(*i++);
	  // If the stream is not synchronized, access cb_ only if it's full i.e. size is equal to capacity
	  if ((!sync_) && (cb_.full()))
	    {
	      // Update the found_ vector that stores matching header bytes status
	      for (int j = 0; j < header_.size(); ++j)
		{
		  found_[j] = (cb_[j] == header_[j] ? true : false);
		}
	      // If all the found_ vector is true, then we are synchronized to the binary frame
	      if (std::all_of(found_.begin(), found_.end(), [](bool v) { return v; }))
		{
		  sync_ = true;
		  // Increment the binary frame len_ that we already read by the size of the header
		  len_ += header_.size();
		}
	    }
	  else if (sync_)
	    {
	      len_++;
	      // We reached the expected length of the binary frame, tell async_read_until that we're done reading this frame
	      if (len_ == 4)
		{
		  length_ = cb_[0] + 2;
		}
	      if (len_ >= length_)
		{
		  ROS_INFO_STREAM("Buffering ended, length: " << len_);
		  return std::make_pair(i, true);
		}
	    }
	}
      return std::make_pair(i, false);
    }

 private:
  std::vector<unsigned char> header_;
  boost::circular_buffer<unsigned char> cb_;
  std::vector<bool> found_;
  int length_;
  int len_;
  bool sync_;
};

namespace boost
{
  namespace asio
  {
    template <>
    struct is_match_condition<match_header_pd0> : public boost::true_type
    {
    };
  }  // namespace asio
}  // namespace boost


/// General header-length match fxns
/// Headers must all be the same length in characters

class match_multi_header_length
{
 public:
  explicit match_multi_header_length(const std::vector<std::vector<unsigned char>> header_id, const std::vector<int> length)
      : header_id_(header_id), length_(length), cb_(header_id[0].size())
      , found_(header_id.size(), std::vector<bool>(header_id[0].size(), false)), tgt_len_(0), len_(0), sync_(false)
  {
  }

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  template <typename Iterator>
  std::pair<Iterator, bool> operator()(Iterator begin, Iterator end)
  {
    Iterator i = begin;

    while (i != end)
    {
      cb_.push_back(*i++);

      if ((!sync_) && (cb_.full()))
      {
        // Update the found_ vector that stores matching header bytes status
        for (int j = 0; j < header_id_.size(); ++j)
        {
          // For each character in the buffer, check if it matches a known header
          for (int k = 0; k < header_id_[0].size(); ++k){
            found_[j][k] = (cb_[k] == header_id_[j][k] );
          }
          if (std::all_of(found_[j].begin(), found_[j].end(), [](bool v) { return v; }))
          {
            sync_ = true;
            tgt_len_ = length_[j];
            len_ += cb_.size();
          }
        }
      }
      else if (sync_)
      {
        len_++;
        if (len_ >= tgt_len_)
        {
          len_ = 0;
          sync_ = false;
          return std::make_pair(i, true);
        }
      }
    }
    return std::make_pair(i, false);
  }

 private:
  std::vector<std::vector<unsigned char>> header_id_;
  std::vector<int> length_;
  boost::circular_buffer<unsigned char> cb_;
  std::vector<std::vector<bool>> found_;
  int tgt_len_, len_;
  bool sync_;
};

namespace boost
{
namespace asio
{
template <>
struct is_match_condition<match_multi_header_length> : public boost::true_type
{
};
}  // namespace asio
}  // namespace boost


/// NORTEKVECTOR MATCH FUNCTIONS

class match_header_nortekvector
{
 public:
  explicit match_header_nortekvector() : length_(24), len_(0), sync_(false), found_data_(2, false), found_system_(2, false), cb_(2)
  {
    data_id_.push_back(0xA5);
    data_id_.push_back(0x10);
    system_id_.push_back(0xa5);
    system_id_.push_back(0x11);
    ROS_INFO_STREAM("Matcher set " << cb_.capacity());
  }

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  template <typename Iterator>
  std::pair<Iterator, bool> operator()(Iterator begin, Iterator end)
  {
    Iterator i = begin;

    while (i != end)
    {
      cb_.push_back(*i++);

      if ((!sync_) && (cb_.full()))
      {
        // Update the found_ vector that stores matching header bytes status
        for (int j = 0; j < data_id_.size() && j < cb_.size(); ++j)
        {
          found_data_[j] = (cb_[j] == data_id_[j] );
          found_system_[j] = (cb_[j] == system_id_[j]);
        }
        // If all the found_ vector is true, then we are synchronized to the binary frame
        if (std::all_of(found_data_.begin(), found_data_.end(), [](bool v) { return v; }))
        {
          sync_ = true;
          // Increment the binary frame len_ that we already read by the size of the header
          length_ = 24;
          len_ += data_id_.size();
//          if (i-2 >= begin){
//            return std::make_pair(i-2, true);
//          } else
//          if (i+24 < end){
//            return std::make_pair(i+24, true);
//          } else {
//            return std::make_pair(end, false);
//          }
        }
        else if (std::all_of(found_system_.begin(), found_system_.end(), [](bool v) { return v; }))
        {
          sync_ = true;
          // Increment the binary frame len_ that we already read by the size of the header
          length_ = 28;
          len_ += system_id_.size();
//          if (i-2 >= begin){
//            return std::make_pair(i-2, true);
//          } else
//          if (i+28 < end){
//            return std::make_pair(i+28, true);
//          } else {
//            return std::make_pair(end, false);
//          }
        }
      }

      else if (sync_)
      {
        len_++;
        ROS_INFO_STREAM("Len " << len_);
        if (len_ >= length_)
        {
          ROS_INFO_STREAM("Buffering ended, length: " << len_);
          len_ = 0;
          sync_ = false;
          return std::make_pair(i, true);
        }
      }
    }
    return std::make_pair(i, false);
  }

 private:
  std::vector<unsigned char> data_id_, system_id_;
  boost::circular_buffer<unsigned char> cb_;
  std::vector<bool> found_data_, found_system_;
  int length_;
  int len_;
  bool sync_;
};

namespace boost
{
namespace asio
{
template <>
struct is_match_condition<match_header_nortekvector> : public boost::true_type
{
};
}  // namespace asio
}  // namespace boost


/// @brief Generates a matcher class for async_read_until. This class makes async_read_until return with every byte
///
/// @return iterator and true pair, meaning that the match condition is always true
///
class passthrough
{
public:
  explicit passthrough()
  {
  }

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  template <typename Iterator>
  std::pair<Iterator, bool> operator()(Iterator begin, Iterator end) const
  {
    Iterator i = begin;

    return std::make_pair(i, true);
  }
};

namespace boost
{
namespace asio
{
template <>
struct is_match_condition<passthrough> : public boost::true_type
{
};
}  // namespace asio
}  // namespace boost

#endif
