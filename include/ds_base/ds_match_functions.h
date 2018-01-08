#ifndef DS_MATCH_FUNCTIONS_H
#define DS_MATCH_FUNCTIONS_H

#include "boost/circular_buffer.hpp"

/// @brief Generates a matcher class for async_read_until. This class makes async_read_until return when the delimiter matches the char c
///
/// @return iterator and true pair if the delimiter is identified, iterator and false pair if the delimiter is not identified
///
class match_char
{
public:
  explicit match_char(char c) : c_(c) {}

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

namespace boost{
  namespace asio {
    template <> struct is_match_condition<match_char>
      : public boost::true_type {};
  } // namespace asio
} // namespace boost

/// @brief Generates a matcher class for async_read_until. This class makes async_read_until return when the binary header is identified and the binary data received matches the expected length
///
/// @return iterator and true pair if the data packet is complete, iterator and false pair otherwise
///
class match_header_length
{
public:
  explicit match_header_length(std::vector<unsigned char> header, int length) : header_(header), length_(length), len_(0), sync_(false), found_(header.size(), false), cb_(header.size()) {}

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  template <typename Iterator>
  std::pair<Iterator, bool> operator()(Iterator begin, Iterator end)
    {
      Iterator i = begin;
      while (i != end)
	{
	  // Add current byte to the circular buffer
	  cb_.push_back(*i++);
	  // If the stream is not synchronized
	  if (!sync_)
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
	      return std::make_pair(i, false);
	    }
	  else if (sync_)
	    {
	      if (len_ < length_)
		{
		  *i++;
		  len_++;
		  return std::make_pair(i, false);
		}
	      // We reached the expected length of the binary frame, tell async_read_until that we're done reading this frame
	      else
		{
		  return std::make_pair(i, true);
		}
	    }
	}
    }

private:
  std::vector<unsigned char> header_;
  boost::circular_buffer<unsigned char> cb_;
  std::vector<bool> found_;
  int length_;
  int len_;
  bool sync_;

};

namespace boost{
  namespace asio {
    template <> struct is_match_condition<match_header_length>
      : public boost::true_type {};
  } // namespace asio
} // namespace boost


/// @brief Generates a matcher class for async_read_until. This class makes async_read_until return with every byte
///
/// @return iterator and true pair, meaning that the match condition is always true
///
class passthrough
{
public:
  explicit passthrough() {}

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  template <typename Iterator>
  std::pair<Iterator, bool> operator()(
      Iterator begin, Iterator end) const
  {
    Iterator i = begin;

    return std::make_pair(i, true);
  }

};

namespace boost{
  namespace asio {
    template <> struct is_match_condition<passthrough>
      : public boost::true_type {};
  } // namespace asio
} // namespace boost

#endif
