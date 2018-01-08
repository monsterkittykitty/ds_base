#ifndef DS_MATCH_FUNCTIONS_H
#define DS_MATCH_FUNCTIONS_H

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
  std::pair<Iterator, bool> operator()(
      Iterator begin, Iterator end) const
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
  explicit match_header_length(std::vector<unsigned char> header, int length) : header_(header), length_(length) {}

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  template <typename Iterator>
  std::pair<Iterator, bool> operator()(
      Iterator begin, Iterator end) const
  {
    ;
  }

private:
  std::vector<unsigned char> header_;
  int length_;
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
