#ifndef DS_MATCH_FUNCTIONS_H
#define DS_MATCH_FUNCTIONS_H

//typedef boost::asio::buffers_iterator< boost::asio::streambuf::const_buffers_type > Iterator;

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

#endif
