#ifndef DS_UTIL_INT2HEX_H
#define DS_UTIL_INT2HEX_H

#include <string>
#include <sstream>
#include <iomanip>

namespace ds_util
{
template <typename T1>
static std::string int_to_hex(T1 i)
{
  uint16_t j = i;  // Recast to uint16 so that it ill be treated as an integer, not a char
  j &= 0x00FF;     // Ensure that the integer remains within uint8 range
  std::stringstream stream;
  stream << std::uppercase << std::setfill('0') << std::setw(sizeof(uint8_t) * 2) << std::hex << j;
  return stream.str();
}

template <typename T2>
static std::string int_to_32_hex(T2 i)
{
  uint16_t j;
  int s = sizeof(uint32_t) * 2;
  std::stringstream stream;
  for (int it = 1; it <= s; it++)
  {
    j = i >> (s - it) * 4;  // Recast to uint16 so that it ill be treated as an integer, not a char
    j &= 0x000F;            // Ensure that the integer remains within uint4 range
    stream << std::uppercase << std::setfill('0') << std::setw(sizeof(uint8_t)) << std::hex << j;
  }

  return stream.str();
}

template <typename T3>
static std::string int_to_long_hex(T3 i)
{
  uint16_t j;
  int s = sizeof(i);
  std::stringstream stream;
  for (int it = 1; it <= s; it++)
  {
    j = i >> (s - it) * 4;  // Recast to uint16 so that it ill be treated as an integer, not a char
    j &= 0x000F;            // Ensure that the integer remains within uint8 range
    stream << std::uppercase << std::setfill('0') << std::setw(sizeof(uint8_t)) << std::hex << j;
  }

  return stream.str();
}
}

#endif  // DS_UTIL_INT2HEX_H
