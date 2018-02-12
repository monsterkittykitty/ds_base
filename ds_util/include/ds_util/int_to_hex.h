#ifndef DS_UTIL_INT2HEX_H
#define DS_UTIL_INT2HEX_H

#include <string>
#include <sstream>
#include <iomanip>

namespace ds_util
{

    template< typename T >
    static std::string int_to_hex( T i )
    {
        uint16_t j = i; //Recast to uint16 so that it ill be treated as an integer, not a char
        j &= 0x00FF; //Ensure that the integer remains within uint8 range
        std::stringstream stream;
        stream << std::uppercase
               << std::setfill ('0') << std::setw(sizeof(uint8_t)*2)
               << std::hex << j;
        return stream.str();
    }

}

#endif //DS_UTIL_INT2HEX_H
