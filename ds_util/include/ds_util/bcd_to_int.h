//
// Created by jvaccaro on 10/4/18.
//


#ifndef DS_UTIL_BCD_TO_INT_H
#define DS_UTIL_BCD_TO_INT_H

#include <cstdint>

namespace ds_util{

/// \brief Converts binary coded decimal (bcd) into actual decimal value
///
/// e.g. 0x24 bcd --> 24dec = 0x18
/// e.g. 0x53 bcd --> 53dec = 0x35
///
/// if integer is out of range (not bcd) then the fxn returns 0.
/// e.g. 0xAA bcd --> 0dec = 0x00
/// \param bcd
/// \return integer 0-99
  uint16_t bcd_to_int(uint8_t bcd);
}
#endif //DS_UTIL_BCD_TO_INT_H
