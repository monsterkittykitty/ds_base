//
// Created by jvaccaro on 10/4/18.
//

#include "../../include/ds_util/bcd_to_int.h"

namespace ds_util{

uint16_t
bcd_to_int(uint8_t bcd){
  // Split into digits
  uint16_t ones = (bcd & 0x000f);
  uint16_t tens = (bcd>>4);
  // Confirm that bcd is a valid encoding
  if (ones > 0x0009 || tens > 0x0009){
    return 0;
  }
  // Return the hex encoding
  return tens*10 + ones;
}
}

