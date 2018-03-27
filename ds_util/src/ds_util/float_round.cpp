//
// Created by jvaccaro on 3/27/18.
//

#include "../../include/ds_util/float_round.h"

namespace ds_util{

    float float_round(float num, int dec) {
        float pwr = std::pow(10, dec);
        return std::round(pwr * num) / pwr;
    }
}
