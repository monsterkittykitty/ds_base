#ifndef DS_BASE_CONTROLLER_BASE_PRIVATE_H
#define DS_BASE_CONTROLLER_BASE_PRIVATE_H

#include "ds_base/controller_base.h"

namespace ds_base
{

struct ControllerBasePrivate
{
  ControllerBasePrivate()
      : is_enabled_(false)
  {
  }
  ~ControllerBasePrivate() = default;

  bool is_enabled_;

};

}

#endif //DS_BASE_CONTROLLER_BASE_PRIVATE_H
