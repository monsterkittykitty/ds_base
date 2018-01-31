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
  ds_nav_msgs::AggregatedState last_reference_;
  ds_nav_msgs::AggregatedState last_state_;

  ros::Subscriber state_update_sub_;
  ros::Subscriber reference_update_sub_;
};

}

#endif //DS_BASE_CONTROLLER_BASE_PRIVATE_H
