#ifndef DS_BASE_CONTROLLER_BASE_PRIVATE_H
#define DS_BASE_CONTROLLER_BASE_PRIVATE_H

#include "ds_base/controller_base.h"
#include "ds_param/ds_param_conn.h"

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

  ds_param::ParamConnection::Ptr param_sub_;
  ds_param::IntParam::Ptr active_controller_;

};

}

#endif //DS_BASE_CONTROLLER_BASE_PRIVATE_H
