#ifndef DS_BASE_REFERENCE_BASE_PRIVATE_H
#define DS_BASE_REFERENCE_BASE_PRIVATE_H

#include "ds_param/ds_param_conn.h"

namespace ds_base
{

struct ReferenceBasePrivate
{
  ReferenceBasePrivate(): is_enabled_(false)
  {
  }
  ~ReferenceBasePrivate() = default;

  bool is_enabled_;
  ros::Publisher ref_pub_;

  ds_param::ParamConnection::Ptr param_sub_;
  ds_param::IntParam::Ptr active_reference_;
};

}
#endif //DS_BASE_REFERENCE_BASE_PRIVATE_H
