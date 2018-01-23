//
// Created by zac on 12/5/17.
//

#ifndef DS_SENSOR_SENSOR_BASE_PRIVATE_H
#define DS_SENSOR_SENSOR_BASE_PRIVATE_H

#include "ds_base/ds_process_private.h"
#include "ds_base/sensor_base.h"
#include "ds_core_msgs/Status.h"

#include <boost/any.hpp>
#include <boost/uuid/uuid.hpp>

#include <unordered_map>

namespace ds_base
{
/// @brief Private implmentation for SensorBase class
///
/// This structure hides the actual implementation details for classes based on SensorBase.
struct SensorBase::Impl : public ds_base::DsProcess::Impl
{
  Impl();
  virtual ~Impl() = default;

  // Disable copy operations.
  Impl(const Impl&) = delete;
  void operator=(const Impl&) = delete;

  ros::ServiceServer send_command_service_;  //!< ros::Service for sending commands
};
}

#endif  // DS_SENSOR_SENSOR_BASE_PRIVATE_H
