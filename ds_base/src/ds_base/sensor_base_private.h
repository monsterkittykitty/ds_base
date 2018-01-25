#ifndef DS_SENSOR_SENSOR_BASE_PRIVATE_H
#define DS_SENSOR_SENSOR_BASE_PRIVATE_H

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
struct SensorBasePrivate
{
  SensorBasePrivate() = default;
  virtual ~SensorBasePrivate() = default;

  DS_DISABLE_COPY(SensorBasePrivate)

  ros::Duration message_timeout_;  //!< Acceptable duration between sensor messages.

  ///@brief last timestamps, checked by SensorBase::checkTimestamps
  SensorBase::TimestampMap last_timestamps_;

  ///@brief Connections available to the SensorBase::sendCommand method
  SensorBase::ConnectionMap connections_;
  ros::ServiceServer send_command_service_;  //!< ros::Service for sending commands
};
}

#endif  // DS_SENSOR_SENSOR_BASE_PRIVATE_H
