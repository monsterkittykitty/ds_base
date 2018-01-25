//
// Created by ivaughn on 1/15/18.
//

#include "ds_bus_device_private.h"
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/nil_generator.hpp>

namespace ds_base
{
DsBusDevicePrivate::DsBusDevicePrivate()
  : message_timeout_(ros::Duration(-1)), uuid_(boost::uuids::nil_uuid())
{
}

ds_core_msgs::IoSMcommand::Response DsBusDevicePrivate::sendIosmCommand(const ds_core_msgs::IoSMcommand::Request& cmd)
{
  // TODO
}
}