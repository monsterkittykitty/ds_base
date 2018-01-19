//
// Created by ivaughn on 1/15/18.
//

#include <ds_base/ds_bus_device_private.h>
#include <ds_base/ds_process.h>


#include <ds_base/util.h>

#include <ds_core_msgs/Status.h>
#include <ds_core_msgs/RawData.h>
#include <ds_core_msgs/IoSMcommand.h>

#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/nil_generator.hpp>

namespace ds_base {
    DsBusDevice::Impl::Impl() : ds_base::DsProcess::Impl(),
    message_timeout_ (ros::Duration(-1)),
    uuid_(boost::uuids::nil_uuid()) {

    }


    ds_core_msgs::IoSMcommand::Response DsBusDevice::Impl::sendIosmCommand(const ds_core_msgs::IoSMcommand::Request &cmd) {
        // TODO

    }
}