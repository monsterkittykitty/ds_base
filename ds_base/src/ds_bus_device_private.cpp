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

    void DsBusDevice::Impl::setup(ds_base::DsProcess *base) {
        // setup all our connection stuff FIRST
        ds_base::DsProcess::Impl::setup(base);

        setupIoSM(base);
    }

    void DsBusDevice::Impl::setupParameters(ds_base::DsProcess *base) {
        ds_base::DsProcess::Impl::setupParameters(base);

        message_timeout_ = ros::Duration(ros::param::param<double>("~message_timeout", 5));

        auto serial_num = ros::param::param<std::string>("~serial_number", "0");
        auto generated_uuid = ds_base::generateUuid(serial_num);

        if(uuid_ != generated_uuid) {
            ROS_ERROR_STREAM("!!!POTENTIAL CONFIGURATION MISMATCH!!!");
            ROS_ERROR_STREAM("Detected UUID mismatch.");
            ROS_ERROR_STREAM("UUID (param server): " << uuid_);
            ROS_ERROR_STREAM("UUID (generated): " << generated_uuid);
        }
        else {
            ROS_INFO_STREAM("UUID matches: " << uuid_);
        }

        bus_node_name_ = ros::param::param<std::string>("~bus_node", "");
    }

    void DsBusDevice::Impl::setupConnections(ds_base::DsProcess *base) {
        ds_base::DsProcess::Impl::setupConnections(base);

        // Connect to the bus
        if (bus_node_name_.empty()) {
            ROS_FATAL_STREAM("No bus specified for bus device node " <<ros::this_node::getName());
            ros::shutdown();
        }
        bus_ = base->subscribe(bus_node_name_ + "/bus", 10, &DsBusDevice::Impl::parseReceivedBytes, this);

        // Connect our service client to control the bus state machine
        iosm_cmd_ = base->serviceClient<ds_core_msgs::IoSMcommand>(bus_node_name_ + "/cmd");
        iosm_cmd_.waitForExistence(ros::Duration(60.0)); // wait up to 1 minute for the service to exist
        if (!iosm_cmd_.exists()) {
            ROS_FATAL_STREAM(ros::this_node::getName() <<" timed out connecting to the command & control service for the bus managed by " <<bus_node_name_);
            ros::shutdown();
        }
    }

    ds_core_msgs::IoSMcommand::Response DsBusDevice::Impl::sendIosmCommand(const ds_core_msgs::IoSMcommand::Request &cmd) {
        // TODO

    }
}