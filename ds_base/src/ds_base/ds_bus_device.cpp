//
// Created by ivaughn on 1/15/18.
//

#include <ds_core_msgs/IoCommandList.h>
#include "ds_base/ds_bus_device.h"
#include "ds_bus_device_private.h"

using namespace ds_base;

// See EXTENDING.md
// Our constructors use the protected constructor from `DsProcess`, providing our
// own version of the private implementation class.
//
// This newly constructed DsBus::Private gets implicitly upcast to DsProcess::Private
// when passed to DsProcess's constructor.
//
// NOTE:  Our public constructors just forward on to our protected versions.  If
// we end up needing to add logic inside the constructors we'll only have to add
// it in two places now (the protected versions) instead of all four.
// Public default constructor:  use our own protected anolog
DsBusDevice::DsBusDevice()
    : DsProcess()
    , d_ptr_(std::unique_ptr<DsBusDevicePrivate>(new DsBusDevicePrivate))
{
  // do nothing
}

// Another public->protected forwarding.
DsBusDevice::DsBusDevice(int argc, char* argv[], const std::string& name)
  : DsProcess(argc, argv, name)
  , d_ptr_(std::unique_ptr<DsBusDevicePrivate>(new DsBusDevicePrivate))
{
  // do nothing
}

DsBusDevice::~DsBusDevice() = default;

void DsBusDevice::setup()
{
  // setup all our connection stuff FIRST
  ds_base::DsProcess::setup();

  setupIoSM();
}

void DsBusDevice::setupParameters()
{
  ds_base::DsProcess::setupParameters();

  DS_D(DsBusDevice);
  d->message_timeout_ = ros::Duration(ros::param::param<double>("~message_timeout", 5));

  auto serial_num = ros::param::param<std::string>("~serial_number", "0");
  d->uuid_ = ds_base::generateUuid(serial_num);

  ROS_INFO_STREAM("Setting device UUID to: " << d->uuid_);

  d->bus_node_name_ = ros::param::param<std::string>("~bus_node", "");
}

void DsBusDevice::setupConnections()
{
  ds_base::DsProcess::setupConnections();

  DS_D(DsBusDevice);
  // Connect to the bus
  if (d->bus_node_name_.empty()) {
    ROS_FATAL_STREAM("No bus_node specified for bus device node " <<ros::this_node::getName());
    ros::shutdown();
  }
  d->bus_ = nodeHandle()->subscribe(d->bus_node_name_ + "/bus", 10, &DsBusDevice::parseReceivedBytes, this);

  // Connect our service client to control the bus state machine
  d->iosm_cmd_ = nodeHandle()->serviceClient<ds_core_msgs::IoSMcommand>(d->bus_node_name_ + "/cmd");
  d->iosm_cmd_.waitForExistence(ros::Duration(60.0));  // wait up to 1 minute for the service to exist
  if (!d->iosm_cmd_.exists())
  {
    ROS_FATAL_STREAM(ros::this_node::getName()
                     << " timed out connecting to the command & control service for the bus managed by "
                     << d->bus_node_name_);
    ros::shutdown();
  }

  d->preempt_cmd_ = nodeHandle()->advertise<ds_core_msgs::IoCommandList>(d->bus_node_name_ + "/preempt_cmd", 10, false);
  d->update_cmd_ = nodeHandle()->advertise<ds_core_msgs::IoCommandList>(d->bus_node_name_ + "/update_cmd", 10, false);
}
