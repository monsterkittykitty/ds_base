//
// Created by ivaughn on 1/15/18.
//

#include "ds_base/ds_bus_device.h"
#include "ds_base/ds_bus_device_private.h"

using namespace ds_base;

// See EXTENDING.md
// Our constructors use the protected constructor from `DsProcess`, providing our
// own version of the private implementation class.
//
// This newly constructed DsBus::Impl gets implicitly upcast to DsProcess::Impl
// when passed to DsProcess's constructor.
//
// NOTE:  Our public constructors just forward on to our protected versions.  If
// we end up needing to add logic inside the constructors we'll only have to add
// it in two places now (the protected versions) instead of all four.
// Public default constructor:  use our own protected anolog
DsBusDevice::DsBusDevice() : DsBusDevice(std::unique_ptr<Impl>(new Impl))
{
  // do nothing
}

// Another public->protected forwarding.
DsBusDevice::DsBusDevice(int argc, char* argv[], const std::string& name)
  : DsBusDevice(std::unique_ptr<Impl>(new Impl), argc, argv, name)
{
  // do nothing
}

// Protected 'default' constructor
DsBusDevice::DsBusDevice(std::unique_ptr<Impl> impl) : ds_base::DsProcess(std::move(impl))
{
  // do nothing
}

// Protected constructor with arguments for ros::init
DsBusDevice::DsBusDevice(std::unique_ptr<Impl> impl, int argc, char* argv[], const std::string& name)
  : ds_base::DsProcess(std::move(impl), argc, argv, name)
{
  // do nothing
}

//
// This is how we get access to our new private DsBus::Impl.
// See, in the constructors above, we upcast DsBus::Impl into SensorBase::Impl, where
// it's stored in the SensorBase::impl_ member.
//
// To get the Impl class back *in the propper type* we need to downcast it again before
// working on it, which is why we have the static_cast<>'s here.
//
inline auto DsBusDevice::d_func() noexcept -> DsBusDevice::Impl*
{
  return static_cast<DsBusDevice::Impl*>(ds_base::DsProcess::d_func());
}

inline auto DsBusDevice::d_func() const noexcept -> DsBusDevice::Impl const*
{
  return static_cast<DsBusDevice::Impl const*>(ds_base::DsProcess::d_func());
}

void DsBusDevice::setup()
{
  // setup all our connection stuff FIRST
  ds_base::DsProcess::setup();

  setupIoSM();
}

void DsBusDevice::setupParameters()
{
  ds_base::DsProcess::setupParameters();

  auto d = d_func();
  d->message_timeout_ = ros::Duration(ros::param::param<double>("~message_timeout", 5));

  auto serial_num = ros::param::param<std::string>("~serial_number", "0");
  d->uuid_ = ds_base::generateUuid(serial_num);

<<<<<<< HEAD
  ROS_INFO_STREAM("Setting device UUID to: " << d->uuid_);
=======
  if (d->uuid_ != generated_uuid)
  {
    ROS_ERROR_STREAM("!!!POTENTIAL CONFIGURATION MISMATCH!!!");
    ROS_ERROR_STREAM("Detected UUID mismatch.");
    ROS_ERROR_STREAM("UUID (param server): " << d->uuid_);
    ROS_ERROR_STREAM("UUID (generated): " << generated_uuid);
  }
  else
  {
    ROS_INFO_STREAM("UUID matches: " << d->uuid_);
  }
>>>>>>> 537a06e6a8d8feb290041c632083101483aae1f6

  d->bus_node_name_ = ros::param::param<std::string>("~bus_node", "");
}

void DsBusDevice::setupConnections()
{
  ds_base::DsProcess::setupConnections();

  auto d = d_func();
  // Connect to the bus
<<<<<<< HEAD
  if (d->bus_node_name_.empty()) {
    ROS_FATAL_STREAM("No bus_node specified for bus device node " <<ros::this_node::getName());
=======
  if (d->bus_node_name_.empty())
  {
    ROS_FATAL_STREAM("No bus specified for bus device node " << ros::this_node::getName());
>>>>>>> 537a06e6a8d8feb290041c632083101483aae1f6
    ros::shutdown();
  }
  d->bus_ = subscribe(d->bus_node_name_ + "/bus", 10, &DsBusDevice::parseReceivedBytes, this);

  // Connect our service client to control the bus state machine
  d->iosm_cmd_ = serviceClient<ds_core_msgs::IoSMcommand>(d->bus_node_name_ + "/cmd");
  d->iosm_cmd_.waitForExistence(ros::Duration(60.0));  // wait up to 1 minute for the service to exist
  if (!d->iosm_cmd_.exists())
  {
    ROS_FATAL_STREAM(ros::this_node::getName()
                     << " timed out connecting to the command & control service for the bus managed by "
                     << d->bus_node_name_);
    ros::shutdown();
  }
}
