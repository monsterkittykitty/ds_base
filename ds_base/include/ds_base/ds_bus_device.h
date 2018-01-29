//
// Created by ivaughn on 1/15/18.
//

#ifndef PROJECT_DS_BUS_DEVICE_H
#define PROJECT_DS_BUS_DEVICE_H

#include <string>
#include <memory>
#include "ds_process.h"

namespace ds_base
{
// Forward declaration of implementation details class
struct DsBusDevicePrivate;

class DsBusDevice : public ds_base::DsProcess
{
  DS_DECLARE_PRIVATE(DsBusDevice)

public:
  explicit DsBusDevice();
  DsBusDevice(int argc, char* argv[], const std::string& name);
  ~DsBusDevice() override;
  DS_DISABLE_COPY(DsBusDevice)

  /// @brief Overall setup function.
  ///
  /// Default implementation adds the setup I/O state machine stuff
  /// in the correct place.
  void setup() override;

protected:
  /// @brief setup parameters
  ///
  /// The default override checks for the following additional
  /// parameters:
  ///  - ~serial_number  (defaults to 0)
  ///  - ~uuid  (defaults to the nil UUID)
  ///
  ///  The generated uuid will be used if ~uuid is not present.  If ~uuid IS present,
  ///  then it is checked against the generated value.
  /// \param base
  void setupParameters() override;

  /// @brief Setup a connection to the bus object
  ///
  /// The default override connects to a node given by the "bus" parameter
  /// and is connected to the parseReceivedBytes method.  Note that
  /// events are generated for ANY traffic on the bus, not just
  /// traffic in response to this node's commands
  ///
  /// \param base
  void setupConnections() override;

  /// @brief Setup the I/O State Machine entries for this node
  ///
  /// This is our chance to set up I/O state machine entries for
  /// our node.  Check out the various functions to manipulate the state
  /// machine (below)
  ///
  /// \param base
  virtual void setupIoSM();

  /// @brief Handle bytes received from the bus
  ///
  /// Overload this method to parse raw bytes from the bus
  /// Note that this is called for ALL bus traffic; you have to do your own filtering.
  ///
  /// \param bytes The raw data from the bus
  virtual void parseReceivedBytes(const ds_core_msgs::RawData& bytes){};


  // Getters & setters
  /// \brief Get access to the device UUID
  const boost::uuids::uuid& Uuid() const;

  /// \brief Get non-const access to the device UUID
  boost::uuids::uuid& Uuid();

  /// \brief Get access to the message timeout duration.  Useful when checking status
  const ros::Duration& MessageTimeout() const;

  /// \brief Get non-const access to the message timeout duration.
  ros::Duration& MessageTimeout();

  /// \brief Get the name of the node managing this bus
  const std::string& BusNodeName() const ;

  /// \brief Accessor for the bus topic subscriber
  ros::Subscriber& Bus();

  /// \brief Accessor for the publisher to publish preempt commands to the bus
  ros::Publisher& PreemptCmd();

  /// \brief Accessor for the publisher to update existing commands
  ros::Publisher& UpdateCmd();

  /// \brief Acessor for the service client for IoSM commands on the Bus node
  ros::ServiceClient& IosmCmd();

private:
  /// @brief Access the underlying pimpl pointer.
  auto ds_bus_device_func() noexcept -> DsBusDevicePrivate*
  {
    return reinterpret_cast<DsBusDevicePrivate*>(d_ptr_.get());
  }

  /// @brief Access the underlying pimpl pointer.
  auto ds_bus_device_func() const noexcept -> DsBusDevicePrivate const*
  {
    return reinterpret_cast<DsBusDevicePrivate const*>(d_ptr_.get());
  }

  std::unique_ptr<DsBusDevicePrivate> d_ptr_;
};

}  // namespace ds_base

#endif  // PROJECT_DS_BUS_DEVICE_H
