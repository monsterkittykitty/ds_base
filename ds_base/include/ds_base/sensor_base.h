//
// Created by zac on 12/5/17.
//

#ifndef DS_SENSOR_SENSORBASE_H
#define DS_SENSOR_SENSORBASE_H

#include "ds_base/ds_process.h"

#include <boost/uuid/uuid.hpp>
#include <memory>

namespace ds_base
{
/// @brief Base class for sensors defined in ds_sensor package.
///
/// See EXTENDING.md in the repo root directory for a detailed guide to creating a new sensor based
/// on this class.
///
/// Basic properties (that can be modified in subclasses)
///
/// - Uses a single connection named "instrument" See DsAsio::setupConnections
/// - Callback to "instrument" asio source connected to SensorBase::parseReceivedBytes
/// - Looks for "message_timeout" parameter to determine acceptable interval between received messages.
///
/// The SensorBase class and it's derivatives follow the PIMPL idiom.  Qt provides some nice macros
/// for using PIMPL with derived classes (and derived PIMPLs).  For straight C++11 I found a succinct
/// example here:
///    https://stackoverflow.com/a/35192820
///
class SensorBase : public ds_base::DsProcess
{

protected:
  /// Protected implementation struct
  struct Impl;

public:
  ///@brief SensorBase constructor
  ///
  /// You must call `ros::init` separately when using this method.
  explicit SensorBase();

  ///@brief SensorBase constructor
  ///
  /// Calls ros::init(argc, argv, name) for you.
  ///
  /// \param argc
  /// \param argv
  /// \param name
  SensorBase(int argc, char* argv[], const std::string& name);
  SensorBase(const SensorBase& rhs);

  ~SensorBase() override;

  /// @brief Set the measurement timeout for the sensor.
  ///
  /// The sensor's status should be set to a degraded condition if
  /// the timeout duration is exceeded between "successful"
  /// measurements.
  ///
  /// A timeout < 0 disables this behavior. (default)
  /// \param timeout
  void setTimeout(ros::Duration timeout) noexcept;

  /// @brief Get the measurement timeout duration.
  ///
  /// \return
  ros::Duration timeout() const noexcept;

  /// @brief Send a command over the a connection
  ///
  /// \param command
  /// \param connection    name of a asio connection
  virtual void sendCommand(std::string command, std::string connection);

  /// @brief Send a command over the instrument connection
  ///
  /// Ensure that the command contains the provided suffix at the end.
  ///
  /// This method can be used to make sure the correct line terminators are
  /// automatically appended to commands.
  ///
  /// \param command
  /// \param connection    name of a asio connection
  /// \param suffix
  virtual void sendCommand(std::string command, std::string connection, std::string suffix);

protected:
  /// @brief Protected constructor for subclasses using their own subclassed pimpl.
  explicit SensorBase(std::unique_ptr<Impl> impl);
  explicit SensorBase(std::unique_ptr<Impl> impl, int argc, char* argv[], const std::string& name);

  /// @brief setup parameters
  ///
  /// The default override checks for the following additional parameters:
  ///
  ///  - ~serial_number  (defaults to 0)
  ///  - ~uuid  (defaults to the nil UUID)
  ///
  ///  The generated uuid will be used if ~uuid is not present.  If ~uuid IS present,
  ///  then it is checked against the generated value.
  ///
  /// \param base
  void setupParameters() override;

  /// @brief Setup asio connections.
  ///
  /// The default behavior adds a single connection named "instrument" that is
  /// connected to SensorBase::parseReceivedBytes
  void setupConnections() override;

  /// @brief Setup services
  ///
  /// The default behavior adds a single services named StringCommand that is
  /// connected to SensorBase::sendCommand
  void setupServices() override;

  /// @brief Handle bytes received from the I/O source.
  ///
  /// Overload this method to parse raw bytes into data messages.
  ///
  /// \param bytes
  virtual void parseReceivedBytes(const ds_core_msgs::RawData& bytes) {}

 private:
  /// @brief Access the underlying pimpl pointer.
  auto d_func() noexcept -> Impl*;

  /// @brief Access the underlying pimpl pointer.
  auto d_func() const noexcept -> Impl const*;

  std::shared_ptr<Impl> impl_;
};
}

#endif  // DS_SENSOR_SENSORBASE_H
