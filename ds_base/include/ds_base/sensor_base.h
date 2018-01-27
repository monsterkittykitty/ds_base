#ifndef DS_SENSOR_SENSORBASE_H
#define DS_SENSOR_SENSORBASE_H

#include "ds_base/ds_process.h"

#include <boost/uuid/uuid.hpp>
#include <memory>

namespace ds_base
{
struct SensorBasePrivate;

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
  DS_DECLARE_PRIVATE(SensorBase)

public:
  using ConnectionMap = std::unordered_map<std::string, boost::shared_ptr<ds_asio::DsConnection>>;
  using TimestampMap = std::unordered_map<std::string, ros::Time>;

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
  ~SensorBase() override;

  DS_DISABLE_COPY(SensorBase)

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

  /// @brief Get access to an instrument connection
  ///
  /// Returns `nullptr` if no connection by that name exists.
  /// \param name
  /// \return
  ds_asio::DsConnection* connection(const std::string& name);

  /// @brief Get access to the full connection map.
  ///
  /// \return
  ConnectionMap& connections();

  /// @brief Update the timestamp for the provided string with the current time.
  ///
  /// \param name
  void updateTimestamp(std::string name);

  /// @brief Update the timestamp for the provided label with the provided time.
  ///
  /// \param name
  void updateTimestamp(std::string name, ros::Time time);

  /// @brief Get the last timestamp for the provided label.
  ///
  /// An invalid ros::Time is returned if the label does not exist.
  ///
  /// \param name
  /// \return
  ros::Time lastTimestamp(const std::string& name) const noexcept;

  /// @brief Get all the recorded timestamps so far
  ///
  /// The returned reference is constant.
  ///
  /// \return
  const TimestampMap& lastTimestamps() const noexcept;

protected:
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
  virtual void parseReceivedBytes(const ds_core_msgs::RawData& bytes)
  {
  }

  /// @brief Update a status message based on the last recorded timestamps.
  ///
  /// Called by the status check timer callback.
  ///
  /// Default logic:
  ///
  /// - if messageTimeout() is set to -1, no further checks are performed.
  /// - if lastTimestamps() is empty, the status is set to STATUS_ERROR
  /// - if any timestamp in lastTimestamps() is > messageTimeout, the
  ///   status is set to STATUS_WARN
  ///
  /// \param status
  virtual void checkMessageTimestamps(ds_core_msgs::Status& status);

  /// @brief  Default sensor status check behavior.
  ///
  /// The default status check performs the following logic:
  ///
  ///  - starts out good
  ///  - calls SensorBase::checkMessageTimestamps, potentially modifying
  ///    the original status.
  ///  - publishes the message.
  ///
  /// \param event
  void checkProcessStatus(const ros::TimerEvent& event) override;

private:
  std::unique_ptr<SensorBasePrivate> d_ptr_;
};
}

#endif  // DS_SENSOR_SENSORBASE_H
