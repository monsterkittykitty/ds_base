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
class DsBusDevice : public ds_base::DsProcess
{
protected:
  // Forward declaration of implementation details class
  struct Impl;

public:
  explicit DsBusDevice();
  DsBusDevice(int argc, char* argv[], const std::string& name);

  /// @brief Overall setup function.
  ///
  /// Default implementation adds the setup I/O state machine stuff
  /// in the correct place.
  void setup() override;

protected:
  // protected constructors so we can subclass THIS class
  explicit DsBusDevice(std::unique_ptr<Impl> impl);
  DsBusDevice(std::unique_ptr<Impl> impl, int argc, char* argv[], const std::string& name);

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
  virtual void setupIoSM(){};

  /// @brief Handle bytes received from the bus
  ///
  /// Overload this method to parse raw bytes from the bus
  /// Note that this is called for ALL bus traffic; you have to do your own filtering.
  ///
  /// \param bytes The raw data from the bus
  virtual void parseReceivedBytes(const ds_core_msgs::RawData& bytes){};

private:
  // functions to access our implementation structure
  auto d_func() noexcept -> Impl*;
  auto d_func() const noexcept -> Impl const*;
};

}  // namespace ds_base

<<<<<<< HEAD
        // functions to access our implementation structure
        auto d_func() noexcept -> Impl *;
        auto d_func() const noexcept -> Impl const *;
    };

} // namespace ds_base

#endif //PROJECT_DS_BUS_DEVICE_H
=======
#endif  // PROJECT_DS_BUS_DEVICE_H
>>>>>>> 537a06e6a8d8feb290041c632083101483aae1f6
