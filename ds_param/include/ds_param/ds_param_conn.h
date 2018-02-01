//
// Created by ivaughn on 1/30/18.
//

#ifndef PROJECT_DS_PARAM_CONN_H
#define PROJECT_DS_PARAM_CONN_H

#include <ros/ros.h>

#include <memory>
#include <string>
#include "ds_param.h"

namespace ds_param {

// forward declaration
class ParamConnectionPrivate;

class ParamConnection : public std::enable_shared_from_this<ParamConnection> {
 private:
  ParamConnection(ros::NodeHandle& handle);

 public:

  /// \typedef A shared_ptr typedef for this class
  typedef std::shared_ptr<ParamConnection> Ptr;

  /// \typedef A handy shared_ptr-to-a-const-copy typedef for this class
  typedef std::shared_ptr<const ParamConnection> ConstPtr;

  /// \brief Create a Parameter Connection instance.  Requires a global
  /// node handle.
  static std::shared_ptr<ParamConnection> create(ros::NodeHandle& _h);

  /// \brief Destructor
  virtual ~ParamConnection();

  /// \brief Connect to an updating parameter on the parameter server
  ///
  /// Connect to the parameter server and create a live-updating parameter
  /// for the given parameter name.
  ///
  /// \tparam T The type of the parameter.  MUST be one of our standard parameter
  /// typedefs: BoolParam, IntParam, FloatParam, DoubleParam, or StringParam
  /// or your code won't link!
  /// \param param_name The name of the parameter.  This will be resolved using the current
  /// node namespace go get the full name.
  /// \param advertise Indicates whether the parameter connection should advertise
  /// this parameter as being associated with this node.  Parameters this process
  /// reads should be advertised; parameters this process sets should not be advertised
  /// \return A shared_ptr to the requested type.  If the parameter does not exist or
  /// cannot be cast to the requested type then an empty pointer (NULL) is returned
  template <typename T>
  typename T::Ptr connect(const std::string& param_name, bool advertise=true);

  /// \brief Get this connection's unique ID
  ///
  /// Every connection generates a unique name to avoid updating in response to its
  /// own updates.  You really shouldn't need or use this, but it might be handy
  /// for debugging or console messages or something
  const std::string& connName() const;

 protected:
  std::shared_ptr<ParamConnectionPrivate> impl;
};
}
#endif //PROJECT_DS_PARAM_CONN_H
