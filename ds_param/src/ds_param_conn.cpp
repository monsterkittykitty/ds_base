//
// Created by ivaughn on 1/30/18.
//

#include "ds_param_private.h"
#include "ds_param_conn_private.h"

namespace ds_param {

ParamConnection::ParamConnection(ros::NodeHandle &_h) : impl(
    std::make_shared<ParamConnectionPrivate>(_h)) {
}

ParamConnection::~ParamConnection() = default;

std::shared_ptr<ParamConnection> ParamConnection::create(ros::NodeHandle &handle) {
  return std::shared_ptr<ParamConnection>(new ParamConnection(handle));
}

template<typename T>
typename T::Ptr ParamConnection::connect(const std::string &param_name, bool advertise) {

  std::string full_name = impl->handle.resolveName(param_name);
  typename T::Ptr ret;
  auto iter = impl->params.find(full_name);
  if (iter != impl->params.end()) {
    ROS_INFO_STREAM("Variable named \"" << full_name << "\" (given \""<<param_name
                                        <<"\") already exists!");
    ret = std::dynamic_pointer_cast<T>(iter->second);
    if (! ret) {
      ROS_ERROR_STREAM("Variable named \"" <<full_name <<"\" already exists, but could not be cast to the requested type");
    } else {
      if (advertise) {
        // FORCE us to advertise an existing variable if connection asks us to, whether
        // the existing version does or not
        ret->setAdvertise(true);
        impl->publishDescription();
      }
    }
    return ret;
  }

  if (!impl->handle.hasParam(full_name)) {
    ROS_ERROR_STREAM("Variable named \"" << full_name << "\" does not exist on the parameter server!");
    return ret;
  }

  ret = std::make_shared<T>(impl, full_name, advertise);

  ret->loadFromServer();

  // add to our global list of parameters
  impl->params.insert(std::make_pair(full_name, std::static_pointer_cast<UpdatingParam>(ret)));

  // update our description
  impl->publishDescription();

  return ret;
}

const std::string& ParamConnection::connName() const {
  return impl->conn_name;
};

void ParamConnection::setCallback(const Callback_t& _cb) {
  impl->callback = _cb;
}

// explicit instantiations-- this basically fills in the template during the build so that
// the necessary implementation ends up in hte library
template typename BoolParam::Ptr   ParamConnection::connect<BoolParam  >(const std::string &param_name, bool advertise);
template typename IntParam::Ptr    ParamConnection::connect<IntParam   >(const std::string &param_name, bool advertise);
template typename FloatParam::Ptr  ParamConnection::connect<FloatParam >(const std::string &param_name, bool advertise);
template typename DoubleParam::Ptr ParamConnection::connect<DoubleParam>(const std::string &param_name, bool advertise);
template typename StringParam::Ptr ParamConnection::connect<StringParam>(const std::string &param_name, bool advertise);
template typename EnumParam::Ptr   ParamConnection::connect<EnumParam  >(const std::string &param_name, bool advertise);

}



