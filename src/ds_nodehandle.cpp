#include "ds_base/ds_nodehandle.h"

namespace ros
{
  DsNodeHandle::DsNodeHandle(boost::asio::io_service *io_service, const std::string &ns, const M_string &remappings):
    NodeHandle(ns, remappings),
    myIoService(io_service)
  {
    queue = new ros::DsCallbackQueue(io_service);
    this->setCallbackQueue((ros::CallbackQueue*) queue);
  }
}
