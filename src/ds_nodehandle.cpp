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

  DsNodeHandle::DsNodeHandle(const DsNodeHandle &rhs):
    NodeHandle(rhs)
  {
    queue = rhs.queue;
    myIoService = rhs.myIoService;
  }

  DsNodeHandle::DsNodeHandle(const DsNodeHandle &parent, const std::string &ns):
    NodeHandle(parent, ns)
  {
    queue = parent.queue;
    myIoService = parent.myIoService;
  }

  DsNodeHandle::DsNodeHandle(const DsNodeHandle &parent, const std::string &ns, const M_string &remappings):
    NodeHandle(parent, ns, remappings)
  {
    queue = parent.queue;
    myIoService = parent.myIoService;
  } 
}
