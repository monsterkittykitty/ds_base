#ifndef DS_NODEHANDLE_H
#define DS_NODEHANDLE_H

#include "ds_base/ds_callbackqueue.h"
#include "ros/ros.h"

namespace ros
{
  class ROSCPP_DECL DsNodeHandle : public NodeHandle
  {
  public:
    DsNodeHandle(boost::asio::io_service *io_service, const std::string &ns=std::string(), const M_string &remappings=M_string());

  private:
    boost::asio::io_service *myIoService;
    ros::DsCallbackQueue    *queue;
  };
}
#endif
