#ifndef DS_NODEHANDLE_H
#define DS_NODEHANDLE_H

#include "ds_asio/ds_callbackqueue.h"
#include "ros/ros.h"

namespace ds_asio
{

class ROSCPP_DECL DsNodeHandle : public ros::NodeHandle
{
public:
  DsNodeHandle(boost::asio::io_service *io_service, const std::string &ns=std::string(), const ros::M_string &remappings=ros::M_string());

  DsNodeHandle(const DsNodeHandle &rhs);

  DsNodeHandle(const DsNodeHandle &parent, const std::string &ns);

  DsNodeHandle(const DsNodeHandle &parent, const std::string &ns, const ros::M_string &remappings);

  virtual ~DsNodeHandle();

private:
  boost::asio::io_service *myIoService;
  DsCallbackQueue    *queue;
};

}


#endif
