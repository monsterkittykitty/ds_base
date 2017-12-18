#ifndef DS_PROCESS_H
#define DS_PROCESS_H

#include "ds_base/ds_asio.h"
#include <ros/ros.h>
#include <boost/asio.hpp>
#include "ds_base/ds_nodehandle.h"

class DsProcess
{
public:
  DsProcess(int argc, char** argv, const std::string &name);
  virtual ~DsProcess();

  DsAsio* myAsio;

protected:
  ros::DsNodeHandle              *nh;
  
private:
};

#endif
