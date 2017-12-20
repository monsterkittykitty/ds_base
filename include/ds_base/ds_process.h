#ifndef DS_PROCESS_H
#define DS_PROCESS_H

#include "ds_base/ds_asio.h"
#include <ros/ros.h>
#include <boost/asio.hpp>
#include "ds_base/ds_nodehandle.h"

class DsProcess
{

public:
  DsProcess();
  DsProcess(int argc, char** argv, const std::string &name);

  virtual ~DsProcess();

  ros::DsNodeHandle* getNh();

  void run();

protected:
  std::unique_ptr<ros::DsNodeHandle> nh;
  std::unique_ptr<DsAsio> myAsio;

};

#endif
