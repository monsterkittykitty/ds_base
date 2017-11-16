#ifndef DS_PROCESS_H
#define DS_PROCESS_H

#include <ros/ros.h>
#include <boost/asio.hpp>
#include <ds_asio.h>

class DsProcess
{
public:
  DsProcess(int argc, char** argv, const std::string &name);
  ~DsProcess();

protected:
  DsAsio* myAsio;
  
private:
};

#endif
