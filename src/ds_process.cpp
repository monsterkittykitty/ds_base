#include "ds_base/ds_process.h"

DsProcess::DsProcess(int argc, char** argv, const std::string &name)
{
  
  myAsio = new DsAsio(argc, argv, name);

  nh = new ros::DsNodeHandle(&myAsio->io_service);
  
}

DsProcess::~DsProcess()
{
  ;
}
