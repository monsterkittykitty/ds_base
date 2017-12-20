#include "ds_base/ds_process.h"


DsProcess::DsProcess()
  : myAsio(std::unique_ptr<DsAsio>(new DsAsio()))
{
}

DsProcess::DsProcess(int argc, char** argv, const std::string &name)
  : myAsio(std::unique_ptr<DsAsio>(new DsAsio(argc, argv, name)))
  , nh(std::unique_ptr<ros::DsNodeHandle>(new ros::DsNodeHandle(&myAsio->io_service)))
{
}

DsProcess::~DsProcess() = default;


ros::DsNodeHandle* DsProcess::getNh() {

  if(!nh) {
    nh.reset(new ros::DsNodeHandle(&(myAsio->io_service)));
  }

  return nh.get();
}
