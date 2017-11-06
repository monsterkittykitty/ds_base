#include <ros/ros.h>
#include <boost/asio.hpp>
#include <ds_asio.h>

class DsProcess
{
public:
  DsProcess();
  ~DsProcess();

protected:
  DsAsio* myAsio;
  
private:
};
