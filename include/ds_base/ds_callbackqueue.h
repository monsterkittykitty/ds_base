#ifndef DS_CALLBACKQUEUE_H
#define DS_CALLBACKQUEUE_H

#include "ros/callback_queue.h"
#include "boost/asio.hpp"
#include "boost/bind.hpp"

namespace ros
{
  class ROSCPP_DECL DsCallbackQueue : public CallbackQueue
  {
  public:
    DsCallbackQueue(boost::asio::io_service *io_service);
    virtual void addCallback(const CallbackInterfacePtr& callback, uint64_t removal_id = 0);
    //void registerIoService(boost::asio::io_service *io_service);

  private:
    boost::asio::io_service *myIoService;
  };
}

#endif
