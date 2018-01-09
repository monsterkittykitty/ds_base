#ifndef DS_CALLBACKQUEUE_H
#define DS_CALLBACKQUEUE_H

#include "ros/callback_queue.h"
#include "boost/asio.hpp"
#include "boost/bind.hpp"

namespace ds_asio
{
  class ROSCPP_DECL DsCallbackQueue : public ros::CallbackQueue
  {
  public:
    /// @brief Construct a new DsCallbackQueue
    ///
    /// @param[in] io_service A pointer to the boost::io_service object to which this DsCallbackQueue object will post events to
    DsCallbackQueue(boost::asio::io_service *io_service);

    /// @brief Add a callback in the callback queue. This method overrides the CallbackQueue base class virtual method
    ///
    /// @param[in] callback A reference to the callback that is being inserted in the queue
    /// @param[in] removal_id An optional parameter used if ros wants to remove the callack from the queue
    virtual void addCallback(const ros::CallbackInterfacePtr& callback, uint64_t removal_id = 0);

  private:
    boost::asio::io_service *myIoService;
  };
}

#endif
