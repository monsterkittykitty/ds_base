#include "ds_asio/ds_callbackqueue.h"

namespace ds_asio
{
DsCallbackQueue::DsCallbackQueue(boost::asio::io_service* io_service) : ros::CallbackQueue(), myIoService(io_service)
{
}

void DsCallbackQueue::addCallback(const ros::CallbackInterfacePtr& callback, uint64_t removal_id)
{
  CallbackInfo info;
  info.callback = callback;
  info.removal_id = removal_id;

  {
    boost::mutex::scoped_lock lock(id_info_mutex_);

    M_IDInfo::iterator it = id_info_.find(removal_id);
    if (it == id_info_.end())
    {
      IDInfoPtr id_info(boost::make_shared<IDInfo>());
      id_info->id = removal_id;
      id_info_.insert(std::make_pair(removal_id, id_info));
    }
  }

  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!enabled_)
    {
      return;
    }

    callbacks_.push_back(info);
  }

  condition_.notify_one();

  // SS - add posting of event to boost::asio io_service here
  myIoService->post(boost::bind(&ds_asio::DsCallbackQueue::callAvailable, this));
}
}
