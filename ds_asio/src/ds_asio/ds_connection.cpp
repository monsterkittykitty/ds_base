#include "ds_asio/ds_connection.h"

namespace ds_asio
{
DsConnection::DsConnection(boost::asio::io_service& _io) : io_service_(_io)
{
  nh_ = nullptr;
}

DsConnection::DsConnection(boost::asio::io_service& _io, std::string name, const ReadCallback& callback,
                           ros::NodeHandle* myNh)
  : io_service_(_io), name_(name), callback_(callback), nh_(myNh)
{
  // do nothing else
}

DsConnection::~DsConnection()
{
  ;
}

void DsConnection::send(const std::string& message)
{
  // asio really needs a shared_ptr to the message; this is just a convienence
  // wrapper to do the copy into a shared_ptr buffer easier on users
  this->send(boost::shared_ptr<std::string>(new std::string(message)));
}

const std::string& DsConnection::getName() const
{
  return name_;
}

boost::asio::io_service& DsConnection::getIoService()
{
  return io_service_;
}

const ReadCallback& DsConnection::getCallback() const
{
  return callback_;
}

void DsConnection::setCallback(const ReadCallback& _cb)
{
  callback_ = _cb;
}
}
