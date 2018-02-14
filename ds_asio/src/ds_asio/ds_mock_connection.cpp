//
// Created by ivaughn on 1/19/18.
//

#include <ds_asio/ds_mock_connection.h>

namespace ds_asio
{
DsMockConnection::DsMockConnection(boost::asio::io_service& io_service) : DsConnection(io_service)
{
  initializing = true;
  writeDuringStartup = false;
}

DsMockConnection::~DsMockConnection()
{ /* do nothing */
}

void DsMockConnection::receive(void)
{
  // do nothing
  // TODO: Make an option to send unsolicited stuff
}

boost::asio::io_service& DsMockConnection::getIoService()
{
  return io_service_;
}

void DsMockConnection::send(boost::shared_ptr<std::string> message)
{
  written.push_back(*message);
  // send the next thing

  sendNextMessage();
}

void DsMockConnection::run()
{
  // TODO: We really ought to set up a maximum run timer here
  initializing = false;
  if (writeDuringStartup)
  {
    sendNextMessage();
  }
  if (! toRead.empty()) {
    io_service_.reset();
  }

  io_service_.run();
}

const std::deque<std::string>& DsMockConnection::Written() const
{
  return written;
}

const std::deque<ds_core_msgs::RawData>& DsMockConnection::ToRead() const
{
  return toRead;
}
std::deque<ds_core_msgs::RawData>& DsMockConnection::ToRead()
{
  return toRead;
}

void DsMockConnection::setWriteDuringStartup() {
  writeDuringStartup = true;
}

void DsMockConnection::sendNextMessage()
{
  // If the queue is empty, then stop the I/O service to make run return.
  if (initializing)
  {
    // if we're initializing, then set the flag to actually send when setup is complete
    writeDuringStartup = true;
    return;
  }
  if (toRead.empty())
  {
    io_service_.stop();
    return;
  }

  // if the message is empty, DO NOT send a message.  This is used to trigger
  // read timeouts and test for good behavior there.
  if (toRead.front().data.size() == 0)
  {
    // this may deadlock things in certain, poorly-chosen situations
    // or maybe not-- I'm not sure, which is why its so darn dangerous!
    toRead.pop_front();
    return;
  }
  toRead.front().header.stamp = ros::Time::now();
  toRead.front().ds_header.io_time = toRead.front().header.stamp;

  // IoSM can't handle an infinite chain of callbacks because of its locking
  // mechanisms.  This will generate a totally new event
  io_service_.post(boost::bind(callback_, toRead.front()));
  toRead.pop_front();
}
void DsMockConnection::setup(ros::NodeHandle &nh) {

}
}