//
// Created by ivaughn on 1/19/18.
//

#ifndef PROJECT_DS_MOCK_CONNECTION_H
#define PROJECT_DS_MOCK_CONNECTION_H

#include "ds_asio/ds_connection.h"
#include "ds_core_msgs/RawData.h"
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/function.hpp>
#include <ros/ros.h>
#include <ctime>
#include <iostream>
#include <string>
#include <deque>

namespace ds_asio
{

    /// \brief This is a mock class intended to let test code set up
class DsMockConnection : public DsConnection
{
public:
  DsMockConnection();
    virtual ~DsMockConnection();


    // public interface
  virtual void receive(void);
  virtual void send(boost::shared_ptr<std::string> message);

    // Set yourself a custom callback
  const boost::function<void(ds_core_msgs::RawData)>& getCallback() const;
  void setCallback(const boost::function<void(ds_core_msgs::RawData)>& _cb);

    /// \brief run the test.  Will return when the test is complete.
    void run();


    /// \brief Get the vector of stuff written (used AFTER) the test
    /// has been run
    const std::deque<ds_core_msgs::RawData>& Written() const;

    /// \brief Get the vector of stuff to Read (used BEFORE the test!)
    const std::deque<ds_core_msgs::RawData>& ToRead() const;
    std::deque<ds_core_msgs::RawData>& ToRead();

protected:
    boost::asio::io_service io_service_;
    boost::function<void(ds_core_msgs::RawData)> callback_;

    /// \brief A vector of strings written by the thing using this to test
    std::deque<ds_core_msgs::RawData> written;

    /// \brief A vector of strings to be ready by the thing under test
    std::deque<ds_core_msgs::RawData> toRead;

    void sendNextMessage();
};

}

#endif //PROJECT_DS_MOCK_CONNECTION_H
