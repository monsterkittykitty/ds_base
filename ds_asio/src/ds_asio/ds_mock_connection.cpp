//
// Created by ivaughn on 1/19/18.
//

#include <ds_asio/ds_mock_connection.h>

namespace ds_asio
{
   DsMockConnection:: DsMockConnection(boost::asio::io_service& io) { /* do nothing */ }

    DsMockConnection::~DsMockConnection() { /* do nothing */ }

    void DsMockConnection::receive(void) {
        // do nothing
        // TODO: Make an option to send unsolicited stuff
    }
    void DsMockConnection::send(boost::shared_ptr<std::string> message) {
        written.push_back(*message);
        // send the next thing

        sendNextMessage();
    }

    void DsMockConnection::run() {
        // TODO: We really ought to set up a maximum run timer here
        io_service_.run();
    }

    const boost::function<void(ds_core_msgs::RawData)>& DsMockConnection::getCallback() const {
        return callback_;
    }
    void DsMockConnection::setCallback(const boost::function<void(ds_core_msgs::RawData)>& _cb) {
        callback_ = _cb;
    }

    const std::deque<ds_core_msgs::RawData>&DsMockConnection:: Written() const {
        return written;
    }

    const std::deque<ds_core_msgs::RawData>& DsMockConnection::ToRead() const {
        return toRead;
    }
    std::deque<ds_core_msgs::RawData>& DsMockConnection::ToRead() {
        return toRead;
    }

    void DsMockConnection::sendNextMessage() {
        // If the queue is empty, then stop the I/O service to make run return.
        if (toRead.empty()) {
            io_service_.stop();
            return;
        }

        // IoSM can't handle an infinite chain of callbacks because of its locking
        // mechanisms.  This will generate a totally new event
        io_service_.post(boost::bind(callback_, toRead.front()));
    }
}