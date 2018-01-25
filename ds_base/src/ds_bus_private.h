//
// Created by ivaughn on 1/15/18.
//

#ifndef PROJECT_DS_IOSM_PROCESS_PRIVATE_H
#define PROJECT_DS_IOSM_PROCESS_PRIVATE_H

#include "ds_asio/ds_iosm.h"
#include "ds_base/ds_bus.h"
#include "ds_base/ds_process_private.h"
#include <ds_base/util.h>

#include <ds_core_msgs/Status.h>
#include <ds_core_msgs/RawData.h>
#include <ds_core_msgs/IoSMcommand.h>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/nil_generator.hpp>

<<<<<<< HEAD
namespace ds_base {

struct DsBus::Impl : public ds_base::DsProcess::Impl {
    Impl(): ds_base::DsProcess::Impl(),
            message_timeout_ (ros::Duration(-1)),
            uuid_(boost::uuids::nil_uuid()) {
        // do nothing (else)
    }

    ~Impl() override = default;


    void _data_recv(const ds_core_msgs::RawData& bytes) {
        last_message_timestamp_ = bytes.header.stamp;
        bus_pub_.publish(bytes);
    }

    bool _service_req(const ds_core_msgs::IoSMcommand::Request& req,
                      ds_core_msgs::IoSMcommand::Response& resp) {
        for (auto iter = req.commands.begin(); iter != req.commands.end(); iter++) {

            ds_asio::IoCommand cmd(*iter);

            switch (req.iosm_command) {
                case ds_core_msgs::IoSMcommand::Request::IOSM_ADD_REGULAR:
                    resp.retval.push_back(iosm->addRegularCommand(cmd));
                    break;

                case ds_core_msgs::IoSMcommand::Request::IOSM_UPDATE_REGULAR:
                    iosm->overwriteRegularCommand(cmd.getId(), cmd);
                    resp.retval.push_back(cmd.getId());
                    break;

                case ds_core_msgs::IoSMcommand::Request::IOSM_REMOVE_REGULAR:
                    iosm->deleteRegularCommand(cmd.getId());
                    resp.retval.push_back(cmd.getId());
                    break;

                case ds_core_msgs::IoSMcommand::Request::IOSM_ADD_PREEMPT:
                    iosm->addPreemptCommand(cmd);
                    resp.retval.push_back(1); // preempt commands don't have an ID
                    break;

                case ds_core_msgs::IoSMcommand::Request::IOSM_ADD_SHUTDOWN:
                case ds_core_msgs::IoSMcommand::Request::IOSM_UPDATE_SHUTDOWN:
                case ds_core_msgs::IoSMcommand::Request::IOSM_REMOVE_SHUTDOWN:
                    // TODO: Implement these
                    ROS_ERROR_STREAM("NOT IMPLEMENTED: Could not add shutdown command " << cmd.getCommand());
                    break;
            }
        }

        return true;
=======
namespace ds_base
{
struct DsBus::Impl : public ds_base::DsProcess::Impl
{
  Impl() : ds_base::DsProcess::Impl(), message_timeout_(ros::Duration(-1)), uuid_(boost::uuids::nil_uuid())
  {
    // do nothing (else)
  }

  ~Impl() override = default;

  void _data_recv(const ds_core_msgs::RawData& bytes)
  {
    last_message_timestamp_ = bytes.header.stamp;
    bus_pub_.publish(bytes);
  }

  bool _service_req(const ds_core_msgs::IoSMcommand::Request& req, ds_core_msgs::IoSMcommand::Response& resp)
  {
    for (auto iter = req.commands.begin(); iter != req.commands.end(); iter++)
    {
      ds_asio::IoCommand cmd(*iter);

      switch (req.iosm_command)
      {
        case ds_core_msgs::IoSMcommand::Request::IOSM_ADD_REGULAR:
          resp.retval.push_back(iosm->addRegularCommand(cmd));
          break;

        case ds_core_msgs::IoSMcommand::Request::IOSM_UPDATE_REGULAR:
          iosm->overwriteRegularCommand(cmd.getId(), cmd);
          resp.retval.push_back(cmd.getId());
          break;

        case ds_core_msgs::IoSMcommand::Request::IOSM_REMOVE_REGULAR:
          iosm->deleteRegularCommand(cmd.getId());
          resp.retval.push_back(cmd.getId());
          break;

        case ds_core_msgs::IoSMcommand::Request::IOSM_ADD_PREEMPT:
          iosm->addPreemptCommand(cmd);
          resp.retval.push_back(1);  // preempt commands don't have an ID
          break;

        case ds_core_msgs::IoSMcommand::Request::IOSM_ADD_SHUTDOWN:
        case ds_core_msgs::IoSMcommand::Request::IOSM_UPDATE_SHUTDOWN:
        case ds_core_msgs::IoSMcommand::Request::IOSM_REMOVE_SHUTDOWN:
          // TODO: Implement these
          ROS_ERROR_STREAM("NOT IMPLEMENTED: Could not add shutdown command " << cmd.getCommand());
          break;
      }
>>>>>>> 537a06e6a8d8feb290041c632083101483aae1f6
    }
  }

  /// @brief Publisher for all incoming bus traffic
  ros::Publisher bus_pub_;

  /// @brief Service handler for command & control
  ros::ServiceServer cmd_serv_;

  /// @brief Our I/O state machine
  boost::shared_ptr<ds_asio::IoSM> iosm;

  /// @brief The bus UUID
  boost::uuids::uuid uuid_;

  /// @brief Timestamp of last data on the bus
  ros::Time last_message_timestamp_;

  /// @brief Time between valid messages to consider bad
  ros::Duration message_timeout_;
};

}  // namespace ds_base

#endif  // PROJECT_DS_IOSM_PROCESS_PRIVATE_H
