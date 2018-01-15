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

namespace ds_base {

struct DsBus::Impl : public ds_base::DsProcess::Impl {
    Impl(): ds_base::DsProcess::Impl(),
            message_timeout_ (ros::Duration(-1)),
            uuid_(boost::uuids::nil_uuid()) {
        // do nothing (else)
    }

    ~Impl() override = default;

    void setupConnections(ds_base::DsProcess* base) override {
        ds_base::DsProcess::Impl::setupConnections(base);

        iosm = base->addIoSM("statemachine", "instrument", boost::bind(&DsBus::Impl::_data_recv, this, _1));
    }

    void setupPublishers(ds_base::DsProcess* base) override {
        // call the superclass
        ds_base::DsProcess::Impl::setupPublishers(base);

        // add our additional secret sauce
        bus_pub_ = base->advertise<ds_core_msgs::RawData>(ros::this_node::getName() + "/bus", 10, false);

        // the whole queue is controlled by a service
        cmd_serv_ = base->advertiseService<ds_core_msgs::IoSMcommand::Request,
                                           ds_core_msgs::IoSMcommand::Response>(ros::this_node::getName() + "/cmd",
                                                                      boost::bind(&DsBus::Impl::_service_req, this, _1, _2));
    }

    void checkProcessStatus(const ros::TimerEvent &event) override {
        const auto now = ros::Time::now();

        auto status = ds_core_msgs::Status();
        status.descriptive_name = descriptive_node_name_;

        status.ds_header.io_time = now;
        std::copy(std::begin(uuid_.data), std::end(uuid_.data), std::begin(status.ds_header.source_uuid));

        if (message_timeout_ < ros::Duration(0)
            || now - last_message_timestamp_ > message_timeout_) {

            status.status = ds_core_msgs::Status::STATUS_GOOD;

        } else {
            status.status = ds_core_msgs::Status::STATUS_ERROR;
        }

        status_publisher_.publish(status);
    }

    void setupParameters(ds_base::DsProcess *base) override {
        ds_base::DsProcess::Impl::setupParameters(base);

        message_timeout_ = ros::Duration(ros::param::param<double>("~message_timeout", 5));

        auto generated_uuid = ds_base::generateUuid("bus_node_" + descriptive_node_name_);

    }

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

} // namespace ds_base

#endif //PROJECT_DS_IOSM_PROCESS_PRIVATE_H
