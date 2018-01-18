//
// Created by ivaughn on 1/10/18.
//
// I/O State Machine
//
// The IO State Machine is described more fully in IO_SM.md//
//

#ifndef PROJECT_DS_IOSM_H
#define PROJECT_DS_IOSM_H

#include <string>
#include <memory>
#include <mutex>
#include <list>
#include <utility>

#include <functional>

#include <boost/regex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/noncopyable.hpp>

#include <ds_asio/ds_connection.h>
#include "ds_core_msgs/RawData.h"
#include "ds_core_msgs/IoCommand.h"
#include <ros/duration.h>


namespace ds_asio {

    ///  @brief The IoCommand class is a single entry in the command table
    ///
    ///  IoCommands specify timeouts, delays, and acceptance criteria.  Each basic
    ///  command implements the following steps:
    ///
    ///  1. [optional] delay before sending a command
    ///  2. Send a provided string
    ///  3. Wait for an acceptable response.  More on this later
    ///  4. Create a new event based on that response
    ///  5. [optional] delay before moving on to the next command
    ///
    class IoCommand {
    public:
        typedef std::shared_ptr<IoCommand> Ptr;
        typedef std::shared_ptr<const IoCommand> ConstPtr;

        typedef boost::function<void(ds_core_msgs::RawData)> RecvFunc;

        /// @brief Shorthand to create a standard command/timeout pair
        ///
        /// \param cmdstr The command string to send
        /// \param timeout_sec  The number of seconds of timeout time for the command (NO other timeouts are applied!)
        /// \param _force_next Allow a non-regular-command after this one (OFF by default)
        /// \param callback A callback fired when the response to this particular command is received
        IoCommand(const std::string &cmdstr, double timeout_sec, bool _force_next=false,
                  RecvFunc callback=RecvFunc());

        /// @brief Shorthand to create a static wait
        ///
        /// The primary use of this is to add a delay to slow down interogation loops
        ///
        /// \param timeout_sec Number of seconds to wait
        IoCommand(double timeout_sec);

        /// @brief Fill in a command string using a IoCommand ROS message
        IoCommand(const ds_core_msgs::IoCommand& _cmd);

        /// @brief Set the command string to send
        void setCommand(const std::string &_cmd);

        /// @brief Get the command string that will be sent
        const std::string &getCommand() const;

        /// @brief Set the delay before the command string is sent
        void setDelayBefore(const ros::Duration &_d);

        /// @brief Get the delay before the command string is sent
        const ros::Duration &getDelayBefore() const;

        /// @brief Set the delay after a valid reply is recieved, but
        /// before the next command is started
        void setDelayAfter(const ros::Duration &_d);

        /// @brief Get the delay after a valid reply is recieved, but
        /// before the next command is started
        const ros::Duration &getDelayAfter() const;

        /// @brief Set the maximum time to wait for a valid reply
        void setTimeout(const ros::Duration &_t);

        /// @brief Get the maximum time to wait for a valid reply
        const ros::Duration &getTimeout() const;

        /// @brief Get the flag to determine whether to flush the Io State Machine's buffer
        /// at the start of this command
        bool flushInput() const;

        /// @brief Set the flag to determine whether to flush the Io State Machine's buffer
        /// at the start of this command
        void setFlush(bool _f);

        /// @brief Check whether this command will generate a callback on
        /// accepting an response
        bool emit() const;

        /// @brief Set whether this command will generate a callback on
        /// accepting an response
        void setEmit(bool _e);

        /// @brief Check whether this command will log a warning to the ROS console on
        /// timeout
        bool warnOnTimeout() const;

        /// @brief Set whether this command will log a warning to the ROS console on
        /// timeout
        void setWarnOnTimeout(bool _w);

        /// @brief Check whether this command will log debug info to the ROS console
        /// timeout
        bool logOnTimeout() const;

        /// @brief Set whether this command will log debug info to the ROS console
        /// timeout
        void setLogOnTimeout(bool _l);

        /// @brief Check whether a preempt command is allowed to follow this command
        ///
        /// Some commands require multiple inputs with return checking in between.  One
        /// key example is setting an address on a bus.  The "Force Next" option
        /// forces the next command to come from the same queue as this one.
        bool getForceNext() const;

        /// @brief Set whether a preempt command is allowed to follow this command
        ///
        /// Some commands require multiple inputs with return checking in between.  One
        /// key example is setting an address on a bus.  The "Force Next" option
        /// forces the next command to come from the same queue as this one.
        void setForceNext(bool _fn);

        /// @brief Get the state machine's ID for this command.  Used to override existing
        /// entries
        uint64_t getId() const;

        /// @brief Set the state machine's ID for this command.  Set by the state machine
        void setId(uint64_t);

        /// @brief Check to see if this command has a custom callback
        bool hasCallback() const;

        /// @brief Get the callback function for this command
        ///
        /// \return The function to call on receipt of data for this command
        const RecvFunc& getCallback() const;


    protected:
        uint64_t id;
        std::string cmd;
        bool emitOnMatch;
        bool timeoutWarn;
        bool timeoutLog;   // log
        bool forceNext; // force the next command to be from the same queue
        // This is useful in some cases where a command
        // sequence should happen atomically, e.g.,
        // we send an address command and then a
        // query or something
        ros::Duration delayBefore;
        ros::Duration delayAfter;
        ros::Duration timeout;

        RecvFunc callback;
    };

    // forward declaration
    namespace iosm_inner {
        class _IoSM_impl;
    };

    /// @brief The public IoState Machine class.
    ///
    /// This class has a fairly gnarly implementation, but the details are hidden behind
    /// a pointer to the implementation.  In particular, note that
    /// the IoSM class carries a shared_ptr to the actual data-- therefore, its
    /// pretty easy to move around, etc.
    class IoSM {
    public:
        IoSM(boost::asio::io_service& io_service,
             std::string name,
             boost::function<void(ds_core_msgs::RawData)> callback,
             ros::NodeHandle* myNh);

        /// @brief Destructor.
        ///
        /// If there are still pending operations this may not immediately stop
        /// them.
        virtual ~IoSM();

        /// @brief Set the I/O connection.
        ///
        /// Necessary because we have to have our IoSM to setup the callback for the
        /// I/O Connection before we create the thing.
        void setConnection(const boost::shared_ptr<DsConnection>& conn);

        /// @brief Get the I/O Connection
        const boost::shared_ptr<DsConnection>& getConnection() const;

        /// @brief Add a regular command to the regular command queue
        ///
        /// \param cmd The command to run
        /// \return The unique ID for this command
        uint64_t addRegularCommand(const IoCommand& cmd);

        /// @brief Delete a regular command from the queue
        ///
        /// \param id The ID of the command to delete (returned by addRegularCommand)
        void deleteRegularCommand(const uint64_t id);

        /// @brief Overwrite an existing command in the regular queue
        ///
        /// \param id The ID of the command to overwrite
        /// \param cmd The new command to execute instead
        void overwriteRegularCommand(const uint64_t id, const IoCommand& cmd);

        /// @brief Add a command to the preempt queue.  This command will be run
        /// as soon as the bus is available
        ///
        /// \param cmd The new command to run ASAP.
        void addPreemptCommand(const IoCommand& cmd);

        /// @brief A callback fired when data is provided by the connection.
        ///
        /// Do not use unless you REALLY know what you're doing
        ///
        /// \param raw The raw data coming in
        void _connCallback(const ds_core_msgs::RawData& raw);

    protected:
        /// \brief Shared pointer to our implementation
        std::shared_ptr<ds_asio::iosm_inner::_IoSM_impl> impl;
    };

} // namespace ds_asio


#endif //PROJECT_DS_IOSM_H
