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
    ///  Whether a given response is "acceptable" is determined by the provided "InputCheck"
    ///  function.  Once a command is sent, the state machine will block
    ///  until either the InputCheck returns >= 0 or the timeout is reached.
    ///
    class IoCommand {
    public:
        typedef std::shared_ptr<IoCommand> Ptr;
        typedef std::shared_ptr<const IoCommand> ConstPtr;

        /// @brief Shorthand to create a standard command/timeout pair
        ///
        /// \param cmdstr The command string to send
        /// \param timeout_sec  The number of seconds of timeout time for the command (NO other timeouts are applied!)
        /// \param _allow_preempt Allow a non-regular-command after this one (ON by default)
        IoCommand(const std::string &cmdstr, double timeout_sec, bool _allow_preempt=true);

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
        /// key example is setting an address on a bus.  The "Allow Preempt" option
        /// prevents the preemptive commands from breaking up that chain
        bool getAllowPreempt() const;

        /// @brief Set whether a preempt command is allowed to follow this command
        ///
        /// Some commands require multiple inputs with return checking in between.  One
        /// key example is setting an address on a bus.  The "Allow Preempt" option
        /// prevents the preemptive commands from breaking up that chain
        void setAllowPreempt(bool _a);

        /// @brief Get the state machine's ID for this command.  Used to override existing
        /// entries
        uint64_t getId() const;

        /// @brief Set the state machine's ID for this command.  Set by the state machine
        void setId(uint64_t);

        /// @brief Typedef for all input check functions
        /// Define the function to accept input (or not)
        /// The check function determines if the input should be accept, and how many
        /// characters from the I/O buffer should be consumed.  Return values are:
        /// < 0 (generally -1): Input not accepted
        /// 0: Accept input, but don't consume any characters
        /// n > 0: Accept input and consume n characters
        typedef std::function<std::pair<int, std::string>(const std::string&)> CheckFunction;

        /// @brief Get the current check function for this command
        const IoCommand::CheckFunction &getInputCheck();

        /// @brief Set the check function used to determine if a specific buffer is
        /// an accepted input or not
        void setInputCheck(const IoCommand::CheckFunction &_f);

        /// @brief Check if a given input is valid using the internal check function.
        std::pair<int, std::string> checkInput(const std::string& inp);

        /// \brief A check function that always rejects the input
        static CheckFunction alwaysReject();

        /// \brief A check function that always accepts any input
        static CheckFunction alwaysAccept();

        /// \brief A check function that always runs the supplied regex on the input and
        /// returns the result.
        ///
        /// \param regex The regex to match against using boost::regex_search.
        /// \return An std::pair with the index of the last matched position and the matched string
        static CheckFunction checkRegex(const std::string &regex);

        /// \brief A check function that always runs the supplied regex on the input and
        /// returns the result.
        ///
        /// \param regex The regex to match against using boost::regex_search.
        /// \return An std::pair with the index of the last matched position and the matched string
        static CheckFunction checkRegex(const boost::regex &regex);

    protected:
        uint64_t id;
        std::string cmd;
        bool emitOnMatch;
        bool timeoutWarn;
        bool timeoutLog;   // log
        bool allowPreempt; // allow the next command to be a preempt command
        // This is useful in some cases where a command
        // sequence should happen atomically, e.g.,
        // we send an address command and then a
        // query or something
        CheckFunction checker;
        ros::Duration delayBefore;
        ros::Duration delayAfter;
        ros::Duration timeout;
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
