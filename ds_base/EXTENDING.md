# Extending the `ds_base` package

`ds_base::DsProcess` attempts to serve as an "interface" for ROS nodes in the `ds` ecosystem.  It
defines a small number of methods that all nodes must provide (like a traditional interface) and
handles setting up the parameters, topics, etc. that all nodes are expected to provide.

This document explains how to subclass `ds_base::DsProcess` to create a new node that place nicely
with the rest of the `ds` ros ecosystem.

## Quick Rules for New ds_base::DsProcess-based Nodes

1. The new sensor must subclass `ds_base::DsProcess`
2. *Ideally* the new sensor should hide all implementation details within a subclass of
  `ds_sensors::SensorBase::Impl`
 
`ds_base::DsProcess` uses the PIMPL idom to hide implementation-specific details from the user.
"Implementation-specific" in this context refers to anything you might consider holding `protected`
or `private` in your class.  It isn't strictly necessary to follow this pattern in your own
classes, but doing so has some benefits that include:

- Setting up ros-related things (parameters, topics, subscriptions) is done for you IF you
  override the respective methods in `DsProcess::Impl`
- Maintaining a stable ABI for your library is much, much easier.  This is less of an issue if
  you're just creating executables.
  
## Step 0:  Read the Source Files!

- `include/ds_process/ds_process.h`
- `include/ds_process/ds_process_private.h`

Seriously, read the header files.  There will be a *lot* more detail in those two files than this one.
And while this document will strive to keep up to date with changes to `DsProcess` it may fall out
of sync at times.  Go read the two source files, then continue.

## Step 0:  Define What You're Building

Yes, this is another step 0.  This is on purpose.  Don't start coding BEFORE you have a decent idea
of what you want.  Let's create an example process that:

- Reads data over a serial port.
- Publishes data on a topic: 'output'
- Looks for a few parameters on the parameter server.
- Periodically publishes on a second topic 'time' based on a timer.

We'll call this project "example".

## Step 1: Start With the Bare Minimum

Create three new files:

- `include/example/example.h`
- `src/example/example_private.h`
- `src/example/example.cpp`

Here's the public header file.

```C++
// file:  include/example/example.h
#ifndef EXAMPLE_H
#define EXAMPLE_H

#include "ds_base/ds_process.h"

class Example : public ds_base::DsProcess {

 protected:
  // This will hold our implementation details.
  struct Impl;

 public:
  // Constructor overrides.  Match the same signatures as DsProcess
  explicit Example();
  Example(int argc, char* argv[], const std::string& name);

 protected:
  // Protected constructors so we can create subclasses of THIS class if desired.
  explicit Example(std::unique_ptr<Impl> impl);
  Example(std::unique_ptr<Impl> impl, int argc, char* argv[], const std::string& name);

 private:
  // Functions for accessing our implementation structure.  It is very important
  // to use these instead of direct access.
  auto d_func() noexcept -> Impl *;
  auto d_func() const noexcept -> Impl const *;
};

}
```

Here's the private implementation header.  NOTE: that this header file lives in `src`.  This is on purpose,
the whole point is to not export the details of the implementation in a public header.  You could make it public
by placing it in `include` if you want other people to be able to derive subclasses in their own libraries.

```C++
// file: src/example/example_private.h

#include "ds_base/ds_process_private.h"

// Not much to do here yet..
struct Example::Impl: public ds_base::DsProcess::Impl
{
  Impl(): ds_base::DsProcess::Impl() 
  {
  }
  ~Impl() override = default;
}
```

And finally here's the source file:

```C++
// file: src/example/example.cpp

#include "example/example.h"
#include "example_private.h"

//
// Our constructors use the protected constructor from `DsProcess`, providing our
// own version of the private implementation class.
//
// This newly constructed Example::Impl gets implicitly upcast to DsProcess::Impl 
// when passed to DsProcess's constructor.
//
// NOTE:  Our public constructors just forward on to our protected versions.  If
// we end up needing to add logic inside the constructors we'll only have to add
// it in two places now (the protected versions) instead of all four.

// Public default constructor:  use our own protected anolog
Example::Example() : Example(std::unique_ptr<Impl>(new Impl))
{
}

// Another public->protected forwarding.
Example::Example(int argc, char* argv[], const std::string& name)
    :Example(std::unique_ptr<Impl>(new Impl), argc, argv, name)
{
}

// Protected 'default' constructor
Example::Example(std::unique_ptr<Impl> impl) : ds_base::DsProcess(std::move(impl))
{
}

// Protected constructor with arguments for ros::init
Example::Example(std::unique_ptr<Impl> impl, int argc, char* argv[], const std::string& name)
    : ds_base::DsProcess(std::move(impl), argc, argv, name)
{
}

//
// This is how we get access to our new private Example::Impl.
// See, in the constructors above, we upcast Example::Impl into SensorBase::Impl, where
// it's stored in the SensorBase::impl_ member.
//
// To get the Impl class back *in the propper type* we need to downcast it again before
// working on it, which is why we have the static_cast<>'s here.
//
inline auto Aps1540::d_func() noexcept -> Example::Impl * {
  return static_cast<Example::Impl *>(ds_base::DsProcess::d_func());
}

inline auto Aps1540::d_func() const noexcept -> Example::Impl const * {
  return static_cast<Example::Impl const *>(ds_base::DsProcess::d_func());
}
```

Not too interesting yet.  We'll get there.  Right now we've got a new class that builds from `DsProcess`
but doesn't actually do anything new.

## Rule 2:  Handling ROS interaction during setup.

If you look at `include/ds_process/ds_process_private.h` you'll see a `DsProcess::Impl::setup` method, 
and lots of setup-helpers, like `DsProcess::Impl::setupConnections`.  As the header describes, 
`DsProcess::Impl::setup` is just a wrapper around all of the smaller setup functions.  This gives us
a good deal of modularity.  If we only care about adding a new parameter lookup we just need to override the 
paramter setup function.  If we need to add setup directives that don't fit nicely into one of the helper
functions then we can always override `setup` itself and add what we need.

**IMPORTANT:** the `Impl::setup` function is called for you in `DsProcess`'s constructors (take a look).
You shouldn't add hooks to this setup function in your own constructors!

So let's go back to our "design spec":

- Reads data over a serial port.
- Publishes data on a topic: 'output'
- Looks for a few parameters on the parameter server.
- Periodically publishes on a second topic 'time' based on a timer.

We need to add:

- two publishers
- an asio connection (with callback function)
- a timer (with callback function)

### Adding the Publishers

Let's add the publishers first:

```C++
// file: src/example/example_private.h

#include "ds_base/ds_process_private.h"

// Not much to do here yet..
struct Example::Impl: public ds_base::DsProcess::Impl
{
  Impl(): ds_base::DsProcess::Impl() 
  {
  }
  
  // our extra steps.
  void setupPublishers(ds_base::DsProcess* base) override
  {
    // Standard method overload chaining.  Call the base class somewhere, we'll do it first
    // here
    ds_base::DsProcess::Impl::setupPublishers(base); 
    
    // Now add our additional steps.
    out_pub_ = base->advertise<DataMessageType>("output", 10, false);
    time_pub_ = base->advertise<TimerMessageType>("time", 10, false);
  }  
    
  ~Impl() override = default;
  
  ros::Publisher out_pub_;  //!< Our publisher for outgoing data
  ros::Publisher time_pub_; //!< Our publisher for the timer callback.
}

```

### Adding the callbacks

Now, timers and connections require callbacks.  Lets define those first:

```C++
// file: src/example/example_private.h

#include "ds_base/ds_process_private.h"

struct Example::Impl: public ds_base::DsProcess::Impl
{
  Impl(): ds_base::DsProcess::Impl() 
  {
  }
  
  // our extra steps.
  void setupPublishers(ds_base::DsProcess* base) override
  {
    // Standard method overload chaining.  Call the base class somewhere, we'll do it first
    // here
    ds_base::DsProcess::Impl::setupPublishers(base);
    
    // Now add our additional steps.
    out_pub_ = base->advertise<DataMessageType>("output", 10, false);
    time_pub_ = base->advertise<TimerMessageType>("time", 10, false);
  }  
  
  // Called when we get some data on our connection.
  void connectionCallback(ds_core_msgs::RawData& bytes)
  {
    ROS_INFO("I received some data!!"); 
  }
  
  // Called form the timer
  void timerCallback(const ros::TimerEvent& event)
  {
    ROS_INFO("Timmer happened!");
  }
  
    
  ~Impl() override = default;
  
  ros::Publisher out_pub_;  //!< Our publisher for outgoing data
  ros::Publisher time_pub_; //!< Our publisher for the timer callback.
}

```

### Adding the connection and timer

And now let's add the timer and connection to our setup.

```C++
// file: src/example/example_private.h

#include "ds_base/ds_process_private.h"

struct Example::Impl: public ds_base::DsProcess::Impl
{
  Impl(): ds_base::DsProcess::Impl() 
  {
  }
  
  // add our connections
  void setupConnections(ds_base::DsProcess*base) override
  {
    ds_base::DsProcess::Impl::setupConnections(base);
    base->addConnection("connection_name", boost::bind(&Example::Impl::connectionCallback, this, _1));
  }
  
  // and our timers
  void setupTimers(ds_base::DsProcess*base) override
  {
    ds_base::DsProcess::Impl::setupTimers(base);
    timer_ = base->createTimer(ros::Duration(1), boost::bind(&Example::Impl::timerCallback, this, _1));
  }
  
  // our extra steps.
  void setupPublishers(ds_base::DsProcess* base) override
  {
    // Standard method overload chaining.  Call the base class somewhere, we'll do it first
    // here
    ds_base::DsProcess::Impl::setupPublishers(base);
    
    // Now add our additional steps.
    out_pub_ = base->advertise<DataMessageType>("output", 10, false);
    time_pub_ = base->advertise<TimerMessageType>("time", 10, false);
  }  
  
  // Called when we get some data on our connection.
  void connectionCallback(ds_core_msgs::RawData& bytes)
  {
    ROS_INFO("I received some data!!"); 
  }
  
  // Called form the timer
  void timerCallback(const ros::TimerEvent& event)
  {
    ROS_INFO("Timmer happened!");
  }
  
    
  ~Impl() override = default;
  
  ros::Publisher out_pub_;  //!< Our publisher for outgoing data
  ros::Publisher time_pub_; //!< Our publisher for the timer callback.
  ros::Timer timer_;        //!< Our timer object.
}

```

### Adding the parameter lookups

This is all good.  But let's say we want to parameterize some settings:

- The connection name should be read from the parameter server
- The timer period duration should be read form the parameter server

Let's enable that ability.

```C++
// file: src/example/example_private.h

#include "ds_base/ds_process_private.h"

struct Example::Impl: public ds_base::DsProcess::Impl
{
  Impl(): ds_base::DsProcess::Impl() 
  {
  }
  
  void setupParameters(ds_base::DsProcess* base) override
  {
    // look for a private param named "connection_name", default to "instrument"
    connection_name_ = ros::param::param<std::string>("~connection_name", "instrument"); 
    
    // Similar for the timer period, default to 1 second.
    timer_period_ = ros::param::param<double>("~timer_period", 1);
  }
  
  // add our connections - now parameterized
  void setupConnections(ds_base::DsProcess* base)
  {
    ds_base::DsProcess::Impl::setupConnections(base) override;
    // The name of the connection is now read from the parameter server
    base->addConnection(connection_name_, boost::bind(&Example::Impl::connectionCallback, this, _1));
  }
  
  // and our timers
  void setupTimers(ds_base::DsProcess*base) override
  {
    ds_base::DsProcess::Impl::setupTimers(base);
    // And same for the timer_
    timer_ = base->createTimer(ros::Duration(timer_period_), boost::bind(&Example::Impl::timerCallback, this, _1));
  }
  
  // our extra steps.
  void setupPublishers(ds_base::DsProcess* base) override
  {
    // Standard method overload chaining.  Call the base class somewhere, we'll do it first
    // here
    ds_base::DsProcess::Impl::setupPublishers(base);
    
    // Now add our additional steps.
    out_pub_ = base->advertise<DataMessageType>("output", 10, false);
    time_pub_ = base->advertise<TimerMessageType>("time", 10, false);
  }  
  
  // Called when we get some data on our connection.
  void connectionCallback(ds_core_msgs::RawData& bytes)
  {
    ROS_INFO("I received some data!!"); 
  }
  
  // Called form the timer
  void timerCallback(const ros::TimerEvent& event)
  {
    ROS_INFO("Timmer happened!");
  }
  
    
  ~Impl() override = default;
  
  std::string connection_name_ //!< Our connection name
  double timer_period_         //!< The timer period.
  ros::Publisher out_pub_;     //!< Our publisher for outgoing data
  ros::Publisher time_pub_;    //!< Our publisher for the timer callback.
  ros::Timer timer_;           //!< Our timer object.
}

```

### Adding directives that don't quite fit...

As mentioned, you're not limited to the `setup`* methods.  You can override `setup()` itself!  Let's do
that to start the timer after everything else is done:

```C++
// file: src/example/example_private.h

#include "ds_base/ds_process_private.h"

struct Example::Impl: public ds_base::DsProcess::Impl
{
  Impl(): ds_base::DsProcess::Impl() 
  {
  }
  
  // Start our timer after setup completes. 
  void setup(ds_base::DsProcess* base) override
  {
    ds_base::DsProcess::Impl::setup(base);
    timer_.start();
  }
  void setupParameters(ds_base::DsProcess* base) override
  {
    // look for a private param named "connection_name", default to "instrument"
    connection_name_ = ros::param::param<std::string>("~connection_name", "instrument"); 
    
    // Similar for the timer period, default to 1 second.
    timer_period_ = ros::param::param<double>("~timer_period", 1);
  }
  
  // add our connections - now parameterized
  void setupConnections(ds_base::DsProcess* base) override
  {
    ds_base::DsProcess::Impl::setupConnections(base);
    // The name of the connection is now read from the parameter server
    base->addConnection(connection_name_, boost::bind(&Example::Impl::connectionCallback, this, _1));
  }
  
  // and our timers
  void setupTimers(ds_base::DsProcess*base) override
  {
    ds_base::DsProcess::Impl::setupTimers(base);
    // And same for the timer_
    timer_ = base->createTimer(ros::Duration(timer_period_), boost::bind(&Example::Impl::timerCallback, this, _1));
  }
  
  // our extra steps.
  void setupPublishers(ds_base::DsProcess* base) override
  {
    // Standard method overload chaining.  Call the base class somewhere, we'll do it first
    // here
    ds_base::DsProcess::Impl::setupPublishers(base);
    
    // Now add our additional steps.
    out_pub_ = base->advertise<DataMessageType>("output", 10, false);
    time_pub_ = base->advertise<TimerMessageType>("time", 10, false);
  }  
  
  // Called when we get some data on our connection.
  void connectionCallback(ds_core_msgs::RawData& bytes)
  {
    ROS_INFO("I received some data!!"); 
  }
  
  // Called form the timer
  void timerCallback(const ros::TimerEvent& event)
  {
    ROS_INFO("Timmer happened!");
  }
  
    
  ~Impl() override = default;
  
  std::string connection_name_ //!< Our connection name
  double timer_period_         //!< The timer period.
  ros::Publisher out_pub_;     //!< Our publisher for outgoing data
  ros::Publisher time_pub_;    //!< Our publisher for the timer callback.
  ros::Timer timer_;           //!< Our timer object.
}

```

## Interacting With the Public Class

If you need the PIMPL object to access the public class, the easiest way to do that is to
add a pointer to the public class as an argument of the private member.  For example:

In the PIMPL:

```C++

void Example::Impl::someMethod(Example* base, double data)
{
    // need to pass data to some public member
    base->methodThatTakesDouble(data);
}

```

Your public class:
```C++

void Example::methodThatCallsPrivate(double data)
{
    // Get a poitner to the impl
    auto d = d_func();
    
    // Call the private member, which in turn will call the next
    // method defined below.
    d->someMethod(this, data);
}

void Example::methodThatTakesDouble(double data)
{
    // does something interesting here
}

```

So calling `Example::methodThatCallsPrivate(data)` executes the following in order:

- `Example::methodThatCallsPrivate(data)`
- `Example::Impl::someMethod(data)`
- `Example::methodThatTakesDouble(double data)`
