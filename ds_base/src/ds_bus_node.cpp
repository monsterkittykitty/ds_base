//
// Created by ivaughn on 1/15/18.
//

/// \file This is a node that uses the DsBus class to expose a single comms bus over ROS

#include <ds_base/ds_bus.h>
#include <boost/program_options.hpp>
#include <memory>

int main(int argc, char* argv[])
{
  // startup ROS.  You almost-definitely want to remap the name
  ros::init(argc, argv, "bus");

  // Create our node
  auto node = std::unique_ptr<ds_base::DsBus>(new ds_base::DsBus);

  // Run the node.  Blocks until exit
  node->run();

  // Return success
  return 0;
}
