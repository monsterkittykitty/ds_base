cmake_minimum_required(VERSION 2.8.3)
project(ds_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ds_core_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ds_process ds_callbackqueue ds_connection ds_udp ds_asio ds_serial ds_nodehandle ds_connection_factory
  CATKIN_DEPENDS roscpp ds_core_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
)

add_library(ds_process
  src/ds_process.cpp
)

add_library(ds_callbackqueue
  src/ds_callbackqueue.cpp
)

add_library(ds_udp
  src/ds_udp.cpp
)

add_library(ds_connection
  src/ds_connection.cpp
)

add_library(ds_asio
  src/ds_asio.cpp
)

add_library(ds_serial
  src/ds_serial.cpp
)

add_library(ds_nodehandle
  src/ds_nodehandle.cpp
)

add_library(ds_connection_factory
  src/ds_connection_factory.cpp
)

target_link_libraries(ds_process
  ${Boost_SYSTEM_LIBRARY}
  ds_connection
  ds_udp
  ds_asio
  ds_callbackqueue
  ds_udp
  ds_serial
  ds_nodehandle
  ds_connection_factory
)

target_link_libraries(ds_callbackqueue
  ${Boost_SYSTEM_LIBRARY}
)

target_link_libraries(ds_connection
  ${Boost_SYSTEM_LIBRARY}
)

target_link_libraries(ds_udp
  ${Boost_SYSTEM_LIBRARY}
  ds_connection
)

target_link_libraries(ds_serial
  ${Boost_SYSTEM_LIBRARY}
  ds_connection
)

target_link_libraries(ds_asio
  ${Boost_SYSTEM_LIBRARY}
)

target_link_libraries(ds_nodehandle
  ${Boost_SYSTEM_LIBRARY}
  ds_callbackqueue
)

target_link_libraries(ds_connection_factory
  ${Boost_SYSTEM_LIBRARY}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ds_base.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
