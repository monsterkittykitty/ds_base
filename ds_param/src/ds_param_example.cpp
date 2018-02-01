//
// Created by ivaughn on 1/30/18.
//

#include "ds_param/ds_param_conn.h"


class ParamDemo {

 public:
  ParamDemo(ros::NodeHandle& _nh) : handle(_nh) {
    conn = ds_param::ParamConnection::create(handle);

    // Grab a copy of our STATIC parameters
    // Note that this is a standard ROS call
    handle.getParam(ros::this_node::getName() + "/other_node", other_node);
    handle.getParam(ros::this_node::getName() + "/start_idx", start);

    // connect to OUR shared enum
    param_enum = conn->connect<ds_param::EnumParam>(ros::this_node::getName() + "/test_enum_param", true);
    // setup our shared enum
    param_enum->addNamedValue("Option 1", 1);
    param_enum->addNamedValue("non-consecutive Option 7", 7);

    // connect to OUR shared parameters
    param_int = conn->connect<ds_param::IntParam>(ros::this_node::getName() + "/test_int_param", true);
    param_str = conn->connect<ds_param::StringParam>(ros::this_node::getName() + "/test_str_param", true);

    // connect to the OTHER NODE's shared parameters
    other_int = conn->connect<ds_param::IntParam>("/" + other_node + "/test_int_param", false);
    other_str = conn->connect<ds_param::StringParam>("/" + other_node + "/test_str_param", false);

    // test prep
    idx=start;
  }

  void _callback(const ros::TimerEvent& evt) {
    std::stringstream ss;
    ss <<"Idx " <<start <<" -> " <<idx;

    other_int->Set(idx);
    other_str->Set(ss.str());

    ROS_ERROR_STREAM(ros::this_node::getName()

                         <<": " <<"MY param: " << param_int->Get()
                         <<" --> \"" <<param_str->Get() <<"\""

                         <<"    OTHER param: " << other_int->Get()
                         <<" --> \"" <<other_str->Get() <<"\"");
    idx++;

  }

 protected:
  ros::NodeHandle& handle;
  ds_param::ParamConnection::Ptr conn;
  int idx;

  // OUR static paramters
  std::string other_node;
  int start;

  // OUR shared parameters
  ds_param::EnumParam::Ptr param_enum;
  ds_param::IntParam::Ptr param_int;
  ds_param::StringParam::Ptr param_str;

  // The OTHER NODE's shared parameters
  ds_param::IntParam::Ptr other_int;
  ds_param::StringParam::Ptr other_str;
};


int main(int argc, char* argv[]) {

  ros::init(argc, argv, "test_node_1");
  ros::NodeHandle nh;
  ParamDemo demo(nh);

  ros::Timer timer = nh.createTimer(ros::Duration(3.0),
                              boost::bind(&ParamDemo::_callback, &demo, _1), false);

  // spins until shut down
  while (ros::ok()) {
    ros::spin();
  }

  return 0;
}