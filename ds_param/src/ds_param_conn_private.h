//
// Created by ivaughn on 1/31/18.
//

#ifndef PROJECT_DS_PARAM_CONN_PRIVATE_H
#define PROJECT_DS_PARAM_CONN_PRIVATE_H

#include "ds_param/ds_param_conn.h"

#include "ds_core_msgs/ParamUpdate.h"
#include "ds_core_msgs/ParamDescription.h"

#include <iomanip>
#include <sstream>

namespace ds_param {

class ParamConnectionPrivate {
  // make non-copyable
 private:
  ParamConnectionPrivate(const ParamConnectionPrivate&) = delete;
  ParamConnectionPrivate& operator=(const ParamConnectionPrivate&) = delete;

 public:
  ParamConnectionPrivate(ros::NodeHandle& h): handle(h) {
    descriptionPub = handle.advertise<ds_core_msgs::ParamDescription>("/updating_param/description", 1, true);
    updatePub = handle.advertise<ds_core_msgs::ParamUpdate>("/updating_param/updates", 1, true);
    updateSub = handle.subscribe("/updating_param/updates", 10, &ParamConnectionPrivate::update_callback, this);

    // Resolve our own name
    std::stringstream nameBuilder;
    nameBuilder <<handle.resolveName(ros::this_node::getName());
    nameBuilder <<"##";
    nameBuilder <<std::setw(9) <<ros::WallTime::now().nsec;
    conn_name = nameBuilder.str();

    ROS_INFO_STREAM("Initialized ds_param::ParamConnection named \"" <<conn_name <<"\"");
  }

  void update_callback(ds_core_msgs::ParamUpdatePtr msg) {
    if (msg->source == conn_name) {
      // DO NOTHING in response to our own messages
      return;
    }

    update_subtype<bool>(msg->bools);
    update_subtype<int>(msg->ints);
    update_subtype<float>(msg->floats);
    update_subtype<double>(msg->doubles);
    update_subtype<std::string>(msg->strings);
  }

  // this is a clever trick... since all the message fields have the
  // same fields, we can abuse templates to write all this code once.
  // The only downside is that we can't enforce type safety
  // between T and MSG-- however, the only place this is used is
  // LITERALLY the previous set of lines, so the simple answer is
  // "please don't screw it"
  //
  // The first template parameter (T) is the type you want to cast to
  // The second template parameter (MSG) is the type that comes from the ROS
  // message-- BECAUSE its a parameter, the compiler will magically fill it in correctly
  // HOWEVER, we have no such luck for "T", because there's no way for the compiler to guess
  // what type you mean.  So you have to tell it.  Which is fine.
  template<typename T, typename MSG>
  void update_subtype(const MSG& vec) {

    for (typename MSG::const_iterator iter = vec.begin(); iter != vec.end(); iter++) {

      auto param_iter = params.find(iter->key);

      if (param_iter != params.end()) {

        typename UpdatingParamT<T>::Ptr param = std::dynamic_pointer_cast<UpdatingParamT<T> >(param_iter->second);
        if (!param) {
          ROS_ERROR_STREAM("Found local parameter for key \"" <<param_iter->first <<"\", but could not "
              "cast it to the requested type! Ignoring update...");
        } else {
          param->updateValue(iter->value);
        }

      }
      // if not found, we assume the variable exists and we just don't care about it
      // so no error, no nothing
    }
  }

  ros::NodeHandle& getHandle() {
    return handle;
  }

  const ros::NodeHandle& getHandle() const {
    return handle;
  }

  void signalUpdate(const UpdatingParam *const p) {
    ds_core_msgs::ParamUpdate msg;
    p->fillUpdateMessage(msg);

    msg.stamp = ros::Time::now();
    msg.source = conn_name;
    updatePub.publish(msg);
  }

  void publishDescription() {
    // build our YAML payload
    std::stringstream ret;
    ret <<"node:\n";
    ret <<"    name: " <<ros::this_node::getName() <<"\n";
    ret <<"    namespace: " <<ros::this_node::getNamespace() <<"\n";
    ret <<"params: [ ";
    for (auto iter = params.begin(); iter!= params.end(); iter++) {
      ret <<iter->second->YamlDescription() <<", ";
    }
    ret <<"]\n";

    // Publish
    ds_core_msgs::ParamDescription toSend;
    toSend.yaml_payload = ret.str();
    descriptionPub.publish(toSend);
  }

 public:
  ros::NodeHandle& handle;
  ros::Publisher descriptionPub;
  ros::Publisher updatePub;
  ros::Subscriber updateSub;

  std::string conn_name;
  std::map<std::string, std::shared_ptr<UpdatingParam> > params;

};

}

#endif //PROJECT_DS_PARAM_CONN_PRIVATE_H
