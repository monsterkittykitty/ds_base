//
// Created by ivaughn on 1/30/18.
//

#include <ds_param/ds_param.h>
#include <ds_param/ds_param_conn.h>
#include "ds_param_private.h"
#include "ds_param_conn_private.h"

namespace ds_param {

//------------------------------------------------------------------------------
// UpdatingParam (super-class)
//------------------------------------------------------------------------------
UpdatingParam::UpdatingParam(const std::shared_ptr<UpdatingParamPrivate>& _impl)
: d_ptr_(_impl) {

}

UpdatingParam::~UpdatingParam() {

}

std::string UpdatingParam::YamlDescription() const {
  return "{ " + Name() + " : UNKNOWN }";
}

const std::string &UpdatingParam::Name() const {
  return d_func()->name;
}

bool UpdatingParam::Advertise() const {
  return d_func()->advertise_flag;
}

void UpdatingParam::setAdvertise(bool v) {
  d_func()->advertise_flag = v;
}

inline auto UpdatingParam::d_func() noexcept -> UpdatingParamPrivate* {
  return d_ptr_.get();
}

inline auto UpdatingParam::d_func() const noexcept -> UpdatingParamPrivate const* {
  return d_ptr_.get();
}

//------------------------------------------------------------------------------
// Param<T> (generic type)
//------------------------------------------------------------------------------

#define PT_D auto d =  UpdatingParamT<T>::d_func()

template<typename T>
UpdatingParamT<T>::UpdatingParamT(const std::shared_ptr<UpdatingParamTPrivate<T> >& _impl)
    : UpdatingParam(_impl) {
  // do nothing
}

template<typename T>
UpdatingParamT<T>::UpdatingParamT(const std::shared_ptr<ParamConnectionPrivate> &_c,
                                  const std::string &_n, bool _a)
    : UpdatingParam(std::shared_ptr<UpdatingParamPrivate>(
    new UpdatingParamTPrivate<T>(_c, _n, _a))) {
  // do nothing
}

template<typename T>
const T &UpdatingParamT<T>::Get() const {
  PT_D;
  return d->value;
}

template<typename T>
void UpdatingParamT<T>::Set(const T &_v) {

  PT_D;

  // First, update our local copy
  d->value = _v;

  // Now the parameter server
  setOnServer();

  // Fire off a message indicating that a value has updated
  d->conn->signalUpdate(this);
}

template<typename T>
void UpdatingParamT<T>::updateValue(const T& _v) {
  PT_D;
  d->value = _v;

  // The public should use use.  They want set.  Because it propagates changes
  // and stuff.
  //
  // HOWEVER, when we get a change from the outside, we want to use
  // a setter that does not tell the world. Because if we told the world...
  // CATASTROPHIC UPDATE STORM (!)

  // so yeah, do nothing here.  And use set, you want set.  Really.
}

template<typename T>
std::string UpdatingParamT<T>::YamlDescription() const {
  return "{ name: \"" + Name() + "\", type: \"" + Type() + "\" }";
}

template<typename T>
inline auto UpdatingParamT<T>::d_func() noexcept -> UpdatingParamTPrivate<T>* {
  return static_cast<UpdatingParamTPrivate<T> *>(UpdatingParam::d_ptr_.get());
}

template<typename T>
inline auto UpdatingParamT<T>::d_func() const noexcept -> UpdatingParamTPrivate<T> const* {
  return static_cast<UpdatingParamTPrivate<T> const *>(UpdatingParam::d_ptr_.get());
}

template<typename T>
void UpdatingParamT<T>::loadFromServer() {
  PT_D;
  if (!d->conn->getHandle().getParam(d->name, d->value)) {
    ROS_ERROR_STREAM("Could not set variable \"" <<Name() <<"\"<" <<Type() <<"> from server!");
  }
}

template<typename T>
void UpdatingParamT<T>::setOnServer() {
  PT_D;
  d->conn->getHandle().setParam(d->name, d->value);
  // no error checking.  ROS should really fix that someday.
}

//--------------------------------------------------------------------
// Specializations
template<>
std::string UpdatingParamT<std::string>::Type() const {
  return "string";
}

template<>
std::string UpdatingParamT<int>::Type() const {
  return "int";
}

template<>
std::string UpdatingParamT<double>::Type() const {
  return "double";
}

template<>
std::string UpdatingParamT<float>::Type() const {
  return "float";
}

template<>
std::string UpdatingParamT<bool>::Type() const {
  return "bool";
}


template<>
void UpdatingParamT<bool>::fillUpdateMessage(ds_core_msgs::ParamUpdate& msg) const {
  auto d = UpdatingParamT<bool>::d_func();
  ds_core_msgs::KeyBool ret;
  ret.key = d->name;
  ret.value = d->value;
  msg.bools.push_back(ret);
}

template<>
void UpdatingParamT<int>::fillUpdateMessage(ds_core_msgs::ParamUpdate& msg) const {
  auto d = UpdatingParamT<int>::d_func();
  ds_core_msgs::KeyInt ret;
  ret.key = d->name;
  ret.value = d->value;
  msg.ints.push_back(ret);
}

template<>
void UpdatingParamT<float>::fillUpdateMessage(ds_core_msgs::ParamUpdate& msg) const {
  auto d = UpdatingParamT<float>::d_func();
  ds_core_msgs::KeyFloat ret;
  ret.key = d->name;
  ret.value = d->value;
  msg.floats.push_back(ret);
}

template<>
void UpdatingParamT<double>::fillUpdateMessage(ds_core_msgs::ParamUpdate& msg) const {
  auto d = UpdatingParamT<double>::d_func();
  ds_core_msgs::KeyDouble ret;
  ret.key = d->name;
  ret.value = d->value;
  msg.doubles.push_back(ret);
}

template<>
void UpdatingParamT<std::string>::fillUpdateMessage(ds_core_msgs::ParamUpdate& msg) const {
  auto d = UpdatingParamT<std::string>::d_func();
  ds_core_msgs::KeyString ret;
  ret.key = d->name;
  ret.value = d->value;
  msg.strings.push_back(ret);
}
//--------------------------------------------------------------------
// explicit instantiations
// ONLY these types can be used
template class UpdatingParamT<bool>;
template class UpdatingParamT<int>;
template class UpdatingParamT<float>;
template class UpdatingParamT<double>;
template class UpdatingParamT<std::string>;


#define PE_D auto d = d_func()

UpdatingParamEnum::UpdatingParamEnum(const std::shared_ptr<ParamConnectionPrivate>& _c,
                                     const std::string& _n, bool _a)
    : UpdatingParamT<int>(std::shared_ptr<UpdatingParamEnumPrivate>(new UpdatingParamEnumPrivate(_c, _n, _a))) {
  // do nothing
}

std::string UpdatingParamEnum::YamlDescription() const {
  PE_D;
  std::stringstream ret;
  ret << "{ name: \"" << d->name <<"\",";
  ret <<" type: \"" << Type() <<"\",";
  ret <<" enum: {";
  for (auto iter=d->namedValues.begin(); iter != d->namedValues.end(); iter++) {
    ret <<" \"" <<iter->first <<"\": " <<iter->second <<", ";
  }
  ret <<"} }";

  return ret.str();
}

void UpdatingParamEnum::addNamedValue(const std::string& _n, int _v) {
  PE_D;
  d->namedValues.push_back(std::pair<std::string, int>(_n, _v));
  d->conn->publishDescription();
}

void UpdatingParamEnum::addNamedValue(const std::pair<std::string, int>& value) {
  PE_D;
  d->namedValues.push_back(value);
  d->conn->publishDescription();
}

const std::vector<std::pair<std::string, int> >& UpdatingParamEnum::getNamedValues() const {
  PE_D;
  return d->namedValues;
};

std::vector<std::pair<std::string, int> >& UpdatingParamEnum::getNamedValues() {
  PE_D;
  return d->namedValues;
};

std::string UpdatingParamEnum::getValueByName() const {
  PE_D;
  for (auto iter=d->namedValues.begin(); iter != d->namedValues.end(); iter++) {
    if (iter->second == d->value) {
      return iter->first;
    }
  }

  std::string ret = std::to_string(d->value);
  ROS_ERROR_STREAM("Parameter \"" <<d->name <<"\" set to unrecognized enum type; returning \"" <<ret <<"\"");
  return ret;
}

void UpdatingParamEnum::setValueByName(const std::string& name) {
  PE_D;
  for (auto iter=d->namedValues.begin(); iter != d->namedValues.end(); iter++) {
    if (iter->first == name) {
      Set(iter->second);
      return;
    }
  }

  ROS_ERROR_STREAM("Pariable \"" <<d->name <<"\" tried to set enumerated value to \""
                                 <<name <<"\", but that value name is not defined!");

}

bool UpdatingParamEnum::hasNamedValue(const std::string& name) const {
  PE_D;
  for (auto iter=d->namedValues.begin(); iter != d->namedValues.end(); iter++) {
    if (iter->first == name) {
      return true;
    }
  }
  return false;
}

inline auto UpdatingParamEnum::d_func() noexcept -> UpdatingParamEnumPrivate* {
  return static_cast<UpdatingParamEnumPrivate *>(UpdatingParam::d_ptr_.get());
}

inline auto UpdatingParamEnum::d_func() const noexcept -> UpdatingParamEnumPrivate const* {
  return static_cast<UpdatingParamEnumPrivate const *>(UpdatingParam::d_ptr_.get());
}

}

