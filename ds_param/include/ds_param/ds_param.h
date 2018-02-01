//
// Created by ivaughn on 1/30/18.
//

#ifndef PROJECT_DS_PARAM_H
#define PROJECT_DS_PARAM_H

#include <memory>
#include "ds_global.h"

#include <ds_core_msgs/ParamUpdate.h>
#include <ros/ros.h>

namespace ds_param {

// forward declaration: see ds_param_connection.h
class ParamConnection;
class ParamConnectionPrivate;
class UpdatingParamPrivate;

/// \brief The abstract base class for all our parameter types.
///
/// Can be useful for type erasure, etc
class UpdatingParam {
 public:
  virtual ~UpdatingParam() = 0;

  /// \brief Get a YAML block description of this object.  Exact contents vary with type
  virtual std::string YamlDescription() const;

  /// \brief Get a string describing this type (also available in the YAML)
  virtual std::string Type() const = 0;

  /// \brief Get the fully-resolved ROS name for this type
  const std::string& Name() const;

  /// \brief Load this parameter from the server.  Called automatically when instantiated with a connection
  virtual void loadFromServer() = 0;

  /// \brief Fill an update message
  virtual void fillUpdateMessage(ds_core_msgs::ParamUpdate& msg) const = 0;

  /// \brief Get the advertise flag
  ///
  /// As a rule, individual programs should only advertise parameters they listen to, not parameters they
  /// set.  That way a configuring utilities don't populate the configuration options list
  ///
  /// \return Wether this process advertises this variable or not
  bool Advertise() const;
  void setAdvertise(bool v);

 protected:
  // only constructor is protected; as this class is pure-virtual,
  // it can only be instantiated by a child class anyway
  UpdatingParam(const std::shared_ptr<UpdatingParamPrivate>& impl);

  std::shared_ptr<UpdatingParamPrivate> d_ptr_;

 private:
  auto d_func() noexcept -> UpdatingParamPrivate*;
  auto d_func() const noexcept -> UpdatingParamPrivate const*;
};

// This class will ONLY work for a limited set of types

// forward declaration
template<typename T>
class UpdatingParamTPrivate;

template <typename T>
class UpdatingParamT : public UpdatingParam {

 protected:
  UpdatingParamT(const std::shared_ptr<UpdatingParamTPrivate<T> >& impl);
 public:
  UpdatingParamT(const std::shared_ptr<ParamConnectionPrivate>& conn, const std::string& name, bool advertise);

  typedef T ValueType;
  typedef std::shared_ptr<UpdatingParamT<T> > Ptr;
  typedef std::shared_ptr<const UpdatingParamT<T> > ConstPtr;

  const T& Get() const;
  void Set(const T& _v);

  virtual std::string YamlDescription() const override;
  virtual std::string Type() const;

  virtual void loadFromServer();
  virtual void fillUpdateMessage(ds_core_msgs::ParamUpdate& msg) const;

  // TODO: Add callback-on-change

 protected:
  void setOnServer();
  // updates the value but does NOT trigger updates; ONLY to be used internally (hence protected)
  void updateValue(const T& _v);
  friend ParamConnectionPrivate; // needed to use updateValue

 private:
  auto d_func() noexcept -> UpdatingParamTPrivate<T>*;
  auto d_func() const noexcept -> UpdatingParamTPrivate<T> const*;

};

// forward declaration
class UpdatingParamEnumPrivate;

/// \brief A class to add some discovery options for enums on top of the standard
/// int implementation
///
/// Note that current enum values are NOT shared between different instances of the same value!
/// That's a pretty major TODO
class UpdatingParamEnum : public UpdatingParamT<int> {
 public:
  UpdatingParamEnum(const std::shared_ptr<ParamConnectionPrivate>& conn, const std::string& name, bool advertise);

  typedef std::shared_ptr<UpdatingParamEnum> Ptr;
  typedef std::shared_ptr<const UpdatingParamEnum> ConstPtr;

  // getters, setters, type, etc are all inherited
  virtual std::string YamlDescription() const override;

  void addNamedValue(const std::pair<std::string, int>& value);
  void addNamedValue(const std::string& name, int value);
  const std::vector<std::pair<std::string, int> >& getNamedValues() const;
  std::vector<std::pair<std::string, int> >& getNamedValues();

  std::string getValueByName() const;
  void setValueByName(const std::string& name);
  bool hasNamedValue(const std::string& name) const;

 protected:
  friend ParamConnectionPrivate;  // need to use updateValue

 private:
  auto d_func() noexcept -> UpdatingParamEnumPrivate*;
  auto d_func() const noexcept -> UpdatingParamEnumPrivate const*;
};

typedef UpdatingParamT<bool> BoolParam;
typedef UpdatingParamT<int> IntParam;
typedef UpdatingParamT<float> FloatParam;
typedef UpdatingParamT<double> DoubleParam;
typedef UpdatingParamT<std::string> StringParam;
typedef UpdatingParamEnum EnumParam;

}

#endif //PROJECT_DS_PARAM_H
