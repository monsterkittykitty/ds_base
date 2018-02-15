//
// Created by ivaughn on 1/30/18.
//

#ifndef PROJECT_DS_PARAM_PRIVATE_H
#define PROJECT_DS_PARAM_PRIVATE_H

#include <ds_param/ds_param.h>
#include <ds_param/ds_param_conn.h>

namespace ds_param {
// we need an abstract base class here
class UpdatingParamPrivate {
  // make noncopyable
 private:
  UpdatingParamPrivate(const UpdatingParamPrivate&) = delete;
  UpdatingParamPrivate& operator=(const UpdatingParamPrivate&) = delete;

 public:
  UpdatingParamPrivate(const std::shared_ptr<ParamConnectionPrivate>& _c, const std::string& _n, bool _a)
      : conn(_c), name(_n), advertise_flag(_a), dirty(false) {}
  virtual ~UpdatingParamPrivate() = default;

  std::shared_ptr<ParamConnectionPrivate> conn;
  std::string name;
  bool advertise_flag;
  bool dirty;

};

template<typename T>
class UpdatingParamTPrivate : public UpdatingParamPrivate {

 // make noncopyable
 private:
  UpdatingParamTPrivate(const UpdatingParamTPrivate&) = delete;
  UpdatingParamTPrivate& operator=(const UpdatingParamTPrivate&) = delete;

 public:

  typedef T ValueType;
  T value;
  boost::optional<T> prev_value;


  UpdatingParamTPrivate(const std::shared_ptr<ParamConnectionPrivate>& _c, const std::string& _n, bool _a)
      : UpdatingParamPrivate(_c, _n, _a) {}
};

class UpdatingParamEnumPrivate : public UpdatingParamTPrivate<int> {
  // make noncopyable
 private:
  UpdatingParamEnumPrivate(const UpdatingParamEnumPrivate&) = delete;
  UpdatingParamEnumPrivate& operator=(const UpdatingParamEnumPrivate&) = delete;

 public:

  /// \brief A list of name/value pairs for our enum
  std::vector<std::pair<std::string, int> > namedValues;

  UpdatingParamEnumPrivate(const std::shared_ptr<ParamConnectionPrivate>& _c, const std::string& _n, bool _a)
      : UpdatingParamTPrivate<int>(_c, _n, _a) {}
};
}
#endif //PROJECT_DS_PARAM_PRIVATE_H
