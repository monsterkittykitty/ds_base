//
// Created by ivaughn on 2/15/18.
//

#ifndef PROJECT_DS_PARAM_GUARD_H
#define PROJECT_DS_PARAM_GUARD_H

#include <memory>

#include "ds_param_conn.h"

namespace ds_param {

/// \brief A class to block updates from going out during its lifetime.
///
/// Used to allow multiple updates to happen in a single message
/// You use this as follows:
///
///  {
///    ParamGuard lock(conn);
///    var_foo.Set(12);
///    var_bar.Set(16);
///  } // lock passes out of scope and automatically unlocks
///    // The connection will be informed and send the most recent value of
///    // any variables that have changed since the lock was placed in a single message
///
class ParamGuard {
 public:
  ParamGuard(const ParamConnection::Ptr& _c);

  /// \brief Destructor automatically unlocks the param connection
  virtual ~ParamGuard();

  /// \brief Explicit unlock function
  void unlock();

  // delete our pointer allocation operators so that users have to use this on the
  // stack.
 private:
  static void *operator new     (size_t) = delete;
  static void *operator new[]   (size_t) = delete;

 protected:
  ParamConnection::Ptr conn;
};

}
#endif //PROJECT_DS_PARAM_GUARD_H
