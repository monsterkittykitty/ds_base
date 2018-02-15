//
// Created by ivaughn on 2/15/18.
//

#include "ds_param/ds_param_guard.h"
#include "ds_param/ds_param_conn.h"

#include <stdexcept>

namespace ds_param {


ParamGuard::ParamGuard(const ParamConnection::Ptr& _c) : conn(_c) {
  // don't lock an already-locked connection
  if (conn->IsLocked()) {
    // throw exception
    // note that our destructor will NOT be called because we're throwing an exception
    // in the constructor so explicitly clean up our conn member
    conn.reset();

    // ALWAYS make a pretty error message!
    std::string msg("ds_param::ParamGuard attempted to lock a connection named "
                        + conn->connName() + " but it was already locked!");
    ROS_ERROR_STREAM(msg);
    throw std::invalid_argument(msg);
  }

  // ok, everything looks good, go ahead and lock the connection
  conn->lock();
}

ParamGuard::~ParamGuard() {
  unlock();
}

void ParamGuard::unlock() {
  conn->unlock();
}


}
