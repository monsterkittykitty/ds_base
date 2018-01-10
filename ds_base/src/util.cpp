//
// Created by zac on 1/10/18.
//

#include "ds_base/util.h"
#include <boost/uuid/name_generator.hpp>

namespace ds_base
{

boost::uuids::uuid generateUuid(const std::string &id, const boost::uuids::uuid &namespace_)
{
  return boost::uuids::name_generator(namespace_)(id);
}

}
