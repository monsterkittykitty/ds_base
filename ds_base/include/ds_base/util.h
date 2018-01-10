#ifndef DS_BASE_UTIL_H
#define DS_BASE_UTIL_H

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/nil_generator.hpp>
#include <string>
namespace ds_base
{

/// @brief Generate a deterministic UUID for this sensor
///
/// The generated UUID should be determanistic.  That is,
/// it should be able to be stored in a config file and checked
/// against a value calculated at run time.
///
/// Namespace UUID's are good for this, using a string as the unique
/// hash.
boost::uuids::uuid generateUuid(const std::string& id, const boost::uuids::uuid& namespace_=boost::uuids::nil_uuid());

}

#endif //DS_BASE_UTIL_H
