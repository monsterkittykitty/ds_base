#ifndef DS_BASE_REFERENCE_BASE_PRIVATE_H
#define DS_BASE_REFERENCE_BASE_PRIVATE_H

namespace ds_base
{

struct ReferenceBasePrivate
{
  ReferenceBasePrivate(): is_enabled_(false)
  {
  }
  ~ReferenceBasePrivate() = default;

  bool is_enabled_;
  ros::Publisher ref_pub_;
};

}
#endif //DS_BASE_REFERENCE_BASE_PRIVATE_H
