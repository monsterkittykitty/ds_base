#include "ds_base/controller_base.h"
#include "controller_base_private.h"

namespace ds_base
{

ControllerBase::ControllerBase()
  : DsProcess()
  , d_ptr_(std::unique_ptr<ControllerBasePrivate>(new ControllerBasePrivate))
{
}
ControllerBase::ControllerBase(int argc, char **argv, const std::string &name)
    : DsProcess(argc, argv, name)
    , d_ptr_(std::unique_ptr<ControllerBasePrivate>(new ControllerBasePrivate))
{
}

ControllerBase::~ControllerBase() = default;

void ControllerBase::setEnabled(bool enabled)
{
  DS_D(ControllerBase);
  d->is_enabled_ = enabled;
}

bool ControllerBase::enabled() const noexcept
{
  const DS_D(ControllerBase);
  return d->is_enabled_;
}
}
