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

const ds_nav_msgs::AggregatedState& ControllerBase::reference() const noexcept {
  const DS_D(ControllerBase);
  return d->last_reference_;
}
void ControllerBase::setReference(const ds_nav_msgs::AggregatedState &msg) {
  DS_D(ControllerBase);
  d->last_reference_ = msg;
}

const ds_nav_msgs::AggregatedState& ControllerBase::state() const noexcept {
  const DS_D(ControllerBase);
  return d->last_state_;
}
void ControllerBase::setState(const ds_nav_msgs::AggregatedState &msg) {
  DS_D(ControllerBase);
  d->last_state_ = msg;
}


void ControllerBase::setupSubscriptions() {
  DsProcess::setupSubscriptions();
  DS_D(ControllerBase);
  d->state_update_sub_ = nodeHandle()->subscribe("state_input", 1, &ControllerBase::setState, this);
  d->reference_update_sub_ = nodeHandle()->subscribe("reference_input", 1, &ControllerBase::setReference, this);
}
}
