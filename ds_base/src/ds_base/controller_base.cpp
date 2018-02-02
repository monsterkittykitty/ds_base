#include "ds_base/controller_base.h"
#include "controller_base_private.h"
#include "../../../ds_param/include/ds_param/ds_param_conn.h"

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
  const auto state_input_topic = ros::param::param<std::string>("~state_input_topic", "estimated_state");
  const auto ref_input_topic = ros::param::param<std::string>("~reference_input_topic", "reference_state");

  auto nh = nodeHandle();
  d->state_update_sub_ = nh->subscribe(state_input_topic, 1, &ControllerBase::setState, this);
  d->reference_update_sub_ = nh->subscribe(ref_input_topic, 1, &ControllerBase::setReference, this);

  d->param_sub_ = ds_param::ParamConnection::create(*nh);
  d->active_controller_ = d->param_sub_->connect<ds_param::IntParam>("active_controller");
  d->param_sub_->setCallback(boost::bind(&ControllerBase::parameterSubscriptionCallback, this, _1));
}
void ControllerBase::setup() {
  DsProcess::setup();
  DS_D(ControllerBase);
  setEnabled(d->active_controller_->Get() == type());
}

void ControllerBase::parameterSubscriptionCallback(const ds_param::ParamConnection::ParamCollection &params)
{

  DS_D(ControllerBase);
  for(auto it=std::begin(params); it != std::end(params); ++it)
  {
    if (*it == d->active_controller_) {
      setEnabled(d->active_controller_->Get() == type());
      break;
    }
  }
}
}
