#include "ds_base/reference_base.h"
#include "reference_base_private.h"

namespace ds_base
{
ReferenceBase::ReferenceBase()
  :DsProcess()
  , d_ptr_(std::unique_ptr<ReferenceBasePrivate>(new ReferenceBasePrivate))
{

}
ReferenceBase::ReferenceBase(int argc, char **argv, const std::string &name)
    : DsProcess(argc, argv, name)
    , d_ptr_(std::unique_ptr<ReferenceBasePrivate>(new ReferenceBasePrivate))
{

}

ReferenceBase::~ReferenceBase() = default;

void ReferenceBase::publishReference(const ds_nav_msgs::AggregatedState &reference)
{
  // Reference is not enabled, don't publish
  if(!enabled())
  {
    return;
  }

  DS_D(ReferenceBase);
  d->ref_pub_.publish(reference);
}

void ReferenceBase::setupPublishers() {
  DsProcess::setupPublishers();
  DS_D(ReferenceBase);
  const auto output_topic = ros::param::param<std::string>("~reference_output_topic", "reference_state");
  d->ref_pub_ = nodeHandle()->advertise<ds_nav_msgs::AggregatedState>(output_topic, 10, false);
}
void ReferenceBase::setEnabled(bool enabled)
{
  DS_D(ReferenceBase);
  d->is_enabled_ = enabled;

}
bool ReferenceBase::enabled() const noexcept {
  const DS_D(ReferenceBase);
  return d->is_enabled_;
}

void ReferenceBase::setupSubscriptions() {
  DsProcess::setupSubscriptions();
  auto nh = nodeHandle();
  DS_D(ReferenceBase);
  d->param_sub_ = ds_param::ParamConnection::create(*nh);
  d->active_reference_ = d->param_sub_->connect<ds_param::IntParam>("active_reference");
  d->param_sub_->setCallback(boost::bind(&ReferenceBase::parameterSubscriptionCallback, this, _1));
}

void ReferenceBase::setup() {
  DsProcess::setup();
  DS_D(ReferenceBase);
  setEnabled(d->active_reference_->Get() == type());
}

void ReferenceBase::parameterSubscriptionCallback(const ds_param::ParamConnection::ParamCollection &params)
{

  DS_D(ReferenceBase);
  for(auto it=std::begin(params); it != std::end(params); ++it)
  {
    if (*it == d->active_reference_) {
      setEnabled(d->active_reference_->Get() == type());
      break;
    }
  }
}

}
