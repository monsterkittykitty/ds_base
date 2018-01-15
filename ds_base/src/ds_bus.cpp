//
// Created by ivaughn on 1/15/18.
//

#include "ds_base/ds_bus.h"
#include "ds_bus_private.h"

using namespace ds_base;

// See EXTENDING.md
// Our constructors use the protected constructor from `DsProcess`, providing our
// own version of the private implementation class.
//
// This newly constructed DsBus::Impl gets implicitly upcast to DsProcess::Impl
// when passed to DsProcess's constructor.
//
// NOTE:  Our public constructors just forward on to our protected versions.  If
// we end up needing to add logic inside the constructors we'll only have to add
// it in two places now (the protected versions) instead of all four.
// Public default constructor:  use our own protected anolog
DsBus::DsBus() : DsBus(std::unique_ptr<Impl>(new Impl))
{
  // do nothing
}

// Another public->protected forwarding.
DsBus::DsBus(int argc, char* argv[], const std::string& name)
    :DsBus(std::unique_ptr<Impl>(new Impl), argc, argv, name)
{
  // do nothing
}

// Protected 'default' constructor
DsBus::DsBus(std::unique_ptr<Impl> impl) : ds_base::DsProcess(std::move(impl))
{
  // do nothing
}

// Protected constructor with arguments for ros::init
DsBus::DsBus(std::unique_ptr<Impl> impl, int argc, char* argv[], const std::string& name)
    : ds_base::DsProcess(std::move(impl), argc, argv, name)
{
  // do nothing
}

//
// This is how we get access to our new private DsBus::Impl.
// See, in the constructors above, we upcast DsBus::Impl into SensorBase::Impl, where
// it's stored in the SensorBase::impl_ member.
//
// To get the Impl class back *in the propper type* we need to downcast it again before
// working on it, which is why we have the static_cast<>'s here.
//
inline auto DsBus::d_func() noexcept -> DsBus::Impl * {
  return static_cast<DsBus::Impl *>(ds_base::DsProcess::d_func());
}

inline auto DsBus::d_func() const noexcept -> DsBus::Impl const * {
  return static_cast<DsBus::Impl const *>(ds_base::DsProcess::d_func());
}

