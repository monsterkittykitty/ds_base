#include "ds_util/thruster.h"
#include <cmath>
#include <limits>

namespace ds_util
{

double linear_force_to_current(double alpha, double beta, double force, double speed)
{

  // Make sure we're non-negative.
  speed = std::abs(speed);

  const auto factor = alpha + beta * speed;
  if (factor == 0)
  {
    return force > 0 ? std::numeric_limits<double>::infinity() : - std::numeric_limits<double>::infinity();
  }

  return force / factor;
}

double linear_current_to_force(double alpha, double beta, double current, double speed)
{
  // Make sure we're non-negative.
  speed = std::abs(speed);

  const auto factor = (alpha + beta * speed);
  return factor * current;
}

}
