#include "ds_util/fofonoff_depth.h"

#include <cmath>

namespace ds_util
{
//  Computes depth according to the Saunders and Fofonoff equation.  Taken from
//  pg 28 of Algorithms for Computation of Fundemental Properties of Seawater,
//  UNESCO 1983.
//
//  http://unesdoc.unesco.org/images/0005/000598/059832eb.pdf
double fofonoff_depth(double pressure_dbar, double latitude_deg) noexcept
{
  const auto x = pow(sin(latitude_deg / 57.29578), 2);
  const auto gravity = 9.780318 * (1.0 + (5.2788e-3 + (2.36e-5) * x) * x) + 1.092e-6 * pressure_dbar;

  auto depth =
      (((-1.82e-15 * pressure_dbar + 2.279e-10) * pressure_dbar - 2.2512e-5) * pressure_dbar + 9.72659) * pressure_dbar;

  depth /= gravity;

  return depth;
}
}
