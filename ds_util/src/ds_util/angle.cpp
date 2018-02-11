#include "ds_util/angle.h"
#include <cmath>
namespace ds_util
{

double angular_separation_radians(double from, double to)
{

  // Move angles into range [0, 2pi)
  from = std::fmod(from, 2*M_PI);
  to = std::fmod(to, 2*M_PI);

  // Move from into range (-pi, pi]
  while (from >= M_PI)
  {
    from -= 2 * M_PI;
  }

  while (from < -M_PI)
  {
    from += 2 * M_PI;
  }

  // Move 'to' into range (-pi, pi]
  while (to >= M_PI)
  {
    to -= 2 * M_PI;
  }

  while (to < -M_PI)
  {
    to += 2 * M_PI;
  }

  const auto sep = std::acos(std::cos(from)*std::cos(to) + std::sin(from)*std::sin(to));
  return to > from ? sep: -sep;
}

}