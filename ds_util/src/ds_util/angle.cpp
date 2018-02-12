#include "ds_util/angle.h"
#include <cmath>
namespace ds_util
{

double angular_separation_radians(double from, double to)
{

  // dot product produces the cosine of the included angle
  auto cDPSI = (std::sin(from)*std::sin(to) + std::cos(from)*std::cos(to));

  // cross product produces the sine of the included angle
  auto sDPSI = (std::cos(from)*std::sin(to) - std::sin(from)*std::cos(to));

  // Finally, take the arctan of the sine and cosine.  This gives us a signed
  // included angle.
  auto DPSI = std::atan2((sDPSI),(cDPSI));

  return DPSI;
}

}