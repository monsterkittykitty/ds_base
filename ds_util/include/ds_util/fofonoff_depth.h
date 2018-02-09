#ifndef DS_UTIL_FOFONOFF_DEPTH_H
#define DS_UTIL_FOFONOFF_DEPTH_H

namespace ds_util
{
///@brief Calculate depth from pressure using Fofonoff
///
/// Computes depth according to the Saunders and Fofonoff equation.  Taken from
/// pg 28 of Algorithms for Computation of Fundemental Properties of Seawater,
/// UNESCO 1983.
///
///  http://unesdoc.unesco.org/images/0005/000598/059832eb.pdf
///
/// \param pressure_dbar   Pressure in deci bars
/// \param latitude_deg    Latitude in degrees
/// \return  depth in meters
double fofonoff_depth(double pressure_dbar, double latitude_deg) noexcept;

}

#endif //DS_UTIL_FOFONOFF_DEPTH_H
