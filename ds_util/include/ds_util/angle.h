#ifndef DS_UTIL_ANGLE_H
#define DS_UTIL_ANGLE_H

namespace ds_util
{
/// @brief Return the separation between two angles.
///
/// This function returns the difference between two angles.  The returned
/// value will always be within the interval [-pi, pi].  The sign of the
/// returned value indicates the direction of rotation:
///
///     angular_separation_radians(0, 20*M_PI/180) == 20*M_PI/180;
///     angular_separation_radians(20*M_PI/180, 0) == -20*M_PI/180;
///
/// \param from
/// \param to
/// \return
double angular_separation_radians(double from, double to);
}

#endif  // DS_UTIL_ANGLE_H
