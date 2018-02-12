#ifndef DS_UTIL_REFERENCE_SMOOTHING_H
#define DS_UTIL_REFERENCE_SMOOTHING_H

#include <tuple>
#include "ros/time.h"

namespace ds_util
{

  /// @brief returns +-1, depending on the sign of the input argument
  double sgn(double x);

  /// @brief Creates a single-DOF smoothed trajectory between the current reference 
  ///        position and velocity to a goal position employing a trapezoidal 
  ///        velocity profile.
  /// \param[in] goal Current goal
  /// \param[in] ref_pos_in Input reference position
  /// \param[in] ref_vel_in Input reference velocity
  /// \param[in] ref_acc_in Input reference acceleration
  /// \param[in] max_vel Maximum velocity
  /// \param[in] max_acc Maximum acceleration
  /// \param[in] dt Timestep for trapezoidal smoothing
  /// \return A std::tuple containing smoothed reference position, velocity, and acceleration
  std::tuple<double, double, double> goal_trajectory_trapezoidal(double goal, double ref_pos_in, double ref_vel_in, double ref_acc_in, double max_vel, double max_acc, ros::Duration dt);

}

#endif //DS_UTIL_ANGLE_H
