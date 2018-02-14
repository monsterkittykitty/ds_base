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


/// \brief Computes a trajectory from the current position and velocity to goal, without exceeding max_velocity.
///
/// Velocities are filtered with time constant Tau
///
/// From reference.cpp in rov.  Here's the original modification history:
///
/// ======================================================================
///
///   function goal_trajectory
///   Computes a trajectory from the current position and velocity
///   to goal, without exeeding max_velocity.  Velocities are
///   filtered with the time constant tau
///
/// Modification History:
///   DATE         AUTHOR  COMMENT
///   ???          DRY     Created and written
///   26-JUL-2000  LLW     Ported from DRY's original in J1.
///   14-OCT-2005  SCM     added acceleration term
///   22-OCT-2005  SCM     fixed bugs
///   14-FEB-2018  JISV    Ported to ROS and broke up into
///                        goal_trajectory_button_smoother and goal_trajectory_acceleration
///   ====================================================================== */
///
/// \param goal The goal to shoot for
/// \param max_velocity  The max velocity to use
/// \param position  The current position for the value we're moving
/// \param velocity The current velocity
/// \param tau The smoothing timecosntant parameter
/// \param dt The timestep
/// \param smooth_ref The smoother type.  1 for "button smoother", 2 for "goal_trajectory_acceleration", anything else for "none"
void goal_trajectory(double goal, double max_velocity, double& position, double& velocity, double tau, double dt, int smooth_ref);

/// \brief Smooths a reference (velocity + position) using the simple legacy max-velocity smoother
///
/// \param goal  The goal to shoot for
/// \param max_velocity  The max velocity to head that way
/// \param position  The current reference (part of reference state)
/// \param velocity  The derivative of the current reference (part of reference state)
/// \param tau The smoothing time constant
/// \param dt  The timestep to advance
void goal_trajectory_button_smoother(double goal, double max_velocity, double& position, double& velocity, double tau, double dt);

/// \brief Smooths a reference (velocity + position) using the possibly-broken constant acceleration smoother
///
/// \param goal  The goal to shoot for
/// \param max_velocity  The max velocity to head that way
/// \param position  The current reference (part of reference state)
/// \param velocity  The derivative of the current reference (part of reference state)
/// \param tau The smoothing time constant
/// \param dt  The timestep to advance
void goal_trajectory_acceleration(double goal, double max_velocity, double& position, double& velocity, double tau, double dt);

}

#endif //DS_UTIL_REFERENCE_SMOOTHING_H
