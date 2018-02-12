#include "ds_util/reference_smoothing.h"

namespace ds_util
{

  double sgn(double x)
  {
    double result;
    result = 1.0;
    if (x < 0.0)
      result = -1.0;
    return (result);
  }

  std::tuple<double, double, double> goal_trajectory_trapezoidal(double goal, double ref_pos_in, double ref_vel_in, double ref_acc_in, double max_vel, double max_acc, ros::Duration dt)
  {
    double err, vel_d;
    double ref_pos = ref_pos_in;
    double ref_vel = ref_vel_in;
    double ref_acc = ref_acc_in;
    double ts = dt.toSec();

    // Update current reference.
    ref_vel = ref_vel + ref_acc * ts;
    ref_pos = ref_pos + ref_vel * ts + 0.5 * ref_acc * ts * ts;
  
    // Compute current vector to goal.
    err = ref_pos - goal;
  
    // Determine acceleration to apply.
    if ( fabs(ref_vel) <= max_acc * ts && fabs(err) <= 0.5 * max_acc * ts * ts )
      {
	// At goal.
	ref_acc = 0;
	ref_vel = 0;
	ref_pos = goal;
      }
    else 
      {
	// Compute the nominal desired velocity as a function of err.
	if ( fabs(err) < 0.5 * max_acc * pow( max_vel / max_acc, 2 ) ) // Decceleration phase.
	  vel_d = -sgn(err) * sqrt( 2 * fabs(err) / max_acc ) * max_acc;
	else
	  vel_d = -sgn(err) * max_vel;

	// But only allow accelerations of no more than max_acc.
	ref_acc = sgn( vel_d - ref_vel ) * fmin( max_acc, fabs(vel_d - ref_vel) / ts );
      }

    return std::make_tuple(ref_pos, ref_vel, ref_acc);
  }

}
