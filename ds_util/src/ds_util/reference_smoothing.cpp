#include "ds_util/reference_smoothing.h"

namespace ds_util
{
double sgn(double x)
{
  double result;
  result = 1.0;
  if (x < 0.0)
  {
    result = -1.0;
  }
  return (result);
}

std::tuple<double, double, double> goal_trajectory_trapezoidal(double goal, double ref_pos_in, double ref_vel_in,
                                                               double ref_acc_in, double max_vel, double max_acc,
                                                               ros::Duration dt)
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
  if (fabs(ref_vel) <= max_acc * ts && fabs(err) <= 0.5 * max_acc * ts * ts)
  {
    // At goal.
    ref_acc = 0;
    ref_vel = 0;
    ref_pos = goal;
  }
  else
  {
    // Compute the nominal desired velocity as a function of err.
    if (fabs(err) < 0.5 * max_acc * pow(max_vel / max_acc, 2))  // Decceleration phase.
    {
      vel_d = -sgn(err) * sqrt(2 * fabs(err) / max_acc) * max_acc;
    }
    else
    {
      vel_d = -sgn(err) * max_vel;
    }

    // But only allow accelerations of no more than max_acc.
    ref_acc = sgn(vel_d - ref_vel) * fmin(max_acc, fabs(vel_d - ref_vel) / ts);
  }

  return std::make_tuple(ref_pos, ref_vel, ref_acc);
}

void goal_trajectory(double goal, double max_velocity, double& position, double& velocity, double tau, double dt,
                     int smooth_ref)
{
  if (smooth_ref == 1)
  {
    goal_trajectory_button_smoother(goal, max_velocity, position, velocity, tau, dt);
  }
  else if (smooth_ref == 2)
  {
    // WARNING! This might be broken!
    goal_trajectory_acceleration(goal, max_velocity, position, velocity, tau, dt);
  }
  else
  {
    velocity = 0.0;
    position = goal;
  }
}

void goal_trajectory_button_smoother(double goal, double max_velocity, double& position, double& velocity, double tau,
                                     double dt)
{
  double dx;
  double last_position;
  double last_velocity;
  double alpha;
  double beta;

  alpha = exp(-dt / tau);
  beta = 1.0 - alpha;
  last_position = position;
  last_velocity = velocity;

  dx = goal - position;
  // find teh sign of dx
  // ----------------------------------------------------------------------
  //   25-MAY-2001 LLW&DAS  Hacked button auto to do step moves
  //                        #define SMOOTH_BUTTON_TRAJECTORY 1 to have smooth buttom moves
  // ----------------------------------------------------------------------

  velocity = alpha * last_velocity + beta * max_velocity * sgn(dx);

  if (fabs(dx / tau) > max_velocity)
  {
    position = goal - dx + velocity * dt;
  }
  else
  {
    position = alpha * position + beta * goal;
    dx = position - last_position;
    velocity = dx / dt;
  }
}

void goal_trajectory_acceleration(double goal, double max_velocity, double& position, double& velocity, double tau,
                                  double dt)
{
  double dx;
  double last_velocity;
  double acceleration_const;
  double acceleration;
  double tau_2;
  acceleration_const = 0.005;

  last_velocity = velocity;
  dx = goal - position;

  // define the acceleration of the vehicle based upon
  // the position from from the goal
  if (dx > 0)
  {
    acceleration = acceleration_const;
  }
  else if (dx < 0)
  {
    acceleration = -acceleration_const;
  }

  // now find change the current velocity based upon that calc

  velocity = last_velocity + acceleration * dt;
  // is that velocity greater then the max velocity if so then...
  if (fabs(velocity) > fabs(max_velocity))
  {
    velocity = max_velocity;
    acceleration = (max_velocity - last_velocity) / dt;
  }

  // WHAT IS THE MIN AMOUNT OF TIME TO SLOW DOWN
  tau_2 = velocity / acceleration_const;

  // CHECK TO SEE IF THE THAT IS GREATER THEN THE TIME IT TAKES TO THE
  // GOAL... if so then....
  if (fabs(dx / tau_2) > last_velocity)
  {
    position = goal - dx + last_velocity * dt + 0.5 * acceleration * dt * dt;
  }
  else  // if not ...
  {
    // RECHECK sign of dx and define accel constant
    if (dx > 0)
    {
      acceleration = -acceleration_const;
    }
    else if (dx < 0)
    {
      acceleration = acceleration_const;
    }

    // slow down hopefully get zero velocity at goal.
    velocity = last_velocity - acceleration * dt;
    // define desired position
    position = goal;

    // this is wrong...
    if (fabs(last_velocity) < fabs(acceleration * dt) || (velocity <= 0.0))
    {
      velocity = 0.0;
      position = goal;
    }
  }
}
}
