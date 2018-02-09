//
// Created by zac on 2/9/18.
//

#ifndef DS_UTIL_THRUSTER_H
#define DS_UTIL_THRUSTER_H

namespace ds_util
{


/// @brief Calculate the required current to produce the desired force
///
/// The current is calculated using:
///
/// I = F / (alpha + beta * speed)
///
/// where
///   - I: is the required current
///   - F: is the desired force
///   - speed:  is the vehicle speed (non-negative!)
///   - alpha, beta: emperically derived fitting coeficients.
///
/// NOTE:  This transfer function has a pole at alpha = -beta*speed!
///        Divide by zero's will return infinity
///
/// NOTE:  This function assumes the current and force signs are the same!
///
/// \param alpha
/// \param beta
/// \param force
/// \param speed
/// \return  current
double linear_force_to_current(double alpha, double beta, double force, double speed);


/// @brief Transform from thruster current to approximate force
///
/// This method is the inverse of linear_force_to_current
///
/// \param alpha
/// \param beta
/// \param current
/// \param speed
/// \return force
double linear_current_to_force(double alpha, double beta, double current, double speed);

}

#endif //DS_UTIL_THRUSTER_H
