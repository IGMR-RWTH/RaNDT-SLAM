/** @file
 *  @brief NormalizeAngle function for Pose Graph Optimization
 *
 *  @date 23.07.2023
 *
 */

#ifndef STATE_MANIFOLD_H
#define STATE_MANIFOLD_H

#include <ceres/ceres.h>
#include <ceres/autodiff_manifold.h>

/** @brief normalize angle in range (-pi, pi]
    *  @param angle angle to be normalized
    */
template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
         two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}
#endif