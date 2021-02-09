/*
 * =====================================================================================
 *
 *       Filename:  PushDisturbance.hpp
 *
 *    Description:  Class to handle disturbances as force on the base.
 *                  Code based from RSL published sources of 
 *                  "Learning Quadrupedal Locomotion Over Challenging Terrain"
 *
 *        Version:  1.0
 *        Created:  14/01/21 11:00:51
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Mathieu Geisert, 
 *   Organization:  
 *
 * =====================================================================================
 */

#pragma once

#include "common/message_macros.hpp"

//Better handle strength of disturbances using yaml file.

template<typename T>
class PushDisturbance {
public:
  PushDisturbance(raisim::ArticulatedSystem* anymal, RandomNumberGenerator<T>& rn)
  : anymal_(anymal), rn_(rn)
  {
    disturbance_.setZero();
  }
  
  ~PushDisturbance() = default;

  void sampleDisturbance(Eigen::Matrix<T, 3, 1> scaling, Eigen::Matrix<T, 3, 1> offset) 
  {
    disturbance_ << rn_.sampleUniform(), rn_.sampleUniform(), rn_.sampleUniform();
    disturbance_ = disturbance_.cwiseProduct(scaling) + offset;    
  }
  
  void sampleDisturbance(double scaling = 120.0) 
  {
    double az = M_PI * rn_.sampleUniform();
    double el = M_PI_2 * rn_.sampleUniform();
    disturbance_[0] = std::cos(el) * std::cos(az);
    disturbance_[1] = std::cos(el) * std::sin(az);
    disturbance_[2] = std::sin(el);

    disturbance_ *= scaling;
  }

  void setZero() { disturbance_.setZero(); }

  void applyDisturbance() {
      anymal_->setExternalForce(0,
                                disturbance_);
  }

  const Eigen::Matrix<T, 3, 1>& getDisturbance() const { return disturbance_; }
  void setDisturbance(Eigen::Matrix<T, 3, 1> disturbance) { disturbance_ = disturbance; }

protected:
  raisim::ArticulatedSystem* anymal_;
  RandomNumberGenerator<T>& rn_;
  Eigen::Matrix<T, 3, 1> disturbance_;
}; // end of class PushDisturbance

