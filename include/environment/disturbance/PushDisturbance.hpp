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

//TODO: Better handle angles for the viewer??? 

template<typename T>
class PushDisturbance {
public:
  PushDisturbance(raisim::ArticulatedSystem* anymal, RandomNumberGenerator<T>& rn, double simulation_dt = 0.0025)
  : anymal_(anymal), rn_(rn), simulation_dt_(simulation_dt), duration_max_step_(0), duration_current_step_(0)
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
  
  //To be executed at simulation dt
  void advance(double probability = 0.3, double duration = 2.0, double scaling = 120.0)
  {
    duration_current_step_++;
    if (duration_current_step_ < duration_max_step_) {
      //Continue with the current disturbance.
    }
    else {
      //Update disturbance.
      duration_current_step_ = 0;
      duration_max_step_ = int(duration / simulation_dt_);
      
      //add disturbance only with a certain paobability
      if ( rn_.sampleUniform01() < probability ) {
        sampleDisturbance(scaling);
      }
      else {
        setZero();
      }
    }

    applyDisturbance();
  }

  const Eigen::Matrix<T, 3, 1>& getDisturbance() const { return disturbance_; }
  void setDisturbance(Eigen::Matrix<T, 3, 1> disturbance) { disturbance_ = disturbance; }

protected:
  raisim::ArticulatedSystem* anymal_;
  RandomNumberGenerator<T>& rn_;
  Eigen::Matrix<T, 3, 1> disturbance_;
  double simulation_dt_;
  int duration_max_step_;
  int duration_current_step_;
}; // end of class PushDisturbance

