/*
 * =====================================================================================
 *
 *       Filename:  PushDisturbancceViewer.hpp
 *
 *    Description:  Class to handle visualize push disturbance in raisimUnity.
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
#include "common/math.hpp"
#include "environment/observation/State.hpp"
#include "environment/disturbance/PushDisturbance.hpp"

constexpr double forceScale = 0.0075;
constexpr double forceSize = 0.1;

template<typename T>
class PushDisturbanceViewer {
public:
  PushDisturbanceViewer(raisim::RaisimServer* server, const State<T>* state, const PushDisturbance<T>* disturbance) 
  : server_(server), disturbance_(disturbance), state_(state)
  {
    forceDisturbanceVisual_ = server->addVisualCylinder("push_disturbance", forceSize, forceScale, 1., 0., 0., 0.);
  }
  
  ~PushDisturbanceViewer() = default;

  void advance()
  {
    //Update velocity command.
    Eigen::Matrix<T, 3, 1> disturbance = disturbance_->getDisturbance();

    if (disturbance.isZero(0)) {
      //Visual off (transparent) if no force
      forceDisturbanceVisual_->setColor(1., 0., 0., 0.);
    }
    else {
      forceDisturbanceVisual_->setColor(1., 0., 0., 1.);

      // --- update position/size
      
      //compute cylinder orientaiton.
      double norm = disturbance.norm();
      double el = std::asin(disturbance[2]/norm);
      double az = std::acos(disturbance[0] / (norm * std::cos(el)));
      if (disturbance[1]<0.0) { az *= -1.; }

      double roll = 0.0, pitch = 1.57 - el, yaw = az;
      Eigen::Vector4d quat;
      quat = Math::MathFunc::EulertoQuat<double>(roll, pitch, yaw);
      Eigen::Matrix<double, 3, 1> cylPosition = state_->getBasePos().template cast<double>();
      cylPosition[2] += 0.0;
      double length = forceScale * norm;
      forceDisturbanceVisual_->setCylinderSize(forceSize, length);
      cylPosition[0] += disturbance[0] * 0.5 * forceScale;
      cylPosition[1] += disturbance[1] * 0.5 * forceScale;
      cylPosition[2] += disturbance[2] * 0.5 * forceScale;
      forceDisturbanceVisual_->setPosition(cylPosition);
      forceDisturbanceVisual_->setOrientation(quat);
    }
  }
  

protected:
  raisim::RaisimServer* server_;
  const PushDisturbance<T>* disturbance_;
  const State<T>* state_;
  raisim::Visuals* forceDisturbanceVisual_;
}; // end of class CommandViewer

