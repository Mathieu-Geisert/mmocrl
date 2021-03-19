/*
 * =====================================================================================
 *
 *       Filename:  CommandViewer.hpp
 *
 *    Description:  Class to handle visualize commands in raisimUnity.
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
#include "environment/command/Command.hpp"
#include "environment/terrain/Terrain.hpp"
#include "common/math.hpp"

constexpr double velocityScale = 1.0;
constexpr double velocitySize = 0.2;
constexpr double goalSize = 0.4;

template<typename T, int Nlimb>
class CommandViewer {
public:
  CommandViewer(raisim::RaisimServer* server, const State<T>& state, const Command<T>& command, const Terrain<T, Nlimb>& terrain) 
  : server_(server), command_(command), state_(state), terrain_(terrain)
  {
    goalPositionVisual_ = server->addVisualSphere("goal_position", goalSize, 0., 1., 1.);
    velocityCommandVisual_ = server->addVisualCylinder("velocity_command", velocitySize, velocityScale, 0., 0., 1., 1.);
  }
  
  ~CommandViewer() = default;

  void advance()
  {
    //Update goalPosition
    const Eigen::Matrix<T, 3, 1>& goalPosition = command_.getGoalPosition();
    double z = terrain_.getHeight(goalPosition[0], goalPosition[1]);
    goalPositionVisual_->setPosition(goalPosition[0], goalPosition[1], z);

    //Update velocity command.
    Eigen::Matrix<T, 3, 1> command = command_.getCommand();
    double commandDirection = std::atan2(command[1], command[0]);
    double roll = 0.0, pitch = 1.57, yaw = commandDirection + state_.getHeadingAngle();
    Eigen::Vector4d quat;
    quat = Math::MathFunc::EulertoQuat<double>(roll, pitch, yaw);

    Eigen::Matrix<double, 3, 1> cylPosition = state_.getBasePos().template cast<double>();
    cylPosition[2] += 0.5;
    double length = velocityScale * command.head(2).norm();
    velocityCommandVisual_->setCylinderSize(velocitySize, length);
    command = state_.getRotationMatrix() * command;
    cylPosition[0] += command[0] * 0.5 * velocityScale;
    cylPosition[1] += command[1] * 0.5 * velocityScale;
    velocityCommandVisual_->setPosition(cylPosition);
    velocityCommandVisual_->setOrientation(quat);
  }
  

protected:
  raisim::RaisimServer* server_;
  const Command<T>& command_;
  const State<T>& state_;
  const Terrain<T, Nlimb>& terrain_;
  raisim::Visuals* goalPositionVisual_;
  raisim::Visuals* velocityCommandVisual_;
}; // end of class CommandViewer

