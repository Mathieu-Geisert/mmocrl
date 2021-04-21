/*
 * =====================================================================================
 *
 *       Filename:  Command.hpp
 *
 *    Description:  Class to handle command input.
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
#include "environment/observation/State.hpp"
//#include "environment/terrain/Terrain.hpp"
#include "common/math.hpp"

enum CommandMode {
  RANDOM = 0,
  FIXED_DIR,
  STRAIGHT,
  STOP,
  ZERO,
  NOZERO
};

template<typename T>//, int Nlimb>
class Command {
public:
  //Command(const State<T>& state, const terrain::Terrain<T, Nlimb>& terrain,  RandomNumberGenerator<T>& rn)
  Command(const State<T>* state,  RandomNumberGenerator<T>* rn)
  : state_(state), 
    //terrain_(terrain),
    rn_(rn)
  {
    goalPosition_.setZero();
    command_.setZero();
    commandMode_ = CommandMode::RANDOM;
  }
  
  ~Command() = default;
  
  void updateCommand() {
    if (commandMode_ == CommandMode::FIXED_DIR) {
      return;
    }
    //if (command_.head(2).norm() != 0.0) {
      Eigen::Matrix<T, 3, 1> bodyPos = state_->getBasePos();
      T x_dist = goalPosition_[0] - bodyPos[0];
      T y_dist = goalPosition_[1] - bodyPos[1];
      if (std::sqrt(x_dist * x_dist + y_dist * y_dist) < 0.5) {
        sampleGoal();
      }
      T commandDirection = std::atan2(y_dist, x_dist);
      const T& headingAngle = state_->getHeadingAngle();
      T commandDirection_bodyframe = commandDirection - headingAngle;
      commandDirection_bodyframe = Math::MathFunc::anglemod(commandDirection_bodyframe);
      command_[0] = std::cos(commandDirection_bodyframe);
      command_[1] = std::sin(commandDirection_bodyframe);
      //command_[2] = commandDirection_bodyframe / 5.;
    //}

//    if (stopMode_) {
//      baseFreq_ = 0.0;
//    } else {
//      baseFreq_ = 1.3 * freqScale_;
//    }
  }
  
  inline void sampleGoal() {
//    goalPosition_[0] = q_[0];
//    goalPosition_[1] = q_[1];
//
//    if (terrainType_ == TerrainType::Stairs) {
//      goalPosition_[0] += terrainProp_.xSize * 0.1 * rn_->sampleUniform();
//      if (q_[1] < 0.3) {
//        goalPosition_[1] = terrainProp_.ySize * (0.3 + 0.2 * rn_->sampleUniform01());
//      } else {
//        goalPosition_[1] = -terrainProp_.ySize * (0.3 + 0.2 * rn_->sampleUniform01());
//      }
//    } else {
//      goalPosition_[0] += terrainProp_.xSize * 0.4 * rn_->sampleUniform();
//      goalPosition_[1] += terrainProp_.ySize * 0.4 * rn_->sampleUniform();
//
//      if (commandMode_ == CommandMode::STRAIGHT) {
//        goalPosition_[0] = q_[0] + terrainProp_.xSize * 0.1 * rn_->sampleUniform();
//        goalPosition_[1] = q_[1] + terrainProp_.ySize * 0.4;
//      }
//    }
//
//    goalPosition_[0] = std::min(0.5 * terrainProp_.xSize - 1.0, goalPosition_[0]);
//    goalPosition_[0] = std::max(-0.5 * terrainProp_.xSize + 1.0, goalPosition_[0]);
//    goalPosition_[1] = std::min(0.5 * terrainProp_.ySize - 1.0, goalPosition_[1]);
//    goalPosition_[1] = std::max(-0.5 * terrainProp_.ySize + 1.0, goalPosition_[1]);
    goalPosition_[0] = 5. * rn_->sampleUniform();
    goalPosition_[1] = 5. * rn_->sampleUniform();
  }

  void sampleCommand() {
    if (commandMode_ == CommandMode::FIXED_DIR) {
      goalPosition_[0] = 0.0;
      goalPosition_[1] = 0.0;
      command_ << 1.0, 0.0, 0.0;
      return;
    } else if (commandMode_ == CommandMode::ZERO) {
      command_.setZero();
      goalPosition_[0] = 10.0;
      goalPosition_[1] = 10.0;
      return;
    }
    sampleGoal();
    Eigen::Matrix<T, 3, 1> bodyPos = state_->getBasePos();
    double commandDirection = std::atan2(goalPosition_[1] - bodyPos[1], goalPosition_[0] - bodyPos[0]);

    double headingAngle = state_->getHeadingAngle();
    double commandDirection_bodyframe = commandDirection - headingAngle;
    commandDirection_bodyframe = Math::MathFunc::anglemod(commandDirection_bodyframe);

    command_[0] = std::cos(commandDirection_bodyframe);
    command_[1] = std::sin(commandDirection_bodyframe);
    command_[2] = 0.0;

//    if (commandMode_ != CommandMode::STRAIGHT) {
//      command_[2] = 1.0 - 2.0 * rn_->intRand(0, 1);
//      command_[2] *= rn_->sampleUniform01();
//      if ((commandMode_ != CommandMode::NOZERO) && (rn_->sampleUniform01() > 0.8)) {
//        command_.head(2).setZero();
//      }
//    }

//    if (terrainType_ == TerrainType::Stairs) {
//      if (rn_->sampleUniform01() < 0.5) {
//        command_[2] = 0.0;
//      }
//    }
  }

  void setCommandMode(CommandMode commandMode) { commandMode_ = commandMode; }
  const CommandMode& getCommandMode() const { return commandMode_; }

  const Eigen::Matrix<T, 3, 1>& getCommand() const { return command_; }
  void setCommand(const Eigen::Matrix<T, 3, 1>& command) { command_ = command; }
  const Eigen::Matrix<T, 3, 1>& getGoalPosition() const { return goalPosition_; }
  void setGoalPosition(const Eigen::Matrix<T, 3, 1>& goalPosition) { goalPosition_ = goalPosition; }

protected:
  const State<T>* state_;
  //const terrain::Terrain<T, Nlimb>& terrain_;
  RandomNumberGenerator<T>* rn_;

  CommandMode commandMode_;
  Eigen::Matrix<T, 3, 1> command_;
  Eigen::Matrix<T, 3, 1> goalPosition_;
}; // end of class Command

