/*
 * =====================================================================================
 *
 *       Filename:  InternalObservation.hpp
 *
 *    Description:  Class to handle common observation (=state, foottraj, actuator hist).
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
#include "environment/motion/FootMotionGenerator.hpp"
#include "environment/actuator/ActuatorModelPNetwork.hpp"
#include "environment/scaling/ScalingAndOffset.hpp"
#include "environment/command/Command.hpp"

//TODO: avoid hardcoding control_dt/simulation_dt for jointTargethistory.

template<typename T, int Nlimb>
class InternalObservation
{
  public:
    InternalObservation(const State<T>* state, const Command<T>* command, const FootMotionGenerator<T, Nlimb>* footMotion, const ActuatorModelPNetwork<T>* actuator)
     : state_(state), footMotion_(footMotion), actuator_(actuator), command_(command)
    {
//      Eigen::Matrix<T, 12, 1> jointNominalConfig;
//      Eigen::Matrix<T, , 1> stateOffset, stateScale;
//      double freqScale = 0.0025 * 2.0 * M_PI;
//
//      jointNominalConfig << -0.138589, 0.480936, -0.761428, 0.138589, 0.480936, -0.761428, -0.138589, -0.480936, 0.761428, 0.138589, -0.480936, 0.761428;
//      
//      stateOffset << 0.0, 0.0, 0.0, /// command
//          0.0, 0.0, 1.0, /// gravity axis
//          Eigen::VectorXf::Constant(6, 0.0), /// body lin/ang vel
//          jointNominalConfig, /// joint position
//          Eigen::VectorXf::Constant(12, 0.0),
//          Eigen::VectorXf::Constant(12, 0.0),
//          Eigen::VectorXf::Constant(4, 0.0), //52
//          Eigen::VectorXf::Constant(8, 0.0), // 60
//          Eigen::VectorXf::Constant(24, 0.0), // 84
//          Eigen::VectorXf::Constant(24, 0.0), // 108
//          jointNominalConfig, /// joint position
//          jointNominalConfig, /// joint position
//          0.0; // 132
//  
//      stateScale << 1.5, 1.5, 1.5, /// command
//          5.0, 5.0, 5.0, /// gravity axis
//          Eigen::VectorXf::Constant(3, 2.0),
//          Eigen::VectorXf::Constant(3, 2.0),
//          Eigen::VectorXf::Constant(12, 2.0), /// joint angles
//          0.5, 0.4, 0.3,
//          0.5, 0.4, 0.3,
//          0.5, 0.4, 0.3,
//          0.5, 0.4, 0.3,
//          6.5, 4.5, 3.5,
//          6.5, 4.5, 3.5,
//          6.5, 4.5, 3.5,
//          6.5, 4.5, 3.5,
//          Eigen::VectorXf::Constant(4, 2.0 / freqScale),
//          Eigen::VectorXf::Constant(8, 1.5),
//          Eigen::VectorXf::Constant(24, 5.0), /// joint position errors
//          0.5, 0.4, 0.3,
//          0.5, 0.4, 0.3,
//          0.5, 0.4, 0.3,
//          0.5, 0.4, 0.3,
//          0.5, 0.4, 0.3,
//          0.5, 0.4, 0.3,
//          0.5, 0.4, 0.3,
//          0.5, 0.4, 0.3,
//          Eigen::VectorXf::Constant(12, 2.0), /// prev. action
//          Eigen::VectorXf::Constant(12, 2.0),
//          2.0 / freqScale;
//
//      scaling_.setOffset(stateOffset);
//      scaling_.setScaling(staeScaling);
    }

//    InternalObservation(const State<T>& state, const FootMotionGenerator<T, Nlimb> footMotion, const ActuatorModelPNetwork<T>& actuator, std::string pathToScalingAndOffset)
//     : state_(state), footMotion_(footMotion), actuator_(actuator), scaling_(pathToScalingAndOffset)
//    {
//    }

    Eigen::Matrix<T,133, 1> getObservation()
    {
      Eigen::Matrix<T, 133, 1> observation;

      //command
      observation.template head<3>() = command_->getCommand();
     
     //robot state 
      observation.template segment<3>(3) = state_->getGravityAxis();
      observation.template segment<3>(6) = state_->getBaseVelInBaseFrame();
      observation.template segment<3>(9) = state_->getBaseAngVelInBaseFrame();
      observation.template segment<12>(12) = state_->getJointPos();
      observation.template segment<12>(24) = state_->getJointVel();

      //current jointError
      observation.template segment<12>(36) = actuator_->getHistory().getLastJointPositionError();

      //footmotion
      observation.template segment<4>(48) = footMotion_->getFrequencies();
      Eigen::Matrix<T, 4, 1> phases = footMotion_->getPhases();
      for (size_t i = 0; i < 4; i++) {
        observation[52 + 2 * i] = std::sin(phases[i]);
        observation[52 + 2 * i + 1] = std::cos(phases[i]);
      }

      //actuator history
      observation.template segment<12>(60) = actuator_->getHistory().getLastJointPositionError(PNetwork::NNHistory2);
      observation.template segment<12>(72) = actuator_->getHistory().getLastJointPositionError(PNetwork::NNHistory1);
     
      observation.template segment<12>(84) = actuator_->getHistory().getLastJointVelocity(PNetwork::NNHistory2);
      observation.template segment<12>(96) = actuator_->getHistory().getLastJointVelocity(PNetwork::NNHistory1);

      observation.template segment<12>(108) = actuator_->getHistory().getLastJointPositionTarget();
      observation.template segment<12>(120) = actuator_->getHistory().getLastJointPositionTarget(4); // TODO: avoid hardcoding history = control_dt/simulation_dt

      observation[132] = footMotion_->getBaseFrequency();

      return observation;
      //return scaling_.apply(observation);
    }

  protected:
    //ScalingAndOffset scaling_;
    const State<T>* state_;
    const FootMotionGenerator<T, Nlimb>* footMotion_;
    const ActuatorModelPNetwork<T>* actuator_;
    const Command<T>* command_;
}; // end of class InternalObservation

