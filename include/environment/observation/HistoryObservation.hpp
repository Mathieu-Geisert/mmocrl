/*
 * =====================================================================================
 *
 *       Filename:  HistoryObservation.hpp
 *
 *    Description:  Class to handle history observation (=observation for the temporal convolutional network)).
 *                  Code based from RSL published sources of 
 *                  "Learning Quadrupedal Locomotion Over Challenging Terrain"
 *
 *        Version:  1.0
 *        Created:  12/07/21 10:00:51
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

template<typename T, int Nlimb, int historyLength>
class HistoryObservation
{
  public:
    HistoryObservation(const State<T>* state, const Command<T>* command, const FootMotionGenerator<T, Nlimb>* footMotion, const ActuatorModelPNetwork<T>* actuator, const ScalingAndOffset<T>* scaling)
     : state_(state), footMotion_(footMotion), actuator_(actuator), command_(command), scaling_(scaling), hist_()
    {
    }

    ~HistoryObservation() = default;

    void advance()
    {
      Eigen::Matrix<T, 60, 1> observation;

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

      //scaling
      observation = scaling_->apply(observation);

      //add to history
      Eigen::Matrix<T, 60*(historyLength-1), 1> temp = hist_.head(60*(historyLength - 1));
      hist_.tail(60*(historyLength - 1)) = temp;//hist_.head(60*(historyLength - 1));
      hist_.head(60) = observation;
    }

    void reset()
    {
       for (int i=0; i<historyLength; i++) {
         advance();
       }
    }

    const Eigen::Matrix<T,60*historyLength, 1>& getObservation() const
    {
      return hist_;
    }

  protected:
    const State<T>* state_;
    const FootMotionGenerator<T, Nlimb>* footMotion_;
    const ActuatorModelPNetwork<T>* actuator_;
    const Command<T>* command_;
    const ScalingAndOffset<T>* scaling_;

    //use normal buffer since the whole history needs to be ordered.
    //JointHistory<T, historyLength, 60> hist_;
    Eigen::Matrix<T, historyLength* 60, 1> hist_; 

}; // end of class InternalObservation

