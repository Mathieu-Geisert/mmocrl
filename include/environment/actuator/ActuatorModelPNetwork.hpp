/*
 * =====================================================================================
 *
 *       Filename:  ActuatorModelPNetwork.hpp
 *
 *    Description:  Actuator model using Neural Network of Anymal actuator controlled with position only.
 *                  Network structure/params taken from RSL published sources of 
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

#include "environment/actuator/ActuatorModelBase.hpp"
#include "environment/history/JointHistory.hpp"
#include "common/SimpleMLPLayer.hpp"

//TODO: Handle History not created in the Actuator class to reduce copies of the same data.
//TODO: simulation dt should be 0.0025. Add safeguard.

namespace PNetwork {
  
  constexpr int JointHistoryLength = 10;  //Minimum NNHistory1
  constexpr int NJoint = 12;

  constexpr int NNInputSize = 6;

  constexpr int NNHistory1 = 9;
  constexpr int NNHistory2 = 4;
  constexpr int NNHistory3 = 1;

  constexpr double VelScaling = 0.4;
  constexpr double PosScaling = 3.0;
  constexpr double TorScaling = 20.0;
} // end of namespace PNetwork

template<typename T>
class ActuatorModelPNetwork : public ActuatorModelBase<T> {
  
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    ActuatorModelPNetwork(raisim::ArticulatedSystem* anymal, std::string actuator_path) 
    : ActuatorModelBase<T>(anymal),
      jointHist_(anymal), 
      actuator_A_({32, 32})
    {
      actuator_A_.updateParamFromTxt(actuator_path);
      this->anymal_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    }

    ~ActuatorModelPNetwork() = default;

    virtual void reset()
    {
      jointHist_.reset();
    }

    virtual void advance() 
    {
      Eigen::Matrix<T, -1, 1> tau(this->nDOF_);
      tau.setZero();
      //anymal_->getGeneralizedForce(tau);

      jointHist_.appendCurrentState(this->targetJointPosition_);
      Eigen::Matrix<T, PNetwork::NNInputSize, 1> seaInput;

      for (int actId = 0; actId < PNetwork::NJoint; actId++) {
        seaInput[0] = jointHist_.getLastJointVelocity(actId, PNetwork::NNHistory1) * PNetwork::VelScaling;
        seaInput[1] = jointHist_.getLastJointVelocity(actId, PNetwork::NNHistory2) * PNetwork::VelScaling;
        seaInput[2] = jointHist_.getLastJointVelocity(actId, PNetwork::NNHistory3) * PNetwork::VelScaling;
        seaInput[3] = jointHist_.getLastJointPositionError(actId, PNetwork::NNHistory1) * PNetwork::PosScaling;
        seaInput[4] = jointHist_.getLastJointPositionError(actId, PNetwork::NNHistory2) * PNetwork::PosScaling;
        seaInput[5] = jointHist_.getLastJointPositionError(actId, PNetwork::NNHistory3) * PNetwork::PosScaling;
        tau(6 + actId) = actuator_A_.forward(seaInput)[0] * PNetwork::TorScaling;
      }

      this->anymal_->setGeneralizedForce(tau.template cast<double>());
    } 

  protected:
    rai::FuncApprox::MLP_fullyconnected<T, PNetwork::NNInputSize, 1, rai::FuncApprox::ActivationType::softsign> actuator_A_;
    JointVelPosErrorHistory<T, PNetwork::JointHistoryLength, PNetwork::NJoint> jointHist_;
}; // end of class ActuatorModelPNetwork
