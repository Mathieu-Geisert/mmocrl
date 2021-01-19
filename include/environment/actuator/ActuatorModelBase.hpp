/*
 * =====================================================================================
 *
 *       Filename:  ActuatorModelBase.hpp
 *
 *    Description:  Base class for Actuator Models.
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

#include <raisim/object/ArticulatedSystem/ArticulatedSystem.hpp>
#include "common/message_macros.hpp"

template<typename T>
class ActuatorModelBase {
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  public:
    ActuatorModelBase(raisim::ArticulatedSystem *anymal)
    : anymal_(anymal), nDOF_(anymal->getDOF()), nJoint_(nDOF_ - 6)
    {
      targetJointPosition_.setZero(nJoint_);
      targetJointVelocity_.setZero(nJoint_);
      feedForwardTorque_.setZero(nJoint_);
    }

    ~ActuatorModelBase() = default;

    virtual void setTargetJointPosition(const Eigen::Matrix<T, -1, 1>& jointPosition)
    {
      FATAL_IF(jointPosition.size() != nJoint_, "jointPosition should have the  size of the number of joints.");
      targetJointPosition_ = jointPosition;
    }

    virtual void setTargetJointVelocity(const Eigen::Matrix<T, -1, 1>& jointVelocity)
    {
      FATAL_IF(jointVelocity.size() != nJoint_, "jointPosition should have the  size of the number of joints.");
      targetJointVelocity_ = jointVelocity;
    }

    virtual void setTargetJointTorque(const Eigen::Matrix<T, -1, 1>& feedForwardTorque)
    {
      FATAL_IF(feedForwardTorque.size() != nJoint_, "jointPosition should have the  size of the number of joints.");
      feedForwardTorque_ = feedForwardTorque;
    }

    virtual void reset() 
    {
      targetJointPosition_.setZero();
      targetJointVelocity_.setZero();
      feedForwardTorque_.setZero();
    }
    
    virtual void advance() = 0;

  protected:
    raisim::ArticulatedSystem *anymal_;
    int nDOF_, nJoint_;

    Eigen::Matrix<T, -1, 1> targetJointPosition_;
    Eigen::Matrix<T, -1, 1> targetJointVelocity_;
    Eigen::Matrix<T, -1, 1> feedForwardTorque_;
};
