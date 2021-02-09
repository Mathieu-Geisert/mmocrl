/*
 * =====================================================================================
 *
 *       Filename:  JointHistory.hpp
 *
 *    Description:  A common class to organize History.
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

template <typename T, int jointHistoryLength, int nJoint>
class JointHistory {
 
 //Implementation of History as a circular buffer to avoid useless copies.
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    JointHistory() 
    {
      reset(); 
    }
    ~JointHistory() = default;
   
    void reset(Eigen::Matrix<T, -1, 1> jointValue)
    {
      FATAL_IF(jointValue.size()<nJoint, "[JointHistory::reset] jointValue size should be at least _nJoint.");
      currentBufferPosition_ = 0;
      for (int i = 0; i < jointHistoryLength; i++) {
        jointHist_.segment(i * nJoint, nJoint) = jointValue.tail(nJoint);
      }
    }
    
    void reset()
    {
      currentBufferPosition_ = 0;
      jointHist_.setZero();
    }
    
    int getNJoint() const { return nJoint; } 
    int getHistoryLength() const { return jointHistoryLength; }
    
    T getLastJointValue(int actId, int iter) const 
    {
      FATAL_IF(actId<0 || actId>nJoint-1, "[JointHistory::getLastJointPosition] wrong actuator Id...");
      return jointHist_(actId + getPositionInBuffer(iter));
    }
    
    Eigen::Matrix<T, nJoint, 1> getLastJointValue(int iter = 1) const
    {
      //std::cout << "currentBufferPosition: " << currentBufferPosition_  << " -- iter: " << iter << " -- getPositionInBuffer: " << getPositionInBuffer(iter) << std::endl;
      return jointHist_.segment(getPositionInBuffer(iter), nJoint);
    }
    
    void appendValue(const Eigen::Matrix<T, -1, 1>& jointValue)
    {
      FATAL_IF(jointValue.size()<nJoint, "[JointHistory::reset] jointValue size should be at least _nJoint.");
      jointHist_.segment(currentBufferPosition_ * nJoint, nJoint) = jointValue.tail(nJoint);
      //std::cout << "jointHist: \n" << jointHist_ << std::endl;
      currentBufferPosition_++;
      if (currentBufferPosition_ >= jointHistoryLength) {
        currentBufferPosition_ = 0;
      }
    }

  protected:

    inline int getPositionInBuffer(int pastIter) const
    {
      FATAL_IF(pastIter<1 || pastIter>jointHistoryLength-1, "[JointHistory::getLastJointVelocity] iteration should be greater 1 and smaller than the history length.");
      int delta = currentBufferPosition_ - pastIter;
      if (delta >= 0)
        return delta * nJoint;
      else
        return (jointHistoryLength + delta) * nJoint;
    }

    int currentBufferPosition_;
    Eigen::Matrix<T, nJoint * jointHistoryLength, 1> jointHist_;
};


template <typename T, int jointHistoryLength, int nJoint>
class JointVelPosErrorHistory {

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  public:
    JointVelPosErrorHistory(raisim::ArticulatedSystem* anymal)
     : anymal_(anymal), jointVelHist_(), jointPosErrorHist_(), jointPosTargetHist_()
    {
      reset();
    }

    ~JointVelPosErrorHistory() = default;

    void reset()
    {
      //if (isnan(anymal_->getGeneralizedVelocity().e().norm())) {
      //  std::cout << "history: velocity is nan" << std::endl; 
      //}
      //if (isnan(anymal_->getGeneralizedCoordinate().e().norm())) {
      //  std::cout << "history: coordinate is nan" << std::endl; 
      //}
      jointVelHist_.reset(anymal_->getGeneralizedVelocity().e().template cast<T>());
      jointPosErrorHist_.reset();
      jointPosTargetHist_.reset(anymal_->getGeneralizedCoordinate().e().template cast<T>());
    }

    int getNJoint() const { return nJoint; }
    int getHistoryLength() const { return jointHistoryLength; }

    void appendCurrentState(const Eigen::Matrix<T, -1, 1>& targetJointPosition)
    {
      //if (isnan(anymal_->getGeneralizedVelocity().e().norm())) {
      //  std::cout << "history: velocity is nan" << std::endl; 
      //}
      //if (isnan(anymal_->getGeneralizedCoordinate().e().norm())) {
      //  std::cout << "history: coordinate is nan" << std::endl; 
      //}
      //if (isnan(targetJointPosition.norm())) {
      //  std::cout << "history: target is nan" << std::endl; 
      //}
      jointVelHist_.appendValue(anymal_->getGeneralizedVelocity().e().template cast<T>());
      jointPosErrorHist_.appendValue(targetJointPosition.tail(nJoint) - anymal_->getGeneralizedCoordinate().e().template cast<T>().tail(nJoint));
      jointPosTargetHist_.appendValue(targetJointPosition);
    }

    Eigen::Matrix<T, nJoint, 1> getLastJointPositionError(int iter = 1) const
    {
      return jointPosErrorHist_.getLastJointValue(iter);
    }

    T getLastJointPositionError(int actId, int iter) const
    {
      return jointPosErrorHist_.getLastJointValue(actId, iter);
 
    }

    Eigen::Matrix<T, nJoint, 1> getLastJointVelocity(int iter = 1) const
    {
      return jointVelHist_.getLastJointValue(iter);
    }

    T getLastJointVelocity(int actId, int iter) const
    {
      return jointVelHist_.getLastJointValue(actId, iter);
    }

    Eigen::Matrix<T, nJoint, 1> getLastJointPositionTarget(int iter = 1) const 
    {
      return jointPosTargetHist_.getLastJointValue(iter);
    }

    T getLastJointPositionTarget(int actId, int iter) const
    {
      return jointPosTargetHist_.getLastJointValue(actId, iter);
    }
  protected:

   // template<typename T2>
   // inline Eigen::Matrix<T, -1, 1> castType(const Eigen::Matrix<T2, -1, 1>& in)
   // {
   //   if constexpr (std::is_same<T2, T>::value) {
   //     return in;
   //   }
   //   else {
   //     return in.template cast<T>;
   //   }
   // }

    raisim::ArticulatedSystem *anymal_;
    JointHistory<T, jointHistoryLength, nJoint> jointVelHist_, jointPosErrorHist_, jointPosTargetHist_;
};
