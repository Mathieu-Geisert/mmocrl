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
    
    int getNJoint() { return nJoint; }
    int getHistoryLength() { return jointHistoryLength; }
    
    T getLastJointValue(int actId, int iter)
    {
      FATAL_IF(actId<0 || actId>nJoint-1, "[JointHistory::getLastJointPosition] wrong actuator Id...");
      return jointHist_(actId + getPositionInBuffer(iter));
    }
    
    Eigen::Matrix<T, nJoint, 1> getLastJointValue(const int& iter = 1)
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

    inline int getPositionInBuffer(const int& pastIter) 
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
     : anymal_(anymal), jointVelHist_(), jointPosHist_()
    {
      reset();
    }

    ~JointVelPosErrorHistory() = default;

    void reset()
    {
      jointVelHist_.reset(anymal_->getGeneralizedVelocity().e().template cast<T>());
      jointPosHist_.reset();
    }

    int getNJoint() { return nJoint; }
    int getHistoryLength() { return jointHistoryLength; }

    void appendCurrentState(const Eigen::Matrix<T, -1, 1> targetJointPosition)
    {
      jointVelHist_.appendValue(anymal_->getGeneralizedVelocity().e().template cast<T>());
      jointPosHist_.appendValue(targetJointPosition.tail(nJoint) - anymal_->getGeneralizedCoordinate().e().template cast<T>().tail(nJoint));
    }

    Eigen::Matrix<T, nJoint, 1> getLastJointPositionError(int iter = 1)
    {
      return jointPosHist_.getLastJointValue(iter);
    }

    T getLastJointPositionError(int actId, int iter)
    {
      return jointPosHist_.getLastJointValue(actId, iter);
 
    }

    Eigen::Matrix<T, nJoint, 1> getLastJointVelocity(int iter = 1)
    {
      return jointVelHist_.getLastJointValue(iter);
    }

    T getLastJointVelocity(int actId, int iter)
    {
      return jointVelHist_.getLastJointValue(actId, iter);
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
    JointHistory<T, jointHistoryLength, nJoint> jointVelHist_, jointPosHist_;
};
