/*
 * =====================================================================================
 *
 *       Filename:  State.hpp
 *
 *    Description:  Class to handle anymal state with some additional value like 
 *                  graviy vector in base frame, rotation matrix, heading angle...
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
#include "common/RandomNumberGenerator.hpp"
#include "common/math.hpp"
#include <raisim/object/ArticulatedSystem/ArticulatedSystem.hpp>

//TODO: be agnostic to the number of joints.
//TODO: try to automate updateSate ???
//TODO: better organize noisy orientation vs real orientation.

template<typename T>
class State {
  public:  
    State(raisim::ArticulatedSystem* anymal)
     :anymal_(anymal)
    {
      q_.setZero(19);
      u_.setZero(18);
      updateState(); 
    } 
  
    ~State() = default;

    virtual void advance() 
    {
      updateState();
    }
  
    virtual void updateState() 
    {
      Eigen::Matrix<T, -1, 1> q_temp = anymal_->getGeneralizedCoordinate().e().template cast<T>();
      Eigen::Matrix<T, -1, 1> u_temp = anymal_->getGeneralizedVelocity().e().template cast<T>();
      
      if (isnan(u_temp.norm()) || isinf(u_temp.norm()) || isnan(q_temp.norm()) || isinf(q_temp.norm())) {
        std::cout << "state badly conditioned..." << std::endl;
        badlyConditioned_ = true;
      }
      else {
        q_ = q_temp;
        u_ = u_temp;
        badlyConditioned_ = false;
      }
     
      updateDependantVariables();
    }

    Eigen::Matrix<T, -1, 1> getJointPos() const { return q_.template tail<12>(); }
    Eigen::Matrix<T, -1, 1> getJointVel() const { return u_.template tail<12>(); }
    Eigen::Matrix<T, 3, 1> getBasePos() const { return q_.template head<3>(); }
    Eigen::Matrix<T, 3, 1> getBaseVel() const { return u_.template head<3>(); }
    Eigen::Matrix<T, 3, 1> getBaseVelInBaseFrame() const { return Rb_.transpose()*u_.template head<3>(); }
    Eigen::Matrix<T, 3, 1> getBaseAngVel() const { return u_.template segment<3>(3); }
    Eigen::Matrix<T, 3, 1> getBaseAngVelInBaseFrame() const { return Rb_.transpose()*u_.template segment<3>(3); }
    const Eigen::Matrix<T, 3, 3>& getRotationMatrix() const { return Rb_; }
    const Eigen::Matrix<T, 3, 1>& getGravityAxis() const { return eg_; }
    const Eigen::Matrix<T, 3, 1>& getXHorizontal() const { return xHorizontal_; }
    const Eigen::Matrix<T, 3, 1>& getYHorizontal() const { return yHorizontal_; }
    const T& getHeadingAngle() const { return headingAngle_; }
  
  protected:
    void updateDependantVariables()
    {
      Eigen::Matrix<T, 4, 1> quat = q_.template segment<4>(3);
      Rb_ = Math::MathFunc::quatToRotMat(quat);
      eg_ = Rb_.row(2).transpose();
      
      yHorizontal_ << 0.0, eg_[2], -eg_[1]; // eg cross 1,0,0
      yHorizontal_.normalize();
      xHorizontal_ = yHorizontal_.cross(eg_); // eg cross y_;
      xHorizontal_.normalize();
    
      headingAngle_ = std::atan2(Rb_.col(0)[1], Rb_.col(0)[0]);
    }

    raisim::ArticulatedSystem* anymal_;
    Eigen::Matrix<T, -1, 1> q_; 
    Eigen::Matrix<T, -1, 1> u_;
    Eigen::Matrix<T, 3, 3> Rb_;
    Eigen::Matrix<T, 3, 1> eg_;
    Eigen::Matrix<T, 3, 1> xHorizontal_, yHorizontal_;
    T headingAngle_;
    bool badlyConditioned_;
}; // end of class State

template<typename T>
class NoisyState : public State<T> {
  //TODO: Add proper interface for loading/changing noise scaling.

  public:
    NoisyState(raisim::ArticulatedSystem* anymal, RandomNumberGenerator<T>* rn) 
      : State<T>(anymal), rn_(rn)
    {
      updateEGBias();
      updateState();
    }

    virtual void updateState() override
    {
      this->q_ = this->anymal_->getGeneralizedCoordinate().e().template cast<T>();
      this->u_ = this->anymal_->getGeneralizedVelocity().e().template cast<T>();
     
      /// noisify orientation with constant bias
      Eigen::Matrix<T, 4, 1> quat = this->q_.template segment<4>(3);
      this->q_.template segment<4>(3) = Math::MathFunc::boxplusI_Frame(quat, eg_bias_);
      
      /// noisify body vel
      for (int i = 0; i < 3; i++)
        this->u_[i] += rn_->sampleUniform() * 0.1;
  
      /// noisify body angvel
      for (int i = 3; i < 6; i++)
        this->u_[i] += rn_->sampleUniform() * 0.2;
  
      /// noisify joint position
      for (int i = 7; i < 19; i++)
        this->q_[i] += rn_->sampleUniform() * 0.01;
  
      /// noisify joint vel
      for (int i = 6; i < 18; i++)
        this->u_[i] += rn_->sampleUniform() * 1.5;
      
      this->updateDependantVariables();
    }

    void updateEGBias(double scaling = 0.1)
    {
      Eigen::Matrix<T, 3, 1> axis;
      axis << rn_->sampleUniform01(), rn_->sampleUniform01(), rn_->sampleUniform01();
      axis.normalize();
      eg_bias_ = scaling * rn_->sampleUniform() * axis;
    }

    const Eigen::Matrix<T, 3, 1>& getEGBias() const { return eg_bias_; }

  protected:
    RandomNumberGenerator<T>* rn_;
    Eigen::Matrix<T, 3, 1> eg_bias_; 
}; // end of class NoisyState
