/*
 * =====================================================================================
 *
 *       Filename:  ActuatorModelPD.hpp
 *
 *    Description:  Actuator model using directly raisim PD controller (+ feed forward torque).
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

template<typename T>
class ActuatorModelPD : public ActuatorModelBase<T> {
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  public:
    ActuatorModelPD(raisim::ArticulatedSystem* anymal, const Eigen::Matrix<T, -1, 1>& pGains, const Eigen::Matrix<T, -1, 1>& dGains) 
    : ActuatorModelBase<T>(anymal)
    {
      init(pGains, dGains);
    }

    ActuatorModelPD(raisim::ArticulatedSystem* anymal, const T& pGains = 50.0, const T& dGains = 0.2) 
    : ActuatorModelBase<T>(anymal)
    {
      Eigen::Matrix<T, -1, 1> jointPgain(this->nJoint_), jointDgain(this->nJoint_);
      jointPgain.setConstant(pGains);
      jointDgain.setConstant(dGains);
      init(jointPgain, jointDgain);
    }

    ~ActuatorModelPD() = default;

    virtual void advance() 
    {
      Eigen::Matrix<T, -1, 1> pTarget(this->nDOF_+1), vTarget(this->nDOF_), gForce(this->nDOF_);
      pTarget.tail(this->nJoint_) = this->targetJointPosition_;
      vTarget.tail(this->nJoint_) = this->targetJointVelocity_;
      this->anymal_->setPdTarget(pTarget.template cast<double>(), vTarget.template cast<double>());

      //only set torques on the joints and keep forces on the base to allow perturbations from another module?
      //anymal_->getGeneralizedForce(gForce); or gForce.setZero();...?
      gForce.setZero();
      gForce.tail(this->nJoint_) = this->feedForwardTorque_;
      this->anymal_->setGeneralizedForce(gForce.template cast<double>());
    } 

  protected:

    void init(const Eigen::Matrix<T, -1, 1>& pGains, const Eigen::Matrix<T, -1, 1>& dGains)
    {
      this->anymal_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
      if (this->nDOF_ == pGains.size() && this->nDOF_ == dGains.size()) {
        this->anymal_->setPdGains(pGains.template cast<double>(), dGains.template cast<double>());
      }
      else if (this->nJoint_ == pGains.size() && this->nJoint_ == dGains.size()) {
        Eigen::Matrix<T, -1, 1> fullPGains, fullDGains;
        fullPGains.setZero(this->nDOF_); fullPGains.tail(this->nJoint_) = pGains;
        fullDGains.setZero(this->nDOF_); fullDGains.tail(this->nJoint_) = dGains;
        //std::cout << "pGains: " << pGains.transpose() << " -- dGains: " << dGains.transpose() << std::endl;
        this->anymal_->setPdGains(fullPGains.template cast<double>(), fullDGains.template cast<double>());
      }
      else {
        FATAL("dGains and pGains should have the same size of the robot number of DOF or number of joints.");
      }
    }
};
