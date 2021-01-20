/*
 * =====================================================================================
 *
 *       Filename:  FoottMotionGenerator.hpp
 *
 *    Description:  Steping Motion generation.
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

template<typename T, int Nleg>
class FootMotionGenerator {
 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    FootMotionGenerator(const ModelParametersBase<T, Nleg>& model, double baseFreq, double clearance, double control_dt)
    : baseFreq_(baseFreq), 
      control_dt_(control_dt),
      footPositionOffset_(model.getReferenceFootPositionOffset())
    {
      clearance_.setConstant(clearance);

     //init pi_
      for (size_t i = 0; i < Nleg; i++) {
        //pi_[i] = 2.0 * M_PI * rn_.sampleUniform01();
        if (i==0 || i==3)
          pi_[i] = M_PI;
        else
          pi_[i] = 0.;
        pi_[i] = anglemod(pi_[i]);
      }
    }

    ~FootMotionGenerator() = default;

    Eigen::Matrix<T, Nleg*3, 1> advance(const Eigen::Matrix<T, Nleg, 1>& deltaFrequency, const Eigen::Matrix<T, 3, 1>& gravityVectorInBaseFrame)
    {
      Eigen::Matrix<T, Nleg*3, 1> footPos_Target(footPositionOffset_);
      Eigen::Matrix<T, 3, 1> gravityNormalized(gravityVectorInBaseFrame);
      gravityNormalized.normalize();

      for (size_t j = 0; j < Nleg; j++) {
        pi_[j] += (deltaFrequency[j] + baseFreq_) * 2.0 * M_PI * control_dt_;
        pi_[j] = anglemod(pi_[j]);
        T dh = 0.0;
        if (pi_[j] > 0.0) {
          T t = pi_[j] / M_PI_2;
          if (t < 1.0) {
            T t2 = t * t;
            T t3 = t2 * t;
            dh = (-2 * t3 + 3 * t2);
          } else {
            t = t - 1;
            T t2 = t * t;
            T t3 = t2 * t;
            dh = (2 * t3 - 3 * t2 + 1.0);
          }
          dh *= clearance_[j];
        }
  
        footPos_Target[j*3+2] = 0.0;
        footPos_Target.segment(j*3, 3) += gravityNormalized * (footPositionOffset_[3 * j + 2] + dh);
      }
    }

    const Eigen::Matrix<T, Nleg, 1>& getCurrentPhases() { return pi_; }

    void setClearance(double clearance) { clearance_.setConstant(clearance); }
    void setClearance(Eigen::Matrix<T, Nleg, 1> clearance) { clearance_ = clearance; }

    void setPhases(Eigen::Matrix<T, Nleg, 1> phases) { pi_ = phases; }
    const Eigen::Matrix<T, Nleg, 1>& setPhases() const { return pi_; }
    void setBaseFrequency(double baseFreq) { baseFreq_ = baseFreq; }
    void updateControlDt(double control_dt) { control_dt_ = control_dt; }
  
  protected:
    inline T anglemod(T a) {
      return wrapAngle((a + M_PI)) - M_PI;
    }

    inline T wrapAngle(T a) {
      double twopi = 2.0 * M_PI;
      return a - twopi * fastfloor(a / twopi);
    }

    inline int fastfloor(T a) {
      int i = int(a);
      if (i > a) i--;
      return i;
    }

    T baseFreq_;
    T control_dt_;
    Eigen::Matrix<T, Nleg, 1> clearance_;
    Eigen::Matrix<T, Nleg, 1> pi_;
    const Eigen::Matrix<T, Nleg*3, 1>& footPositionOffset_;
}; // end of class FootMotionGenerator

