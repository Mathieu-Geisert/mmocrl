//
// Created by joonho on 10.06.19.(IK_c100.hpp)
// Modified by Mathieu Geisert on 20.01.2021
//

#pragma once 

#include <Eigen/Core>
#include "environment/motion/ModelParametersBase.hpp"
#include "common/message_macros.hpp"

template<typename T, int Nlimb>
class InverseKinematics {

 public:
  InverseKinematics(const ModelParametersBase<T, Nlimb>& model) 
  //: model_(model)
  {
    a1_squared_ = model.getPositionThighToShankInThighFrame()[0][0] * model.getPositionThighToShankInThighFrame()[0][0]
        + model.getPositionThighToShankInThighFrame()[0][2] * model.getPositionThighToShankInThighFrame()[0][2];
    a2_squared_ = model.getPositionShankToFootInShankFrame()[0][0] * model.getPositionShankToFootInShankFrame()[0][0]
        + model.getPositionShankToFootInShankFrame()[0][2] * model.getPositionShankToFootInShankFrame()[0][2];

    minReachSP_ = std::abs(sqrt(a1_squared_) - sqrt(a2_squared_)) + 0.1;
    maxReachSP_ = sqrt(a1_squared_) + sqrt(a2_squared_) - 0.05;
    minReach_ = std::sqrt(haa_to_foot_y_offset_[0] * haa_to_foot_y_offset_[0] + minReachSP_ * minReachSP_);
    maxReach_ = sqrt(haa_to_foot_y_offset_[0] * haa_to_foot_y_offset_[0] + maxReachSP_ * maxReachSP_);
    KFEOffset_ = std::abs(std::atan(model.getPositionShankToFootInShankFrame()[0][0] / model.getPositionShankToFootInShankFrame()[0][2]));
    
    positionBaseToHAACenterInBaseFrame_.resize(4);
    positionBaseToHAACenterInBaseFrame_ = model.getPositionBaseToHipInBaseFrame();
    for (size_t i = 0; i < 4; i++) {
      positionBaseToHAACenterInBaseFrame_[i][0] += model.getPositionHipToThighInHipFrame()[i][0];
      hfe_to_foot_y_offset_[i] = model.getPositionThighToShankInThighFrame()[i][1];
      hfe_to_foot_y_offset_[i] += model.getPositionShankToFootInShankFrame()[i][1];
      haa_to_foot_y_offset_[i] = hfe_to_foot_y_offset_[i];
      haa_to_foot_y_offset_[i] += model.getPositionHipToThighInHipFrame()[i][1];
    }
  }

  ~InverseKinematics() = default;

  void EulerAnglesZYX(const T z, const T y, const T x, Eigen::Matrix<T, 3, 3>& C) const {
    T cx = cos(-x);
    T sx = sin(-x);
    T cy = cos(-y);
    T sy = sin(-y);
    T cz = cos(-x);
    T sz = sin(-x);
    //[cos(z)*cos(y), -sin(z)*cos(x)+cos(z)*sin(y)*sin(x),  sin(z)*sin(x)+cos(z)*sin(y)*cos(x)]
    //[sin(z)*cos(y),  cos(z)*cos(x)+sin(z)*sin(y)*sin(x), -cos(z)*sin(x)+sin(z)*sin(y)*cos(x)]
    //[      -sin(y),                       cos(y)*sin(x),                       cos(y)*cos(x)]
    C <<
      cz * cy, -sz * cx + cz * sy * sx, sz * sx + cz * sy * cx,
        sz * cy, cz * cx + sz * sy * sx, -cz * sx + sz * sy * cx,
        -sy, cy * sx, cy * cx;
  }

  inline bool IKSagittal(
      Eigen::Matrix<T, 3, 1>& legJoints,
      const Eigen::Matrix<T, 3, 1>& positionBaseToFootInBaseFrame,
      size_t limb) const {

    FATAL_IF(limb >= Nlimb, "[InverseKinematics] limb number should be smaller than Nlimb."); 

    Eigen::Matrix<T, 3, 1>
        positionHAAToFootInBaseFrame = positionBaseToFootInBaseFrame - positionBaseToHAACenterInBaseFrame_[limb];

    const T d = haa_to_foot_y_offset_[limb];
    const T dSquared = d * d;

    ///Rescaling target
    T reach = positionHAAToFootInBaseFrame.norm();
    if (reach > maxReach_) {
      positionHAAToFootInBaseFrame /= reach;
      positionHAAToFootInBaseFrame *= maxReach_;
    } else if (reach < minReach_) {
      positionHAAToFootInBaseFrame /= reach;
      positionHAAToFootInBaseFrame *= minReach_;
    }
    T positionYzSquared = positionHAAToFootInBaseFrame.tail(2).squaredNorm();

    if (positionYzSquared < dSquared) {

      positionHAAToFootInBaseFrame.tail(2) /= std::sqrt(positionYzSquared);
      positionHAAToFootInBaseFrame.tail(2) *= (std::abs(d) + 0.01);

      if (positionHAAToFootInBaseFrame[0] > maxReachSP_) {
        positionHAAToFootInBaseFrame[0] /= std::abs(positionHAAToFootInBaseFrame[0]);
        positionHAAToFootInBaseFrame[0] *= maxReachSP_;
      }
      positionYzSquared = positionHAAToFootInBaseFrame.tail(2).squaredNorm();
    }

    //compute HAA angle
    T rSquared = positionYzSquared - dSquared;
    const T r = std::sqrt(rSquared);
    const T delta = std::atan2(positionHAAToFootInBaseFrame.y(),
                                    -positionHAAToFootInBaseFrame.z());
    const T beta = std::atan2(r, d);
    const T qHAA = beta + delta - M_PI_2;
    legJoints[0] = qHAA;


    ///simplification for anymal
    const T l_squared = (rSquared + positionHAAToFootInBaseFrame[0] * positionHAAToFootInBaseFrame[0]);
    const T phi1 = std::acos((a1_squared_ + l_squared - a2_squared_) * 0.5 / (sqrt(a1_squared_ * l_squared)));
    const T phi2 = std::acos((a2_squared_ + l_squared - a1_squared_) * 0.5 / (sqrt(a2_squared_ * l_squared)));

    T qKFE = phi1 + phi2 - KFEOffset_;

    if (limb < 2) {
      qKFE *= -1.0;
    }
    legJoints[2] = qKFE;

    T theta_prime = atan2(positionHAAToFootInBaseFrame[0], r);
    T qHFE = phi1 - theta_prime;

    if (limb > 1) {
      qHFE = -phi1 - theta_prime;
    }
    legJoints[1] = qHFE;
    return true;
  }

  //ModelParamtersBase<T, Nlimb>* model_;
  T a1_squared_;
  T a2_squared_;
  T KFEOffset_;
  T minReach_;
  T maxReach_;
  T minReachSP_;
  T maxReachSP_;

  std::vector<Eigen::Matrix<T, 3, 1>> positionBaseToHAACenterInBaseFrame_;
  T haa_to_foot_y_offset_[4];
  T hfe_to_foot_y_offset_[4];
};
