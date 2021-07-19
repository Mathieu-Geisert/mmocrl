//
// Created by Mathieu Geisert on 20.01.2021.
//

#pragma once

#include <Eigen/Core>
#include "environment/motion/ModelParametersBase.hpp"

template<typename T>
class ModelParametersA1 : public ModelParametersBase<T, 4> {

  public:
    ModelParametersA1()
      : ModelParametersBase<T,4>()
    {
      this->referenceHight_ = -0.27;
      this->referenceFootPositionOffset_ << 0.1805 + 0.025, 0.12, this->referenceHight_,
                                            0.1805 + 0.025, -0.12, this->referenceHight_,
                                            -0.1805 - 0.025, 0.12, this->referenceHight_,
                                            -0.1805 - 0.025, -0.12, this->referenceHight_;
      this->referenceJointConfiguration_ << -0.0, 0.8, -1.6, // Front Left
                                           -0.0, 0.8, -1.6, // Front Right
                                           -0.0, 0.8, -1.6, // Rear Left
                                           -0.0, 0.8, -1.6, // Rear Right

    this->positionBaseToHipInBaseFrame_[0] << 0.1805, 0.047, 0.0;
    this->positionBaseToHipInBaseFrame_[1] << 0.1805, -0.047, 0.0;
    this->positionBaseToHipInBaseFrame_[2] << -0.1805, 0.047, 0.0;
    this->positionBaseToHipInBaseFrame_[3] << -0.1805, -0.047, 0.0;

    this->positionHipToThighInHipFrame_[0] << 0.00, 0.08381, -0.0;
    this->positionHipToThighInHipFrame_[1] << 0.00, -0.08381, -0.0;
    this->positionHipToThighInHipFrame_[2] << 0.00, 0.08381, -0.0;
    this->positionHipToThighInHipFrame_[3] << 0.00, -0.08381, -0.0;

    this->positionThighToShankInThighFrame_[0] << 0.0, 0.000, -0.2;
    this->positionThighToShankInThighFrame_[1] << 0.0, 0.000, -0.2;
    this->positionThighToShankInThighFrame_[2] << 0.0, 0.000, -0.2;
    this->positionThighToShankInThighFrame_[3] << 0.0, 0.000, -0.2;

    this->positionShankToFootInShankFrame_[0] << 0.00, 0.00, -0.2;
    this->positionShankToFootInShankFrame_[1] << 0.00, 0.00, -0.2;
    this->positionShankToFootInShankFrame_[2] << 0.00, 0.00, -0.2;
    this->positionShankToFootInShankFrame_[3] << 0.00, 0.00, -0.2;

    this->positionShankToFootInShankFrame_[0][2] += 0.018;
    this->positionShankToFootInShankFrame_[1][2] += 0.018;
    this->positionShankToFootInShankFrame_[2][2] += 0.018;
    this->positionShankToFootInShankFrame_[3][2] += 0.018;

    for (int k = 0; k < 4; k++) {
      this->footIds_[k] = 3 * k + 3;
      this->footNames_[k] = "foot" + std::to_string(k);
    }

    //for (size_t i = 0; i < 4; i++) {
    //  this->positionBaseToHAACenterInBaseFrame_[i] = this->positionBaseToHipInBaseFrame_[i];
    //  this->positionBaseToHAACenterInBaseFrame_[i][0] += this->positionHipToThighInHipFrame_[i][0];
    //  this->HFEToFootYOffset_[i] = this->positionThighToShankInThighFrame_[i][1];
    //  this->HFEToFootYOffset_[i] += this->positionShankToFootInShankFrame_[i][1];
    //  this->HAAToFootYOffset_[i] = this->HFEToFootYOffset_[i];
    //  this->HAAToFootYOffset_[i] += this->positionHipToThighInHipFrame_[i][1];
    //}
    }

    ~ModelParametersA1() = default;

  protected:
};

