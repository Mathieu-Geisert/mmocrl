//
// Created by Mathieu Geisert on 20.01.2021.
//

#pragma once

#include <Eigen/Core>
#include "environment/motion/ModelParametersBase.hpp"

template<typename T>
class ModelParametersAnymalC100 : public ModelParametersBase<T, 4> {

  public:
    ModelParametersAnymalC100()
      : ModelParametersBase<T,4>()
    {
      this->referenceHight_ = -0.28;
      this->referenceFootPositionOffset_ << 0.3 + 0.1, 0.2, this->referenceHight_,
                                            0.3 + 0.1, -0.2, this->referenceHight_,
                                            -0.3 - 0.1, 0.2, this->referenceHight_,
                                            -0.3 - 0.1, -0.2, this->referenceHight_;
      this->referenceJointConfiguration_ << -0.138589, 0.480936, -0.761428,
                                       0.138589, 0.480936, -0.761428,
                                       -0.138589, -0.480936, 0.761428,
                                       0.138589, -0.480936,  0.761428;

    this->positionBaseToHipInBaseFrame_[0] << 0.3, 0.104, 0.0;
    this->positionBaseToHipInBaseFrame_[1] << 0.3, -0.104, 0.0;
    this->positionBaseToHipInBaseFrame_[2] << -0.3, 0.104, 0.0;
    this->positionBaseToHipInBaseFrame_[3] << -0.3, -0.104, 0.0;

    this->positionHipToThighInHipFrame_[0] << 0.06, 0.08381, -0.0;
    this->positionHipToThighInHipFrame_[1] << 0.06, -0.08381, -0.0;
    this->positionHipToThighInHipFrame_[2] << -0.06, 0.08381, -0.0;
    this->positionHipToThighInHipFrame_[3] << -0.06, -0.08381, -0.0;

    this->positionThighToShankInThighFrame_[0] << 0.0, 0.1003, -0.285;
    this->positionThighToShankInThighFrame_[1] << 0.0, -0.1003, -0.285;
    this->positionThighToShankInThighFrame_[2] << 0.0, 0.1003, -0.285;
    this->positionThighToShankInThighFrame_[3] << 0.0, -0.1003, -0.285;

    this->positionShankToFootInShankFrame_[0] << 0.08795, -0.01305, -0.33797;
    this->positionShankToFootInShankFrame_[1] << 0.08795, 0.01305, -0.33797;
    this->positionShankToFootInShankFrame_[2] << -0.08795, -0.01305, -0.33797;
    this->positionShankToFootInShankFrame_[3] << -0.08795, 0.01305, -0.33797;

    this->positionShankToFootInShankFrame_[0][2] += 0.0225;
    this->positionShankToFootInShankFrame_[1][2] += 0.0225;
    this->positionShankToFootInShankFrame_[2][2] += 0.0225;
    this->positionShankToFootInShankFrame_[3][2] += 0.0225;

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

    ~ModelParametersAnymalC100() = default;

  protected:
};

