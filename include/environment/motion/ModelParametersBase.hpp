//
// Created by Mathieu Geisert on 20.01.2021.
//

#pragma once

#include <Eigen/Core>

//TODO: clean std::vector vs Matrix and harmonize shapes.

template<typename T, int Nlimb>
class ModelParametersBase {

  public:
    ModelParametersBase()
    {
      positionBaseToHipInBaseFrame_.resize(Nlimb);
      positionHipToThighInHipFrame_.resize(Nlimb);
      positionThighToShankInThighFrame_.resize(Nlimb);
      positionShankToFootInShankFrame_.resize(Nlimb);
      //positionBaseToHAACenterInBaseFrame.resize(Nlimb);
      
      footIds_.resize(Nlimb);
      footNames_.resize(Nlimb);
    }

    ~ModelParametersBase() = default;

    const std::vector<Eigen::Matrix<T, 3, 1>>& getPositionHipToThighInHipFrame() const { return positionHipToThighInHipFrame_; }
    const std::vector<Eigen::Matrix<T, 3, 1>>& getPositionThighToShankInThighFrame() const { return positionThighToShankInThighFrame_; }
    const std::vector<Eigen::Matrix<T, 3, 1>>& getPositionShankToFootInShankFrame() const { return positionShankToFootInShankFrame_; }
    const std::vector<Eigen::Matrix<T, 3, 1>>& getPositionBaseToHipInBaseFrame() const { return positionBaseToHipInBaseFrame_; }
    //const std::vector<Eigen::Matrix<T, 3, 1>>& getPositionBaseToHAACenterInBaseFrame() { return positionBaseToHAACenterInBaseFrame; }
    
    const Eigen::Matrix<T, Nlimb*3, 1>& getReferenceJointConfiguration() const { return referenceJointConfiguration_; }

    const T& getReferenceHight() const { return referenceHight_; }
    const Eigen::Matrix<T, Nlimb*3, 1>& getReferenceFootPositionOffset() const { return referenceFootPositionOffset_; }

    const std::vector<uint>& getFootIds() const { return footIds_; }
    const std::vector<std::string>& getFootNames() const { return footNames_; }
    //const Eigen::Matrix<T, Nlimb, 1>& getHAAToFootYOffset() { return HAAToFootYOffset_; }
    //const Eigen::Matrix<T, Nlimb, 1>& getHFEToFootYOffset() { return HFEToFootYOffset_; }

  protected:
    std::vector<Eigen::Matrix<T, 3, 1>> positionHipToThighInHipFrame_;
    std::vector<Eigen::Matrix<T, 3, 1>> positionThighToShankInThighFrame_;
    std::vector<Eigen::Matrix<T, 3, 1>> positionShankToFootInShankFrame_;
    std::vector<Eigen::Matrix<T, 3, 1>> positionBaseToHipInBaseFrame_;
    //std::vector<Eigen::Matrix<T, 3, 1>> positionBaseToHAACenterInBaseFrame_;

    Eigen::Matrix<T, Nlimb*3, 1> referenceJointConfiguration_;

    T referenceHight_;
    Eigen::Matrix<T, Nlimb*3, 1> referenceFootPositionOffset_;
   
    std::vector<uint> footIds_;
    std::vector<std::string> footNames_;

    //Eigen::Matrix<T, Nlimb, 1> HAAToFootYOffset_;
    //Eigen::Matrix<T, Nlimb, 1> HFEToFootYOffset_;
};

