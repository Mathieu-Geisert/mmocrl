/*
 * =====================================================================================
 *
 *       Filename:  Terrain.hpp
 *
 *    Description:  Class to handle generation of the terrains.
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
#include "environment/motion/ModelParametersBase.hpp"

//TODO: adjust matrix template???
//TODO: clean namespace.
//TODO: check seed

enum TerrainType {
  Flat,
  Hills,
  Steps,
  Stairs,
  SingleStep,
  UniformSlope,
  Other
};

constexpr const double PixelSize = 0.02;

template<typename T, int Nlimb>
class Terrain {
 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    Terrain(raisim::World* world, const ModelParametersBase<T, Nlimb>* modelParameters, RandomNumberGenerator<T>* rn, double xSize=5., double ySize=5.)
     : world_(world), terrainGenerator_(), footNames_(modelParameters->getFootNames()), rn_(rn)
    {
      terrainProp_.xSize = xSize;
      terrainProp_.ySize = ySize;
      terrainProp_.xSamples = terrainProp_.xSize / PixelSize;
      terrainProp_.ySamples = terrainProp_.ySize / PixelSize;

      terrainProp_.fractalOctaves = 1;
      terrainProp_.frequency = 0.2; ///
      terrainProp_.fractalLacunarity = 3.0;
      terrainProp_.fractalGain = 0.1;
      terrainGenerator_.getTerrainProp() = terrainProp_;

      //init with ground
      terrainType_ = TerrainType::Flat;
      board_ = world_->addGround(0.0, "terrain");
      heights_.resize(terrainProp_.xSamples * terrainProp_.ySamples, 0.0);
      world_->setERP(0.1, 0.0);
      
      setFootFriction(0.9);
    }

    ~Terrain() = default;


  TerrainType getTerrainType() { return terrainType_; }

  Eigen::Vector3d getTerrainNormal(double posX, double posY, double delta = 1.0) const
  {
    Eigen::Matrix<double, 3, 1> terrainNormal;
    if (terrainType_ == TerrainType::Flat) {
      terrainNormal.setZero();
      terrainNormal[2] = 1.0;
    }
    else {
      Eigen::Matrix<double, 3, 1> a1;
      Eigen::Matrix<double, 3, 1> a2;
      a1[0] = 2.0 * delta;
      a1[1] = 0.0;
      a1[2] = terrain_->getHeight(posX + delta, posY) - terrain_->getHeight(posX - delta, posY);
      a2[0] = 0.0;
      a2[1] = 2.0 * delta;
      a2[2] = terrain_->getHeight(posX, posY + delta) - terrain_->getHeight(posX, posY - delta);
      a1.normalize();
      a2.normalize();
      terrainNormal = a1.cross(a2);
    }
    return terrainNormal;
  }

  double getHeight(const double& posX, const double& posY) const
  {
    if (terrainType_ == TerrainType::Flat) {
      return 0.;
    }
    else {
      return terrain_->getHeight(posX, posY);
    }
  }

  void setFootFriction(double c_f) {
    for (int idx=0; idx<Nlimb; idx++) {
      footFriction_[idx] = c_f;
      world_->setMaterialPairProp("terrain", footNames_[idx], footFriction_[idx], 0.0, 0.0);
    }
  }

  void setFootFriction(int idx, double c_f) {
    FATAL_IF(idx >= Nlimb, "[setFootFriction] idx out of bound...");
    footFriction_[idx] = c_f;
    world_->setMaterialPairProp("terrain", footNames_[idx], footFriction_[idx], 0.0, 0.0);
  }

  const Eigen::Matrix<T, Nlimb, 1>& getFootFriction() const { return footFriction_; }

<<<<<<< Updated upstream
  void setRandomFriction(double low, double mid) {
    for (int i = 0; i < 4; i++) {
      footFriction_[i] = std::max(mid + 0.1 * rn_.sampleNormal(), low);
      world_->setMaterialPairProp("terrain", footNames_[i], footFriction_[i], 0.0, 0.0);
    }
    world_->setDefaultMaterial(0.1 + rn_.sampleUniform01() * 0.4, 0.0, 0.0);
=======
  void setRandomFriction(double mean, double std) {
    for (int i = 0; i < 4; i++) {
      footFriction_[i] = std::max(mean + std * rn_->sampleNormal(), 0.1);
      world_->setMaterialPairProp("terrain", footNames_[i], footFriction_[i], 0.0, 0.0);
    }
    world_->setDefaultMaterial(0.1 + rn_->sampleUniform01() * 0.4, 0.0, 0.0);
>>>>>>> Stashed changes
  }
  

  void generateTerrain(TerrainType terrainType, Eigen::Matrix<double, 3, 1> &params)
  {
    if (terrainType_ != TerrainType::Flat)
      world_->removeObject(terrain_);

    terrainType_ = terrainType;

   // if (taskIndex_ == 0) {
   //   terrainType_ = TerrainType::Hills;
   //   setF(0.5, 0.9);
   // } else if (taskIndex_ == 1) {
   //   terrainType_ = TerrainType::Steps;
   //   setF(0.5, 0.9);
   // } else if (taskIndex_ == 2) {
   //   terrainType_ = TerrainType::Stairs;
   //   setF(0.5, 0.8);
   // } else if (taskIndex_ == 3) {
   //   terrainType_ = TerrainType::Hills;
   //   setF(0.1, 0.2);
   // } else if (taskIndex_ == 4) {
   //   terrainType_ = TerrainType::Steps;
   //   setF(0.1, 0.2);
   // } else if (taskIndex_ == 5) {
   //   terrainType_ = TerrainType::Hills;
   //   setF(0.2, 0.7);
   // } else if (taskIndex_ == 6) {
   //   terrainType_ = TerrainType::Steps;
   //   setF(0.2, 0.7);
   // } else if (taskIndex_ == 7) {
   //   terrainType_ = TerrainType::Stairs;
   //   setF(0.2, 0.7);
   // } else if (taskIndex_ == 8) {
   //   terrainType_ = TerrainType::UniformSlope;
   //   setF(0.2, 0.7);
   // }

    if (terrainType_ != TerrainType::Flat) {
      switch (terrainType_) { 
        case TerrainType::Hills: std::cout << "Generate Hills" << std::endl;        generateTerrainHills(params); break;
        case TerrainType::Steps:  std::cout << "Generate Steps" << std::endl;       generateTerrainSteps(params); break;
        case TerrainType::Stairs:  std::cout << "Generate Stairs" << std::endl;      generateTerrainStairs(params); break;
        case TerrainType::SingleStep:  std::cout << "Generate SingleStep" << std::endl;  generateTerrainSingleStep(params); break;
        case TerrainType::UniformSlope:  std::cout << "Generate Slope" << std::endl;generateTerrainUniformSlope(params); break;
        case TerrainType::Other:     std::cout << "Generate Other" << std::endl;    generateTerrainOther(params); break;
        default: throw std::runtime_error("Terrain not defined!");
      }
      terrain_ = world_->addHeightMap(terrainProp_.xSamples,
                                      terrainProp_.ySamples,
                                      terrainProp_.xSize,
                                      terrainProp_.ySize, 0.0, 0.0, heights_, "terrain");
    } else {
      world_->addGround(0.0, "terrain");

    }    
  }

  void generateTerrainStairs(Eigen::Matrix<double, 3, 1> &params)
  {
    //TODO: Why stepLengh/Height does not directly correspond to input params???
    double stepLength = params[1] + 0.1;
    double stepHeight = params[2] + 0.05;

    int N = (int) (stepLength / PixelSize);
    int mid0 = 0.5 * terrainProp_.ySamples - (int) (0.5 / PixelSize);
    int mid1 = 0.5 * terrainProp_.ySamples + (int) (0.7 / PixelSize);
    double stepStart = 0.0;
    double max = 0.0;
    int cnt = 0;
    bool chamfer = false;
    for (int y = 0; y < mid0; y++) {
      if (cnt == N) {
        stepStart = max;
        cnt = 0;
      }
      if (cnt == 0 && rn_.sampleUniform01() < 0.5) chamfer = true;
      else chamfer = false;
      for (int x = 0; x < terrainProp_.xSamples; x++) {
        size_t idx = y * terrainProp_.xSamples + x;
        max = stepStart + stepHeight;
        heights_[idx] = max;
        if (chamfer) heights_[idx] -= PixelSize;
      }
      cnt++;
    }

    for (int y = mid0; y < mid1; y++) {
      for (int x = 0; x < terrainProp_.xSamples; x++) {
        size_t idx = y * terrainProp_.xSamples + x;
        heights_[idx] = max;
      }
    }

    cnt = N;
    for (int y = mid1; y < terrainProp_.ySamples; y++) {
      if (cnt == N) {
        stepStart = max;
        cnt = 0;
      }
      if (cnt == 0 && rn_.sampleUniform01() < 0.5) chamfer = true;
      else chamfer = false;
      for (int x = 0; x < terrainProp_.xSamples; x++) {
        size_t idx = y * terrainProp_.xSamples + x;
        max = stepStart + stepHeight;
        heights_[idx] = max;
        if (chamfer) heights_[idx] -= PixelSize;

      }
      cnt++;
    }
  }

  void generateTerrainSteps(Eigen::Matrix<double, 3, 1> &params)
  {
    //TODO: Why stepLengh/Height does not directly correspond to input params???
    double stepSize = params[1] + 0.2;
    double stepHeight = params[2] + 0.05;

    int xNum = terrainProp_.xSize / stepSize;
    int yNum = terrainProp_.ySize / stepSize;
    int gridWidth_ = stepSize / PixelSize;

    Eigen::Map<Eigen::Matrix<double, -1, -1>> mapMat(heights_.data(),
                                                     terrainProp_.xSamples,
                                                     terrainProp_.ySamples);

    mapMat.setZero();
    ///steps
    for (size_t i = 0; i < xNum; i++) {
      for (size_t j = 0; j < yNum; j++) {
        double h = rn_.sampleUniform01() * stepHeight + 0.5;

        mapMat.block(gridWidth_ * i, gridWidth_ * j, gridWidth_, gridWidth_).setConstant(h);
      }
    }
  }
   
   
  void generateTerrainSingleStep(Eigen::Matrix<double, 3, 1> &params)
  {
    //TODO: Why stepLengh/Height does not directly correspond to input params???
    double stepHeight = params[0] + 0.5;

    Eigen::Map<Eigen::Matrix<double, -1, -1>> mapMat(heights_.data(),
                                                     terrainProp_.xSamples,
                                                     terrainProp_.ySamples);

    mapMat.setConstant(stepHeight);
    int end_idx = 0.6 * terrainProp_.ySamples;
    for (int y = 0; y < end_idx; y++) {
      for (int x = 0; x < terrainProp_.xSamples; x++) {
        size_t idx = y * terrainProp_.xSamples + x;
        if (y == end_idx - 1) {
//          heights_[idx] -= pixelSize_;
        } else {
          heights_[idx] = 0.5;
        }
      }
    }
  }
 
  void generateTerrainHills(Eigen::Matrix<double, 3, 1> &params)
  {
    //TODO: manage param.
    terrainProp_.frequency = params[1]; ///
    terrainProp_.fractalOctaves = 1;

    if (seed_ == -1) {
      terrainGenerator_.setSeed(rn_.intRand(0, 100));
    } else {
      terrainGenerator_.setSeed(seed_);
    }

    terrainGenerator_.getTerrainProp() = terrainProp_;

    heights_ = terrainGenerator_.generatePerlinFractalTerrain();
    for (size_t i = 0; i < heights_.size(); i++) {
      heights_[i] += 1.0;
    }

    Eigen::Map<Eigen::Matrix<double, -1, -1>> mapMat(heights_.data(),
                                                     terrainProp_.xSamples,
                                                     terrainProp_.ySamples);
    mapMat *= params[2] + 0.2;

    for (size_t idx = 0; idx < heights_.size(); idx++) {
      heights_[idx] += (params[0]) * rn_.sampleUniform01();
    }
  }

  void generateTerrainUniformSlope(Eigen::Matrix<double, 3, 1> &params)
  {
    ///temp
    double hills = params[0];
    double dh = std::tan(hills) * PixelSize;

    for (int y = 0; y < terrainProp_.ySamples; y++) {
      for (int x = 0; x < terrainProp_.xSamples; x++) {
        size_t idx = y * terrainProp_.xSamples + x;
        heights_[idx] = dh * y + 0.0;
      }
    }
  } 
  
  
  void generateTerrainOther(Eigen::Matrix<double, 3, 1> &params)
  {
    //TODO:adjust params.
    double stepSize = params[0] + 0.2;
    terrainProp_.frequency = 0.2 + params[1]; ///
    double param3 = params[2] + 0.2;

    terrainProp_.fractalOctaves = 1;

    if (seed_ == -1) {
      terrainGenerator_.setSeed(rn_.intRand(0, 100));
    } else {
      terrainGenerator_.setSeed(seed_);
    }

    terrainGenerator_.getTerrainProp() = terrainProp_;

    heights_ = terrainGenerator_.generatePerlinFractalTerrain();

    for (size_t i = 0; i < heights_.size(); i++) {
      heights_[i] += 1.0;
    }

    Eigen::Map<Eigen::Matrix<double, -1, -1>> mapMat(heights_.data(),
                                                     terrainProp_.xSamples,
                                                     terrainProp_.ySamples);
    mapMat *= param3;

    ///steps
    int xNum = terrainProp_.xSize / stepSize;
    int yNum = terrainProp_.ySize / stepSize;
    int gridWidth = stepSize / PixelSize;

    for (size_t i = 0; i < xNum; i++) {
      for (size_t j = 0; j < yNum; j++) {
        double h = mapMat.block(gridWidth * i, gridWidth * j, gridWidth, gridWidth).mean();

        if (rn_.sampleUniform01() < 0.5) {
          mapMat.block(gridWidth * i, gridWidth * j, gridWidth, gridWidth).setConstant(h);
        }
      }
    }
  }

  
  protected:
    raisim::World* world_;
    raisim::Ground *board_;
    raisim::TerrainGenerator terrainGenerator_;
    raisim::TerrainProperties terrainProp_;
    TerrainType terrainType_;
    raisim::HeightMap* terrain_;
    std::vector<double> heights_;
    std::vector<std::string> footNames_;
    Eigen::Matrix<T, Nlimb, 1> footFriction_;
    RandomNumberGenerator<T>& rn_;
    int seed_ = 0;
}; // end of clann Terrain

