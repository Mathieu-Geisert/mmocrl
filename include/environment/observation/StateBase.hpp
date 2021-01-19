/*
 * =====================================================================================
 *
 *       Filename:  TerrainGenerator.hpp
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

#include "common/message_macro.hpp"

namespace terrain {

enum TerrainType {
  Flat_,
  Hills,
  Steps,
  Stairs,
  SingleStep,
  UniformSlope
};

constexpr const double PixelSize = 0.02;


class TerrainGenerator {
 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    TerrainGenerator(raisim::World* world)
     : world_(world)
    {
      terrainProp_.xSize = 10.0;
      terrainProp_.ySize = 10.0;
      terrainProp_.xSamples = terrainProp_.xSize / gridSize_;
      terrainProp_.ySamples = terrainProp_.ySize / gridSize_;

      terrainProp_.fractalOctaves = 1;
      terrainProp_.frequency = 0.2; ///
      terrainProp_.fractalLacunarity = 3.0;
      terrainProp_.fractalGain = 0.1;
      terrainGenerator_.getTerrainProp() = terrainProp_;
      
      //init with ground
      terrain_ = env_->addGround(0.0, "terrain");
    }

    ~TerrainGenerator() = default;

  Eigen::Vector3d getTerrainNormal(double posX, double posY, double delta = 1.0)
  {
    Eigen::Matrix<double, 3, 1> terrainNormal;
    if (terrainType_ == TerrainType::Flat) {
      terrainNormal.setZero();
    }
    else {
      Eigen::Matrix<double, 3, 1> a1;
      Eigen::Matrix<double, 3, 1> a2;
      a1_[0] = 2.0 * delta;
      a1_[1] = 0.0;
      a1_[2] = terrain_->getHeight(posX + delta, posY) - terrain_->getHeight(posX - delta, posY);
      a2_[0] = 0.0;
      a2_[1] = 2.0 * delta;
      a2_[2] = terrain_->getHeight(posX, posY + delta) - terrain_->getHeight(posX, posY - delta);
      a1.normalize();
      a2.normalize();
      terrainNormal = a1_.cross(a2_);
    }
    return terrainNormal;
  }

  void setRandomFriction(double low, double mid) {
    for (int i = 0; i < 4; i++) {
      footFriction_[i] = std::max(mid + 0.1 * rn_.sampleNormal(), low);
      env_->setMaterialPairProp("terrain", footNames_[i], footFriction_[i], 0.0, 0.0);
    }
    env_->setDefaultMaterial(0.1 + rn_.sampleUniform01() * 0.4, 0.0, 0.0);
  }
  

  void generateTerrain(Eigen::Matrix<double, 3, 1> &params) {

    if (terrainType_ != Flat) {
      //if (vis_on_) {
      //  raisim::OgreVis::get()->remove(terrain_);
      //}
      env_->removeObject(terrain_);
    }

    double pixelSize_ = 0.02;

    if (taskIndex_ == 0) {
      terrainType_ = TerrainType::Hills;
      setF(0.5, 0.9);
    } else if (taskIndex_ == 1) {
      terrainType_ = TerrainType::Steps;
      setF(0.5, 0.9);
    } else if (taskIndex_ == 2) {
      terrainType_ = TerrainType::Stairs;
      setF(0.5, 0.8);
    } else if (taskIndex_ == 3) {
      terrainType_ = TerrainType::Hills;
      setF(0.1, 0.2);
    } else if (taskIndex_ == 4) {
      terrainType_ = TerrainType::Steps;
      setF(0.1, 0.2);
    } else if (taskIndex_ == 5) {
      terrainType_ = TerrainType::Hills;
      setF(0.2, 0.7);
    } else if (taskIndex_ == 6) {
      terrainType_ = TerrainType::Steps;
      setF(0.2, 0.7);
    } else if (taskIndex_ == 7) {
      terrainType_ = TerrainType::Stairs;
      setF(0.2, 0.7);
    } else if (taskIndex_ == 8) {
      terrainType_ = TerrainType::UniformSlope;
      setF(0.2, 0.7);
    };

    if (terrainType_ == TerrainType::Stairs) {
      terrainProp_.xSize = 4.0;
      terrainProp_.ySize = 8.0;
      terrainProp_.xSamples = terrainProp_.xSize / pixelSize_;
      terrainProp_.ySamples = terrainProp_.ySize / pixelSize_;

      heights_.resize(terrainProp_.xSamples * terrainProp_.ySamples);

      double stepLength = params[1] + 0.1;
      double stepHeight = params[2] + 0.05;

      int N = (int) (stepLength / pixelSize_);
      int mid0 = 0.5 * terrainProp_.ySamples - (int) (0.5 / pixelSize_);
      int mid1 = 0.5 * terrainProp_.ySamples + (int) (0.7 / pixelSize_);
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
          if (chamfer) heights_[idx] -= gridSize_;
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
          if (chamfer) heights_[idx] -= gridSize_;

        }
        cnt++;
      }
    } else if (terrainType_ == TerrainType::Steps) {
      pixelSize_ = 0.02;
      terrainProp_.xSize = 6.0;
      terrainProp_.ySize = 6.0;
      terrainProp_.xSamples = terrainProp_.xSize / pixelSize_;
      terrainProp_.ySamples = terrainProp_.ySize / pixelSize_;

      heights_.resize(terrainProp_.xSamples * terrainProp_.ySamples);

      double stepSize = params[1] + 0.2;
      double stepHeight = params[2] + 0.05;
      int xNum = terrainProp_.xSize / stepSize;
      int yNum = terrainProp_.ySize / stepSize;
      int gridWidth_ = stepSize / pixelSize_;

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

    } else if (terrainType_ == TerrainType::SingleStep) {

      terrainProp_.xSize = 8.0;
      terrainProp_.ySize = 8.0;
      terrainProp_.xSamples = terrainProp_.xSize / pixelSize_;
      terrainProp_.ySamples = terrainProp_.ySize / pixelSize_;

      heights_.resize(terrainProp_.xSamples * terrainProp_.ySamples);

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
//            heights_[idx] -= pixelSize_;
          } else {
            heights_[idx] = 0.5;
          }
        }
      }

    } else if (terrainType_ == TerrainType::Hills) {
      pixelSize_ = 0.15;

      terrainProp_.xSize = 80.0;
      terrainProp_.ySize = 80.0;
      terrainProp_.xSamples = terrainProp_.xSize / pixelSize_;
      terrainProp_.ySamples = terrainProp_.ySize / pixelSize_;

      heights_.resize(terrainProp_.xSamples * terrainProp_.ySamples);

      terrainProp_.fractalOctaves = 1;
      terrainProp_.frequency = 0.1 + params[1]; ///

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
    } else if (terrainType_ == TerrainType::UniformSlope) {
      pixelSize_ = 0.02;
      terrainProp_.xSize = 8.0;
      terrainProp_.ySize = 8.0;
      terrainProp_.xSamples = terrainProp_.xSize / pixelSize_;
      terrainProp_.ySamples = terrainProp_.ySize / pixelSize_;
      ///temp
      double Hills = params[0];
      double dh = std::tan(Hills) * pixelSize_;

      for (int y = 0; y < terrainProp_.ySamples; y++) {
        for (int x = 0; x < terrainProp_.xSamples; x++) {
          size_t idx = y * terrainProp_.xSamples + x;
          heights_[idx] = dh * y + 5.0;
        }
      }
    } else {
      terrainProp_.xSize = 8.0;
      terrainProp_.ySize = 8.0;
      terrainProp_.xSamples = terrainProp_.xSize / pixelSize_;
      terrainProp_.ySamples = terrainProp_.ySize / pixelSize_;

      heights_.resize(terrainProp_.xSamples * terrainProp_.ySamples);

      terrainProp_.fractalOctaves = 1;
      terrainProp_.frequency = 0.2 + params[2]; ///

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
      mapMat *= params[3] + 0.2;

      ///steps
      double stepSize = params[1] + 0.2;
      int xNum = terrainProp_.xSize / stepSize;
      int yNum = terrainProp_.ySize / stepSize;
      int gridWidth_ = stepSize / pixelSize_;

      for (size_t i = 0; i < xNum; i++) {
        for (size_t j = 0; j < yNum; j++) {
          double h = mapMat.block(gridWidth_ * i, gridWidth_ * j, gridWidth_, gridWidth_).mean();

          if (rn_.sampleUniform01() < 0.5) {
            mapMat.block(gridWidth_ * i, gridWidth_ * j, gridWidth_, gridWidth_).setConstant(h);
          }
        }
      }
    }
    Eigen::Map<Eigen::Matrix<double, -1, -1>> mapMat(heights_.data(),
                                                     terrainProp_.xSamples,
                                                     terrainProp_.ySamples);

    terrain_ = env_->addHeightMap(terrainProp_.xSamples,
                                  terrainProp_.ySamples,
                                  terrainProp_.xSize,
                                  terrainProp_.ySize, 0.0, 0.0, heights_, "terrain");
    //if (vis_on_) {
    //  raisim::OgreVis::get()->createGraphicalObject(terrain_, "height_map", "gray");
    //}
  }
  protected:
    raisim::World* world_;
    raisim::HeightMap* terrain_;
    raisim::TerrainProperties terrainProp_;
    TerrainType terrainType_;
}; // end of clann TerrainGenerator
} // end of namespace terrain

