//----------------------------// 
// Author: Mathieu Geisert.   //
//----------------------------//

#pragma once

#include "VectorizedEnvironment.hpp"
#include "common/RandomNumberGenerator.hpp"
#include "TerrainParticleFilter.hpp"

namespace raisim {

template<class ChildEnvironment>
class VectorizedEnvironmentTerrainCurriculum : public VectorizedEnvironment<ChildEnvironment> {

 public:

  explicit VectorizedEnvironmentTerrainCurriculum(std::string resourceDir, std::string cfg)
      : VectorizedEnvironment<ChildEnvironment>(resourceDir, cfg)
  {
  }


  void init() {
    VectorizedEnvironment<ChildEnvironment>::init();
    
    //set terrain filter and init
    Eigen::Vector3f min, max;
    Eigen::Vector2f friction;
    std::vector<ChildEnvironment*> terrainEnvs;

    terrainFilters_.clear();
    terrainFilters_.reserve(4);
    int env_per_terrain = this->getNumOfEnvs() / 4;
    terrainEnvs.reserve(env_per_terrain);
    int terrainId = 0;
    
    //step
    min << 0.0, 0.1, 0.05;
    max << 0.0, 0.5, 0.3;
    friction << 0.7, 0.2;
    for (;terrainId < 1*env_per_terrain; terrainId++) {
      terrainEnvs.push_back(this->environments_[terrainId]);
    }
    terrainFilters_.push_back(TerrainParticleFilter<ChildEnvironment>(terrainEnvs, TerrainType::Steps, min, max, friction));
    terrainEnvs.clear();
    
    //stair
    min << 0.0, 0.1, 0.02;
    max << 0.0, 0.5, 0.2;
    friction << 0.7, 0.2;
    for (;terrainId < 2*env_per_terrain; terrainId++) {
      terrainEnvs.push_back(this->environments_[terrainId]);
    }
    terrainFilters_.push_back(TerrainParticleFilter<ChildEnvironment>(terrainEnvs, TerrainType::Stairs, min, max, friction));
    terrainEnvs.clear();

    //Hills
    min << 0.0, 0.2, 0.2;
    max << 0.05, 1.0, 3.0;
    friction << 0.7, 0.2;
    for (;terrainId < 3*env_per_terrain; terrainId++) {
      terrainEnvs.push_back(this->environments_[terrainId]);
    }
    //TerrainParticleFilter<ChildEnvironment>* hills = new ;
    terrainFilters_.push_back(TerrainParticleFilter<ChildEnvironment>(terrainEnvs, TerrainType::Hills, min, max, friction));
    terrainEnvs.clear();

    //Slippery Hills
    min << 0.0, 0.2, 0.2;
    max << 0.5, 1.0, 3.0;
    friction << 0.3, 0.1;
    for (;terrainId < 4*env_per_terrain; terrainId++) {
      terrainEnvs.push_back(this->environments_[terrainId]);
    }
    //TerrainParticleFilter<ChildEnvironment>* slipperyHills = new ;
    terrainFilters_.push_back(TerrainParticleFilter<ChildEnvironment>(terrainEnvs, TerrainType::Hills, min, max, friction));
    terrainEnvs.clear();

    for (int i=0; i<terrainFilters_.size(); i++) {
      terrainFilters_[i].init();
    }
  }

  ~VectorizedEnvironmentTerrainCurriculum() {
  }

  void updateTerrains() 
  {
    for (int i=0; i<terrainFilters_.size(); i++) {
      terrainFilters_[i].advance();
    }
  }

 private:
  std::vector<TerrainParticleFilter<ChildEnvironment>> terrainFilters_;
};

}

