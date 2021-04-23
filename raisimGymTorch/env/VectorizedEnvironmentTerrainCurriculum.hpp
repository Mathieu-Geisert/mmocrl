//----------------------------// 
// Author: Mathieu Geisert.   //
//----------------------------//

#pragma once

#include "VectorizedEnvironment.hpp"
#include "common/RandomNumberGenerator.hpp"

namespace raisim {

template<class ChildEnvironment>
class VectorizedEnvironmentTerrainCurriculum : public VectorizedEnvironment {

 public:

  explicit VectorizedEnvironmentTerrainCurriculum(std::string resourceDir, std::string cfg)
      : VectorizedEnvironment(resourceDir, cfg),
        p_transition_(0.8f), p_replay_(0.05f), transition_step_size_(0.05), nb_particles_(10) 
  {
    if(&cfg_["p_transition"]) {
      p_transition_ = cfg_["p_transition"].template As<float>();
    }
    if(&cfg_["p_replay"]) {
      p_replay_ = cfg_["p_replay"].template As<float>();
    }
    if(&cfg_["transition_step_size"]) {
      transition_step_size_ = cfg_["transition_step_size"].template As<float>();
    }
    if(&cfg_["nb_particles"]) {
      nb_particles_ = cfg_["nb_particles"].template As<int>();
    }

    min_ << 0.0, 0.2, 0.0;
    max_ << 0.05, 1.0, 3.0;

    for (int i=0; i<nb_particles_; i++) {
      particules_[i].push_back(min_);
    }
  }

  ~VectorizedEnvironmentTerrainCurriculum() {
  }


 private:

  void updateTerrains() 
  {
    //compute weights
    Eigen::Matrix<float, -1, 1> traversabilities;
    traversabilities.setZeros(nb_particules);

    //get traversability from each environment
    for(int i=0; i<getNumOfEnvs(); i++) {
      int env_per_particle = int(getNumOfEnvs()/nb_particles);
      int current_particle = i/env_per_particle;
      traversabilities[current_particle] += environment_[i].getTraversability() / float(env_per_particle);
    }

    //update weights and sample.
    std::vector<Eigen::Vector3f> previous_particles_;
    float sum = traversabilities.sum();
    for (int i=0; i<nb_particles_; i++) {
      float sample = rn_.sampleUniform01() * sum;
      float particle_value = 0.;
      for (int j=0; j<nb_particles; j++) {
        particle_value += traversabilities[j];
        if (sample < particle_value) {
          replayBuffer_.push_back(previous_particles_[j]);
          particules_[i] = previous_particles_[j];
          break;
        }
      }
    }

    //replay and transition
    for (int i=0; i<nb_particles_; i++) {
      float p = rn_.sampleUniform01();
      if (p < p_replay_) {
        //sample from replay
        particles_[i] = replayBuffer[rn_.intRand(0, replayBuffer_.size())];
      }
      
      p = rn_.sampleUniform01();
      if ( p < p_transition_) {
        //add mutation
        Eigen::Vector3f direction;
        rn_.sampleOnUnitSphere(direction.data());
        direction = direction * (max_ - min_) * transition_step_size_;
        particles_[i] += direction;
      }
    }

    //update environments
    for(int i=0; i<getNumOfEnvs(); i++) {
      int env_per_particle = int(getNumOfEnvs()/nb_particles);
      int current_particle = i/env_per_particle;
      environent_[i].updateTerrain(TerrainType::Hills, particles_[i]);
    } 
  }

  std::vector<Eigen::Vector3f> replayBuffer_;
  std::vector<Eigen::Vector3f> particles_;

  Eigen::Vector3f min_, max_;
  float p_transition_, p_replay_, transition_step_size_;
  int nb_particles_;
  RandomNumberGererator<float> rn_;
};

}

