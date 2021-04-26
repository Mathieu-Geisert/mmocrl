//----------------------------// 
// Author: Mathieu Geisert.   //
//----------------------------//

#pragma once

#include "VectorizedEnvironment.hpp"
#include "common/RandomNumberGenerator.hpp"

namespace raisim {

template<class ChildEnvironment>
class VectorizedEnvironmentTerrainCurriculum : public VectorizedEnvironment<ChildEnvironment> {

 public:

  explicit VectorizedEnvironmentTerrainCurriculum(std::string resourceDir, std::string cfg)
      : VectorizedEnvironment<ChildEnvironment>(resourceDir, cfg),
        p_transition_(0.8f), p_replay_(0.05f), transition_step_size_(0.20), nb_particles_(10) 
  {
//    if(&this->cfg_["p_transition"]) {
//      p_transition_ = this->cfg_["p_transition"].template As<float>();
//    }
//    if(&this->cfg_["p_replay"]) {
//      p_replay_ = this->cfg_["p_replay"].template As<float>();
//    }
//    if(&this->cfg_["transition_step_size"]) {
//      transition_step_size_ = this->cfg_["transition_step_size"].template As<float>();
//    }
//    if(&this->cfg_["nb_particles"]) {
//      nb_particles_ = this->cfg_["nb_particles"].template As<int>();
//    }

    min_ << 0.0, 0.2, 0.2;
    max_ << 0.05, 1.0, 3.0;
  }


  void init() {
    VectorizedEnvironment<ChildEnvironment>::init();
    replayBuffer_.clear();
    particles_.clear(); particles_.reserve(nb_particles_);
    for (int i=0; i<nb_particles_; i++) {
      particles_.push_back(min_);
    }
    for(int i=0; i<this->getNumOfEnvs(); i++) {
      int env_per_particle = int(this->getNumOfEnvs()/nb_particles_);
      int current_particle = i/env_per_particle;
      this->environments_[i]->updateTerrain(particles_[current_particle]);
    } 
  }

  ~VectorizedEnvironmentTerrainCurriculum() {
  }

  void updateTerrains() 
  {
    //compute weights
    Eigen::Matrix<float, -1, 1> traversabilities, means;
    traversabilities.setZero(nb_particles_);
    means.setZero(nb_particles_);

    //get traversability from each environment
    for(int i=0; i<this->getNumOfEnvs(); i++) {
      int env_per_particle = int(this->getNumOfEnvs()/nb_particles_);
      int current_particle = i/env_per_particle;
      float traversability = this->environments_[i]->getTraversability();
      means[current_particle] += double(traversability) / float(env_per_particle);
      if (traversability > 0.4 && traversability < 0.9)
        traversabilities[current_particle] += 1 / float(env_per_particle);
    }

    for (int i=0; i<nb_particles_; i++) {
      std::cout << "terrain: " << particles_[i].transpose() << " -- traversabilities: " << traversabilities[i] << " -- means: " << means[i] << std::endl;
    }

    //update weights and sample.
    std::vector<Eigen::Vector3f> previous_particles = particles_;
    float sum = traversabilities.sum();
    //keep particle 0 = flat terrain.
    for (int i=0; i<nb_particles_; i++) {
      float sample = rn_.sampleUniform01() * sum;
      float particle_value = 0.;
      for (int j=0; j<nb_particles_; j++) {
        particle_value += traversabilities[j];
        if (sample <= particle_value) {
          replayBuffer_.push_back(previous_particles[j]);
          particles_[i] = previous_particles[j];
          break;
        }
      }
    }

    //replay and transition
    //keep particle 0 = flat terrain.
    for (int i=0; i<nb_particles_; i++) {
      float p = rn_.sampleUniform01();
      if (p < p_replay_) {
        //sample from replay
        particles_[i] = replayBuffer_[rn_.intRand(0, replayBuffer_.size())];
      }
      
      p = rn_.sampleUniform01();
      if ( p < p_transition_) {
        //add mutation
        Eigen::Vector3f direction;
        rn_.sampleOnUnitSphere<3>(direction.data());
        direction = direction.cwiseProduct((max_ - min_) * transition_step_size_);
        particles_[i] += direction;
        for (int j=0; j<3; j++) {
          if (particles_[i][j] < min_[j]) { particles_[i][j] = min_[j]; } 
          if (particles_[i][j] > max_[j]) { particles_[i][j] = max_[j]; } 
        }
      }
    }

    //update environments
    for(int i=0; i<this->getNumOfEnvs(); i++) {
      int env_per_particle = int(this->getNumOfEnvs()/nb_particles_);
      int current_particle = i/env_per_particle;
      this->environments_[i]->updateTerrain(particles_[current_particle]);
    } 
  }

 private:
  std::vector<Eigen::Vector3f> replayBuffer_;
  std::vector<Eigen::Vector3f> particles_;

  Eigen::Vector3f min_, max_;
  float p_transition_, p_replay_, transition_step_size_;
  int nb_particles_;
  RandomNumberGenerator<float> rn_;
};

}

