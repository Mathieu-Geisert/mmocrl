/*
 * =====================================================================================
 *
 *       Filename:  TerrainParticuleFilter.hpp
 *
 *    Description: Particule Filter of Terrain Currilum learning with several type of terrains. 
 *
 *        Version:  1.0
 *        Created:  29/06/21 12:29:56
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Mathieu Geisert, 
 *   Organization:  
 *
 * =====================================================================================
 */


#pragma once

//#include "VectorizedEnvironment.hpp"
#include "common/RandomNumberGenerator.hpp"

//For now use fixed friction distribution.
template<class ChildEnvironment>
class TerrainParticleFilter
{
  public:
    TerrainParticleFilter(const std::vector<ChildEnvironment *>& environments, TerrainType terrainType, Eigen::Vector3f min, Eigen::Vector3f max, Eigen::Vector2f friction, int nb_particles = 10) :
      environments_(environments), terrainType_(terrainType),
      min_(min), max_(max), friction_(friction),
      p_transition_(0.8f), p_replay_(0.05f), transition_step_size_(0.20), nb_particles_(nb_particles) 
    {
      if ( environments.size() % nb_particles != 0) {
        std::cout << "[TerrainParticleFilter] size of environments needs to be a multiple of the nb of particles..." << std::endl;
        exit(0);
      }
      init();
    }

    ~TerrainParticleFilter() {};

  void init() {
    replayBuffer_.clear();
    particles_.clear(); 
    particles_.reserve(nb_particles_);
    for (int i=0; i<nb_particles_; i++) {
      particles_.push_back(min_);
    }
  }

  const std::vector<Eigen::Vector3f>& getParticules() { return particles_; }
    
  void advance() 
  {
    //compute weights
    Eigen::Matrix<float, -1, 1> traversabilities, means; //mean values is just used for information.
    traversabilities.setZero(nb_particles_);
    means.setZero(nb_particles_);

    //get traversability from each environment
    //Each evironment compute a traversability value (whatever the number of trajectory).
    //traversability probability (of being between 0.4 and 0.9) is computed from the value of each enviroment.
    //This means we expect several environments per particle to be able to estimate the probability.
    int env_per_particle = int(environments_.size()/nb_particles_);
    for(int i=0; i<environments_.size(); i++) {
      int current_particle = i/env_per_particle;
      //std::cout << "traversability: " << environments_[0]->getTraversability() << std::endl;
      float traversability = environments_[i]->getTraversability();
      //std::cout << "traversalibity: " << traversability << std::endl;
      means[current_particle] += double(traversability) / float(env_per_particle);
      if (traversability > 0.4 && traversability < 0.85)
        traversabilities[current_particle] += 1 / float(env_per_particle);
    }

    std::cout << "Terrain: " << int(terrainType_) << std::endl;
    for (int i=0; i<nb_particles_; i++) {
      std::cout << "particule: " << particles_[i].transpose() << " -- traversabilities: " << traversabilities[i] << " -- means: " << means[i] << std::endl;
    }

    //update weights and sample.
    std::vector<Eigen::Vector3f> previous_particles = particles_;
    float sum = traversabilities.sum();
    for (int i=0; i<nb_particles_; i++) {
      float sample = rn_.sampleUniform01() * sum;
      float particle_value = 0.;
      for (int j=0; j<nb_particles_; j++) {
        particle_value += traversabilities[j];
        if (sample <= particle_value) {
          addToReplayBuffer(previous_particles[j]);
          particles_[i] = previous_particles[j];
          break;
        }
      }
    }

    //replay and transition
    for (int i=0; i<nb_particles_; i++) {
      float p = rn_.sampleUniform01();
      if (p < p_replay_) {
        //sample from replay
        particles_[i] = replayBuffer_[rn_.intRand(0, replayBuffer_.size())];
      }
      else if ( p < p_transition_ + p_replay_ ) { 
        //add mutation
        Eigen::Vector3f direction;
        rn_.sampleOnUnitSphere<3>(direction.data());
        direction = /* rn_.sampleUniform01() * */ transition_step_size_ * direction.cwiseProduct((max_ - min_));
        particles_[i] += direction;
        for (int j=0; j<3; j++) {
          if (particles_[i][j] < min_[j]) { particles_[i][j] = min_[j]; } 
          if (particles_[i][j] > max_[j]) { particles_[i][j] = max_[j]; } 
        }
      }
    }

    //update environments
    for(int i=0; i<environments_.size(); i++) {
      int current_particle = i/env_per_particle;
      environments_[i]->updateTerrain(terrainType_, particles_[current_particle], friction_);
    } 
  }

  void addToReplayBuffer(const Eigen::Vector3f& params) 
  {
    //Only add terrain if different from previous ones.
    for (int i=0; i<replayBuffer_.size(); i++) {
      if ( (params - replayBuffer_[i]).norm() < 0.01 )
        return;
    }
    replayBuffer_.push_back(params);
  }

  private:
    std::vector<ChildEnvironment *> environments_;
    TerrainType terrainType_;
    std::vector<Eigen::Vector3f> replayBuffer_;
    std::vector<Eigen::Vector3f> particles_;

    Eigen::Vector3f min_, max_;
    Eigen::Matrix<float, 2, 1> friction_;
    float p_transition_, p_replay_, transition_step_size_;
    int nb_particles_;
    RandomNumberGenerator<float> rn_;
};
