//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <cstdint>
#include <set>
#include "RaisimGymEnv.hpp"
#include <cmath>
#include "environment/terrain/Terrain.hpp"
#include "environment/motion/ModelParametersAnymalC100.hpp"
#include "environment/motion/FootMotionGenerator.hpp"
#include "environment/actuator/ActuatorModelPNetwork.hpp"
#include "environment/motion/IK.hpp"
#include "environment/terrain/ContactManager.hpp"
#include "common/RandomNumberGenerator.hpp"
#include "environment/disturbance/PushDisturbance.hpp"
#include "environment/observation/InternalObservation.hpp"
#include "environment/observation/PrivilegedObservation.hpp"
#include "environment/command/Command.hpp"
#include "environment/command/CommandViewer.hpp"
#include "environment/terrain/LocalTerrainViewer.hpp"

namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable) {

    /// add objects
    anymal_ = world_->addArticulatedSystem(resourceDir_+"/robot/c100/urdf/anymal_minimal.urdf");
    anymal_->setName("anymal");
    world_->setTimeStep(simulation_dt_);//0.0025);

    rn_ = std::make_shared<RandomNumberGenerator<float>>();
    disturbance_ = std::make_shared<PushDisturbance<float>>(anymal_, *rn_);
    c100Params_ = std::make_shared<ModelParametersAnymalC100<float>>();
    robotState_ = std::make_shared<State<float>>(anymal_);
    terrain_ = std::make_shared<Terrain<float, 4>>(world_.get(), *c100Params_, *rn_);
    command_ = std::make_shared<Command<float>>(*robotState_, *rn_);
    contact_ = std::make_shared<ContactManager<float, 4>>(anymal_, *robotState_, *terrain_, simulation_dt_);
    IK_ = std::make_shared<InverseKinematics<float, 4>>(*c100Params_);
    std::string actuator_network_path = resourceDir_+"/actuator/C100/seaModel_2500.txt";
    actuator_ = std::make_shared<ActuatorModelPNetwork<float>>(anymal_, actuator_network_path);
    footMotion_ = std::make_shared<FootMotionGenerator<float, 4>>(*c100Params_, *robotState_, 1.3, 0.2, control_dt_);
    privilegedObservation_ = std::make_shared<PrivilegedObservation<float, 4>>(*contact_, *terrain_, *robotState_, *disturbance_);
    stateObservation_ = std::make_shared<InternalObservation<float, 4>>(*robotState_, *command_, *footMotion_, *actuator_);
    stateScaling_ = std::make_shared<ScalingAndOffset<float>>(resourceDir_+"/scaling/state.yaml");
    actionScaling_ = std::make_shared<ScalingAndOffset<float>>(resourceDir_+"/scaling/action.yaml");

    /// this is nominal configuration of anymal
    gc_init_ << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
    gv_init_.setZero();

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 212; 
    actionDim_ = 16;

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile (cfg["reward"]);

    /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      commandViewer_ = std::make_shared<CommandViewer<float, 4>>(server_.get(), *robotState_, *command_, *terrain_);
      terrainViewer_ = std::make_shared<LocalTerrainViewer<float, 4>>(server_.get(), anymal_, *contact_);
      server_->launchServer();
      server_->focusOn(anymal_);
    }

    command_->sampleGoal();
    robotState_->advance();
    Eigen::Matrix<float, 12, 1> foot_target;
    Eigen::Matrix<float, 4, 1> deltaFreq; deltaFreq.setZero();
    foot_target = footMotion_->advance(deltaFreq);
    foot_target_hist2_ = foot_target;
    foot_target_hist1_ = foot_target;
  }

  void init() final { }

  void reset() final {
    gc_init_[2] = terrain_->getHeight(gc_init_[0], gc_init_[1]) + 0.5;
    anymal_->setState(gc_init_, gv_init_);
    actuator_->reset();
    footMotion_->reset();
    command_->sampleGoal();
    robotState_->advance();
    contact_->advance();

    Eigen::Matrix<float, 12, 1> foot_target;
    Eigen::Matrix<float, 4, 1> deltaFreq; deltaFreq.setZero();
    foot_target = footMotion_->advance(deltaFreq);
    foot_target_hist2_ = foot_target;
    foot_target_hist1_ = foot_target;
    badlyConditioned_ = false;
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    if (isnan(action.norm()) || isinf(action.norm())) {
      std::cout << "action badly conditioned: " << action.transpose() << std::endl;
      badlyConditioned_ = true;
    }
    
    Eigen::Matrix<float, 12, 1> gc_target, foot_target;
    Eigen::Matrix<float, 16, 1> actionScaled; actionScaled.setZero();

    actionScaled = actionScaling_->apply(action.template cast<float>());
    command_->updateCommand();
    foot_target = footMotion_->advance(actionScaled.head(4));
    foot_target += actionScaled.tail(12);
    gc_target = IK_->IKSagittal(foot_target);
    actuator_->setTargetJointPosition(gc_target);

    if(visualize_) server_->lockVisualizationServerMutex();
    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      actuator_->advance();
      disturbance_->advance(0.5);
      world_->integrate();
    }
    robotState_->advance();
    contact_->advance();
    if(visualize_) {
      commandViewer_->advance();
      terrainViewer_->advance();
      server_->unlockVisualizationServerMutex();
    }


    //Reward
    //desired base velocity direction
    Eigen::Vector2f vel = robotState_->getBaseVelInBaseFrame().head(2);
    double v_pr = vel.transpose() * command_->getCommand().head(2);
    double r_lv = 1.0;
    if (v_pr < 0.6)
      r_lv = exp(-2.0*pow(v_pr - 0.6, 2));
    rewards_.record("r_lv", r_lv);

    if (v_pr > 0.45) { P_traversability_++; } 
//    if(visualize_) {
//      it_++;
//      if (it_ % 10 == 0) { 
//        std::cout << "AngVel[2]: " << robotState_->getBaseAngVel()[2] << " - BaseVelInBaseFrame: " << vel.transpose()
//          << " - command: " << command_->getCommand().transpose() << " - v_pr: " << v_pr << " - r_lv: " << r_lv << std::endl;
//      }
//    }

    //Desired angular rotation yaw
    double w_pr = robotState_->getBaseAngVel()[2];
    w_pr -= command_->getCommand()[2];
    double r_av = 0.0;
    //if (w_pr < 0.6)
      r_av = exp(-1.5*pow(w_pr, 2)); //- 0.6, 2));
    rewards_.record("r_av", r_av);

    //No pitch/roll and no ortogonal velocity w.r.t commanded direction.
    double w = exp( -1.5 * pow(robotState_->getBaseAngVel().head(2).norm(), 2));
    Eigen::Vector3f vel3 = v_pr*command_->getCommand(); 
    vel3[2] = 0.; //set z vel to 0.
    double v_0 = exp( -1.5 * pow( (robotState_->getBaseVelInBaseFrame() - vel3).norm(),2));
    rewards_.record("r_b", v_0 + w);

    //Foot clearance.
    int Iswing = 0;
    int InotClear = 0;
    Eigen::Matrix<float, 4, 1> phases = footMotion_->getPhases();
    Eigen::Matrix<float, 6, 9> footHeightWrtTerrain = contact_->getFootHeightWrtLocalTerrain();
    for (int i=0; i<4; i++) {
      if (phases[i] > 0.0) {
        Iswing++;
        for (int j=0; j<footHeightWrtTerrain.rows(); j++) {
          if (footHeightWrtTerrain(i,j) < 0.03) {
            InotClear++;
            break;
          }
        }
      }
    }
    float r_fc = 0.5f;
    if (Iswing>0) {
      r_fc = float(Iswing - InotClear)/float(Iswing);
    }
    rewards_.record("r_fc", r_fc);

    //Smoothing
    Eigen::Matrix<float, 12, 1> smoothing = actionScaled.tail(12) - 2* foot_target_hist1_ + foot_target_hist2_;//foot_target - 2* foot_target_hist1_ + foot_target_hist2_;
    rewards_.record("r_s", - smoothing.norm());
    foot_target_hist2_ = foot_target_hist1_;
    foot_target_hist1_ = actionScaled.tail(12);//foot_target;
    
    //Torque
    rewards_.record("r_t", - anymal_->getGeneralizedForce().e().tail(12).squaredNorm());
    if(visualize_) {
      std::cout << "reward: " << rewards_.sum() 
        << "- r_lv: " << r_lv 
        << " - r_av: " << r_av 
        << " - r_b: " << v_0 + w 
        << " - r_fc: " << r_fc 
        << " - r_s: " << -smoothing.norm()
        << " - r_t: " << -anymal_->getGeneralizedForce().e().tail(12).squaredNorm()
        << std::endl;
      //usleep(100000);
    }

    it_++;
    if (it_ > 1000) {
      double traversability = double(P_traversability_) / double(it_);
      if (terrain_->getTerrainType() == TerrainType::Flat) {
        if (traversability > 0.95) {
          generateTerrainHills();
        }
//        if (traversability > 0.4) {
//          terrain_->setRandomFriction(0.2, 0.5);
//        }
      }
      else {
        if (traversability < 0.4 || traversability > 0.9) {
          generateTerrainHills();
        }
      }
      it_ = 0;
      P_traversability_ = 0;
    }
    
    return rewards_.sum();
  }

  void generateTerrainHills() {
    Eigen::Vector3d terrain_params;
    terrain_params[0] = rn_->sampleUniform01() * 0.05;
    terrain_params[1] = rn_->sampleUniform01() * 0.8 + 0.2;
    terrain_params[2] = rn_->sampleUniform01() * 2.8 + 0.2; 
    terrain_->generateTerrain(TerrainType::Hills, terrain_params);
    gc_init_[2] = terrain_->getHeight(gc_init_[0], gc_init_[1]) + 0.5;
    anymal_->setState(gc_init_, gv_init_);
//  terrain_->setRandomFriction(0.2, 0.5);
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    Eigen::Matrix<float, -1, 1> state = stateObservation_->getObservation();
    state = stateScaling_->apply(state);
    ob.head(state.size()) = state;
    ob.segment(state.size(), 79) = privilegedObservation_->getObservation();
    if (isnan(ob.norm()) || isinf(ob.norm())) {
      if (badlyConditioned_ == false) {
        std::cout << "observation badly conditioned even when action was good: " << ob.transpose() << std::endl;
        for (int i=0; i<ob.size(); i++) {
          if (isnan(ob[i]) || isinf(ob[i]))
            std::cout << "nan for i=" << i << std::endl;
        }
      }
    }

    //std::cout << ob.transpose() << std::endl;
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    if (contact_->isBaseContact())// || contact_->isInternalContact())
      return true;

    terminalReward = 0.f;
    return false;
  }

  void turnOffVisualization() { server_->hibernate(); visualize_ = false;}
  void turnOnVisualization() { server_->wakeup(); visualize_ = true;}

 private:
  std::shared_ptr<RandomNumberGenerator<float>> rn_;
  std::shared_ptr<PushDisturbance<float>> disturbance_;
  std::shared_ptr<ModelParametersAnymalC100<float>> c100Params_;
  std::shared_ptr<State<float>> robotState_;
  std::shared_ptr<Terrain<float, 4>> terrain_;
  std::shared_ptr<Command<float>> command_;
  std::shared_ptr<CommandViewer<float, 4>> commandViewer_;
  std::shared_ptr<ContactManager<float, 4>> contact_;
  std::shared_ptr<InverseKinematics<float, 4>> IK_;
  std::shared_ptr<ActuatorModelPNetwork<float>> actuator_;
  std::shared_ptr<FootMotionGenerator<float, 4>> footMotion_;
  std::shared_ptr<PrivilegedObservation<float, 4>> privilegedObservation_;
  std::shared_ptr<InternalObservation<float, 4>> stateObservation_;
  std::shared_ptr<ScalingAndOffset<float>> stateScaling_;
  std::shared_ptr<ScalingAndOffset<float>> actionScaling_;
  std::shared_ptr<LocalTerrainViewer<float, 4>> terrainViewer_;

  Eigen::Matrix<double, 19, 1> gc_init_;
  Eigen::Matrix<double, 18, 1> gv_init_;
  Eigen::Matrix<float, 12, 1> foot_target_hist1_;
  Eigen::Matrix<float, 12, 1> foot_target_hist2_;
  bool visualizable_ = false;
  raisim::ArticulatedSystem* anymal_;
  double terminalRewardCoeff_ = -1.;
  //raisim::Reward rewards_;
  bool badlyConditioned_ = false;
  int it_ = 0;
  int P_traversability_ = 0;
  bool visualize_ = false;
};
}

