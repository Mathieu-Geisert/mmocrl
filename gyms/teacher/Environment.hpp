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
    world_->setTimeStep(0.0025);

    rn_ = std::make_shared<RandomNumberGenerator<double>>();
    disturbance_ = std::make_shared<PushDisturbance<double>>(anymal_, *rn_);
    c100Params_ = std::make_shared<ModelParametersAnymalC100<double>>();
    robotState_ = std::make_shared<State<double>>(anymal_);
    terrain_ = std::make_shared<Terrain<double, 4>>(world_.get(), *c100Params_, *rn_);
    command_ = std::make_shared<Command<double>>(*robotState_, *rn_);
    contact_ = std::make_shared<ContactManager<double, 4>>(anymal_, *robotState_, *terrain_, simulation_dt_);
    IK_ = std::make_shared<InverseKinematics<double, 4>>(*c100Params_);
    std::string actuator_network_path = "/home/mgeisert/git/mmocrl/rsc/actuator/C100/seaModel_2500.txt";
    actuator_ = std::make_shared<ActuatorModelPNetwork<double>>(anymal_, actuator_network_path);
    footMotion_ = std::make_shared<FootMotionGenerator<double, 4>>(*c100Params_, *robotState_, 1.3, 0.2, control_dt_);
    privilegedObservation_ = std::make_shared<PrivilegedObservation<double, 4>>(*contact_, *terrain_, *robotState_, *disturbance_);
    stateObservation_ = std::make_shared<InternalObservation<double, 4>>(*robotState_, *command_, *footMotion_, *actuator_);
    stateScaling_ = std::make_shared<ScalingAndOffset<double>>("/home/mgeisert/git/mmocrl/rsc/scaling/state.yaml");
    actionScaling_ = std::make_shared<ScalingAndOffset<double>>("/home/mgeisert/git/mmocrl/rsc/scaling/action.yaml");

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
      commandViewer_ = std::make_shared<CommandViewer<double, 4>>(server_.get(), *robotState_, *command_, *terrain_);
      terrainViewer_ = std::make_shared<LocalTerrainViewer<double, 4>>(server_.get(), anymal_, *contact_);
      server_->launchServer();
      server_->focusOn(anymal_);
    }

    command_->sampleGoal();
    robotState_->advance();
    Eigen::Matrix<double, 12, 1> foot_target;
    Eigen::Matrix<double, 4, 1> deltaFreq; deltaFreq.setZero();
    foot_target = footMotion_->advance(deltaFreq);
    foot_target_hist2_ = foot_target;
    foot_target_hist1_ = foot_target;
  }

  void init() final { }

  void reset() final {
    anymal_->setState(gc_init_, gv_init_);
    actuator_->reset();
    footMotion_->reset();
    command_->sampleGoal();
    robotState_->advance();
    contact_->advance();

    Eigen::Matrix<double, 12, 1> foot_target;
    Eigen::Matrix<double, 4, 1> deltaFreq; deltaFreq.setZero();
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
    Eigen::Matrix<double, 12, 1> gc_target, foot_target;
    Eigen::Matrix<double, 16, 1> actionScaled; actionScaled.setZero();
    Eigen::Matrix<double, 3, 1> sol;

    actionScaled = actionScaling_->apply(action.template cast<double>());
    command_->updateCommand();
    //std::cout << "command: " << command_->getCommand().transpose() << std::endl;
    foot_target = footMotion_->advance(actionScaled.head(4));
    foot_target += actionScaled.tail(12);
    for (int j = 0; j < 4; j++) {
      IK_->IKSagittal(sol, foot_target.segment(3 * j, 3), j);
      gc_target.segment(3 * j, 3) = sol;
    }
    gc_target = IK_->IKSagittal(foot_target);

    actuator_->setTargetJointPosition(gc_target);

    if(visualize_) server_->lockVisualizationServerMutex();
    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      actuator_->advance();
      world_->integrate();
    }
    robotState_->advance();
    contact_->advance();
    if(visualize_) {
      commandViewer_->advance();
      terrainViewer_->advance();
      server_->unlockVisualizationServerMutex();
//      it_++;
//      if (it_ % 10 == 0) { 
//        std::cout << "AngVel: " << robotState_->getBaseAngVel().transpose() << std::endl;
//      }
    }


    //Reward
    Eigen::Vector2d vel = robotState_->getBaseVelInBaseFrame().head(2);
    double v_pr = vel.transpose() * command_->getCommand().head(2);
    double r_lv = 1.0;
    if (v_pr < 0.6)
      r_lv = exp(-2.0*pow(v_pr - 0.6, 2));
    rewards_.record("r_lv", r_lv);
    if(visualize_) {
      it_++;
      if (it_ % 10 == 0) { 
        std::cout << "AngVel[2]: " << robotState_->getBaseAngVel()[2] << " - BaseVelInBaseFrame: " << vel.transpose()
          << " - command: " << command_->getCommand().transpose() << " - v_pr: " << v_pr << " - r_lv: " << r_lv << std::endl;
      }
    }

    double w_pr = robotState_->getBaseAngVel()[2];
    w_pr -= command_->getCommand()[2];
    double r_av = 1.0;
    if (w_pr < 0.6)
      r_av = exp(-1.5*pow(w_pr, 2)); //- 0.6, 2));
//    if (command_->getCommand()[2] == 0.)
//    r_av = 0.0;
    rewards_.record("r_av", r_av);

    double w = exp( -1.5 * pow(robotState_->getBaseAngVel().head(2).norm(), 2));
    double v_0 = 0.;//exp( -1.5 * pow( (vel - v_pr*command_->getCommand().head(2)).norm(),2));
    rewards_.record("r_b", v_0 + w);

    Eigen::Matrix<double, 12, 1> smoothing = foot_target - 2* foot_target_hist1_ + foot_target_hist2_;
    rewards_.record("r_s", - smoothing.norm());
    foot_target_hist2_ = foot_target_hist1_;
    foot_target_hist1_ = foot_target;
    
    rewards_.record("r_t", - anymal_->getGeneralizedForce().squaredNorm());
    
    return rewards_.sum();
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    Eigen::Matrix<double, -1, 1> state = stateObservation_->getObservation();
    state = stateScaling_->apply(state);
    ob.head(state.size()) = state.template cast<float>();
    ob.segment(state.size(), 79) = privilegedObservation_->getObservation().template cast<float>();
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
  std::shared_ptr<RandomNumberGenerator<double>> rn_;
  std::shared_ptr<PushDisturbance<double>> disturbance_;
  std::shared_ptr<ModelParametersAnymalC100<double>> c100Params_;
  std::shared_ptr<State<double>> robotState_;
  std::shared_ptr<Terrain<double, 4>> terrain_;
  std::shared_ptr<Command<double>> command_;
  std::shared_ptr<CommandViewer<double, 4>> commandViewer_;
  std::shared_ptr<ContactManager<double, 4>> contact_;
  std::shared_ptr<InverseKinematics<double, 4>> IK_;
  std::shared_ptr<ActuatorModelPNetwork<double>> actuator_;
  std::shared_ptr<FootMotionGenerator<double, 4>> footMotion_;
  std::shared_ptr<PrivilegedObservation<double, 4>> privilegedObservation_;
  std::shared_ptr<InternalObservation<double, 4>> stateObservation_;
  std::shared_ptr<ScalingAndOffset<double>> stateScaling_;
  std::shared_ptr<ScalingAndOffset<double>> actionScaling_;
  std::shared_ptr<LocalTerrainViewer<double, 4>> terrainViewer_;

  Eigen::Matrix<double, 19, 1> gc_init_;
  Eigen::Matrix<double, 18, 1> gv_init_;
  Eigen::Matrix<double, 12, 1> foot_target_hist1_;
  Eigen::Matrix<double, 12, 1> foot_target_hist2_;
  bool visualizable_ = false;
  raisim::ArticulatedSystem* anymal_;
  double terminalRewardCoeff_ = -10.;
  raisim::Reward rewards_;
  bool badlyConditioned_ = false;
  int it_ = 0;
  bool visualize_ = false;
};
}
