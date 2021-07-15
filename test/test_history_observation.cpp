//
// Created by Mathieu Geisert on 15/01/21.
// Test program for terrain generation class.

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>
#include "environment/terrain/Terrain.hpp"
#include "environment/motion/ModelParametersAnymalC100.hpp"
#include "environment/motion/FootMotionGenerator.hpp"
#include "environment/actuator/ActuatorModelPNetwork.hpp"
#include "environment/motion/IK.hpp"
#include "environment/terrain/ContactManager.hpp"
#include "common/RandomNumberGenerator.hpp"
#include "environment/disturbance/PushDisturbance.hpp"
#include "environment/observation/HistoryObservation.hpp"
#include "environment/command/Command.hpp"
#include "environment/command/CommandViewer.hpp"
#include <chrono> 

using namespace std::chrono; 

int main(int argc, char *argv[]) {

  std::string urdf_path = "/home/mgeisert/git/mmocrl/rsc/robot/c100/urdf/anymal_minimal.urdf";
  std::string actuator_network_path = "/home/mgeisert/git/mmocrl/rsc/actuator/C100/seaModel_2500.txt";
  
  raisim::World* world = new raisim::World();
  raisim::RaisimServer* server = new raisim::RaisimServer(world);
  server->launchServer();
  raisim::ArticulatedSystem* anymal = world->addArticulatedSystem(urdf_path);
  world->setTimeStep(0.0025);

  RandomNumberGenerator<float> rn;
  PushDisturbance disturbance(anymal, &rn);
  ModelParametersAnymalC100<float> c100Params;
  State<float> robotState(anymal);
  Terrain terrainGenerator(world, &c100Params, &rn);
  Command command(&robotState, &rn);
  CommandViewer commandViewer(server, &robotState, &command, &terrainGenerator);

  ContactManager contact(anymal, &robotState, &terrainGenerator, 0.0025);

  InverseKinematics IK(&c100Params);
  ActuatorModelPNetwork<float> actuator(anymal, actuator_network_path);

  FootMotionGenerator footMotion(&c100Params, &robotState, 2, 0.2, 0.0025);

  ScalingAndOffset<float> scaling_hist("/home/mgeisert/git/mmocrl/rsc/scaling/state_hist.yaml");
  HistoryObservation<float, 4, 100> obs(&robotState, &command, &footMotion, &actuator, &scaling_hist);

  //Init
  int gcDim = anymal->getGeneralizedCoordinateDim();
  int gvDim = anymal->getDOF();
  Eigen::VectorXf gc_init, gv_init;
  gc_init.setZero(gcDim);
  gv_init.setZero(gvDim); gv_init[0] = 0.5;
  gc_init.head(7) << 0, 0, 0.50, 0.707, 0.0, 0.0, 0.707;
  gc_init.tail(12) = c100Params.getReferenceJointConfiguration();
  anymal->setState(gc_init.template cast<double>(), gv_init.template cast<double>());
  
  Eigen::Vector3f sol;
  Eigen::Matrix<float, 12, 1> gc_target, foot_target;
  Eigen::Vector4f deltaFreq;
  deltaFreq.setZero();
  
  command.sampleGoal();
  command.updateCommand();
  robotState.advance();
  obs.reset();

  int max_it = 6000;
  Eigen::Matrix<float, 60, 1> it0, it20, it100;
  Eigen::VectorXd time(max_it);
  for (int i=0; i<max_it; i++) {
    auto start = high_resolution_clock::now();
    command.updateCommand();
    robotState.advance();
    foot_target = footMotion.advance(deltaFreq);
    for (int j = 0; j < 4; j++) {
      IK.IKSagittal(sol, foot_target.segment(3 * j, 3), j);
      gc_target.segment(3 * j, 3) = sol;
    }
    actuator.setTargetJointPosition(gc_target);
    actuator.advance();
    server->lockVisualizationServerMutex();
    world->integrate();
    commandViewer.advance();
    server->unlockVisualizationServerMutex();
    obs.advance();
    
    auto stop = high_resolution_clock::now();

    //std::cout << "Motion phase: " << obs.getObservation().transpose().<< std::endl;
    
    if (i==0) {
      it0 = obs.getObservation().head(60);
    }
    if (i==20) {
      it20 = obs.getObservation().head(60);
    }
    if (i==100) {
      it100 = obs.getObservation().head(60);
    }

    if (i>0 && i<100) {
      Eigen::Matrix<float, 60, 1> observation = obs.getObservation().segment<60>(60*i);
      if ( (observation - it0).norm() > 1e-6 ) {
        std::cout << "Hist not matching it0 -- it : " << i << std::endl;// << "obs: " << observation.transpose() << std::endl << "mem: " << it0.transpose() << std::endl;
      }
    }
    
    if (i>20 && i<120) {
      Eigen::Matrix<float, 60, 1> observation = obs.getObservation().segment<60>(60*(i-20));
      if ( (observation - it20).norm() > 1e-6 ) {
        std::cout << "Hist not matching it20 -- it : " << i << std::endl;//" -- observation: " << observation.transpose() <<
      }
    }

    if (i>100 && i<200) {
      Eigen::Matrix<float, 60, 1> observation = obs.getObservation().segment<60>(60*(i-100));
      if ( (observation - it100).norm() > 1e-6 ) {
        std::cout << "Hist not matching it100 -- it : " << i << std::endl;//" -- observation: " << observation.transpose() <<
      }
    }

    time[i] = std::chrono::duration<double>(duration_cast<microseconds>(stop - start)).count(); 
    usleep(2500);
    if (i%1200 == 0) {
      command.sampleGoal();
      std::cout << "s=" << i/400 << std::endl;
    }
  }
  std::cout << "mean time (us): " << time.mean() << std::endl;
 
  std::cout << "--- End ---" << std::endl;
  return 0;
}
