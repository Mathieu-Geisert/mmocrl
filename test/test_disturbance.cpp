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
#include "environment/disturbance/PushDisturbanceViewer.hpp"
#include "environment/observation/InternalObservation.hpp"
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
  ModelParametersAnymalC100<float> c100Params;
  State<float> robotState(anymal);
  Terrain terrainGenerator(world, c100Params, rn);
  Command command(robotState, rn);
  CommandViewer commandViewer(server, robotState, command, terrainGenerator);
  PushDisturbance disturbance(anymal, rn);
  PushDisturbanceViewer disturbanceViewer(server, robotState, disturbance);

  ContactManager contact(anymal, robotState, terrainGenerator, 0.0025);

  InverseKinematics IK(c100Params);
  ActuatorModelPNetwork<float> actuator(anymal, actuator_network_path);

  FootMotionGenerator footMotion(c100Params, robotState, 2, 0.2, 0.0025);

  InternalObservation obs(robotState, command, footMotion, actuator);

  //Init
  int gcDim = anymal->getGeneralizedCoordinateDim();
  int gvDim = anymal->getDOF();
  Eigen::VectorXf gc_init, gv_init;
  gc_init.setZero(gcDim);
  gv_init.setZero(gvDim);
  gc_init.head(7) << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0;
  gc_init.tail(12) = c100Params.getReferenceJointConfiguration();
  anymal->setState(gc_init.template cast<double>(), gv_init.template cast<double>());
  
  Eigen::Vector3f sol;
  Eigen::Matrix<float, 12, 1> gc_target, foot_target;
  Eigen::Vector4f deltaFreq;
  deltaFreq.setZero();
  
  command.sampleGoal();
  command.updateCommand();
  
  disturbance.advance(1.);
  Eigen::Vector3f force = disturbance.getDisturbance();
  std::cout << "Initial disturbance: " << force.transpose() << std::endl;

  int max_it = 4000;
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
    disturbance.advance(1.);
    if (force != disturbance.getDisturbance()) {
      force = disturbance.getDisturbance();
      std::cout << "It : " << i << " -- New disturbance: " << force.transpose() << std::endl;
    }

    server->lockVisualizationServerMutex();
    world->integrate();
    commandViewer.advance();
    disturbanceViewer.advance();
    server->unlockVisualizationServerMutex();
    usleep(2500);
  }
 
  std::cout << "--- End ---" << std::endl;
  return 0;
}
