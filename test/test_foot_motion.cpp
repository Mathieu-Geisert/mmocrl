//
// Created by Mathieu Geisert on 15/01/21.
// Test program for terrain generation class.

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>
#include "environment/terrain/TerrainGenerator.hpp"
#include "environment/motion/ModelParametersAnymalC100.hpp"
#include "environment/motion/FootMotionGenerator.hpp"
#include "environment/actuator/ActuatorModelPNetwork.hpp"
#include "environment/motion/IK.hpp"
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

  terrain::TerrainGenerator terrainGenerator(world);
  ModelParametersAnymalC100<float> c100Params;
  InverseKinematics IK(c100Params);
  ActuatorModelPNetwork<float> actuator(anymal, actuator_network_path);

  int gcDim = anymal->getGeneralizedCoordinateDim();
  int gvDim = anymal->getDOF();
  Eigen::VectorXf gc_init, gv_init;
  gc_init.setZero(gcDim);
  gv_init.setZero(gvDim);
  gc_init.head(7) << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0;
  gc_init.tail(12) = c100Params.getReferenceJointConfiguration();
  anymal->setState(gc_init.template cast<double>(), gv_init.template cast<double>());
 
  FootMotionGenerator footMotion(c100Params, 2, 0.2, 0.0025);
  Eigen::Vector3f e_g, sol;
  e_g.setZero();
  e_g[2] = 1.;
   
  Eigen::Matrix<float, 12, 1> gc_target, foot_target;
  Eigen::Vector4f deltaFreq;
  deltaFreq.setZero();
  
  int max_it = 4000;
  Eigen::VectorXd time(max_it);
  for (int i=0; i<max_it; i++) {
    //std::cout << "inter: " << i << std::endl;
    auto start = high_resolution_clock::now();
    foot_target = footMotion.advance(deltaFreq, e_g);
    auto stop = high_resolution_clock::now();
    for (int j = 0; j < 4; j++) {
      IK.IKSagittal(sol, foot_target.segment(3 * j, 3), j);
      gc_target.segment(3 * j, 3) = sol;
    }
    actuator.setTargetJointPosition(gc_target);
    actuator.advance();
    time[i] = std::chrono::duration<double>(duration_cast<microseconds>(stop - start)).count(); 
    server->lockVisualizationServerMutex();
    world->integrate();
    server->unlockVisualizationServerMutex();
    usleep(2500);
    if (i%400 == 0)
      std::cout << "s=" << i/400 << std::endl;
  }
  std::cout << "mean time (us): " << time.mean() << std::endl;
 
  std::cout << "--- End ---" << std::endl;
  return 0;
}
