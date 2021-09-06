//
// Created by Mathieu Geisert on 15/01/21.
// Test program for terrain generation class.

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>
#include "environment/terrain/Terrain.hpp"
#include "environment/motion/ModelParametersA1.hpp"
#include "environment/motion/FootMotionGenerator.hpp"
#include "environment/actuator/ActuatorModelPNetwork.hpp"
#include "environment/motion/IK.hpp"
#include "environment/observation/State.hpp"
#include "common/RandomNumberGenerator.hpp"
#include <chrono> 
#include <unistd.h>

using namespace std::chrono; 

int main(int argc, char *argv[]) {

  std::string urdf_path = "/home/mgeisert/raisim_ws/src/mmocrl/rsc/robot/a1/urdf/a1.urdf";
  std::string actuator_network_path = "/home/mgeisert/raisim_ws/src/mmocrl/rsc/actuator/C100/seaModel_2500.txt";
  
  raisim::World* world = new raisim::World();
  raisim::RaisimServer* server = new raisim::RaisimServer(world);
  server->launchServer();
  raisim::ArticulatedSystem* a1 = world->addArticulatedSystem(urdf_path);
  world->setTimeStep(0.0025);

  RandomNumberGenerator<float> rn;
  ModelParametersA1<float> a1Params;
  Terrain terrainGenerator(world, &a1Params, &rn);
  InverseKinematics IK(&a1Params);
  ActuatorModelPNetwork<float> actuator(a1, actuator_network_path);

  int gcDim = a1->getGeneralizedCoordinateDim();
  int gvDim = a1->getDOF();
  Eigen::VectorXf gc_init, gv_init;
  gc_init.setZero(gcDim);
  gv_init.setZero(gvDim);
  gc_init.head(7) << 0, 0, 0.300, 1.0, 0.0, 0.0, 0.0;
  gc_init.tail(12) = a1Params.getReferenceJointConfiguration();
  a1->setState(gc_init.template cast<double>(), gv_init.template cast<double>());

  State<float> robotState(a1);
  FootMotionGenerator footMotion(&a1Params, &robotState, 3.0, 0.0, 0.0025);
  Eigen::Vector3f e_g, sol;
  e_g.setZero();
  e_g[2] = 1.;

  std::cout << "reference offset: " << a1Params.getReferenceFootPositionOffset().transpose() << std::endl; 
  Eigen::Matrix<float, 12, 1> gc_target, foot_target;
  Eigen::Vector4f deltaFreq;
  deltaFreq.setZero();
  
  int max_it = 8000;
  Eigen::VectorXd time(max_it);
  for (int i=0; i<max_it; i++) {
    //std::cout << "inter: " << i << std::endl;
    auto start = high_resolution_clock::now();
    robotState.updateState();
    foot_target = footMotion.advance(deltaFreq);
    std::cout << footMotion.getPhases()[0] << " -- " << foot_target.transpose() << std::endl;
    auto stop = high_resolution_clock::now();
    for (int j = 0; j < 4; j++) {
      IK.IKSagittal(sol, foot_target.segment(3 * j, 3), j);
      if (j>1) {
        //sol[1] += 3.14/2;
        sol[2] += 0.0;
      }
      gc_target.segment(3 * j, 3) = sol;
    }
    gc_init[2] = 0.45;
    gc_init.tail(12) = gc_target;
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(std::sin(footMotion.getPhases()[0])*0.6, Eigen::Vector3d::UnitX());
    //q = Eigen::AngleAxisd(std::sin(0)*0.6, Eigen::Vector3d::UnitX());
    gc_init[3] = q.w();
    gc_init[4] = q.x();
    gc_init[5] = q.y();
    gc_init[6] = q.z();
    //actuator.setTargetJointPosition(gc_target);
    //actuator.advance();
    a1->setState(gc_init.template cast<double>(), gv_init.template cast<double>());
    time[i] = std::chrono::duration<double>(duration_cast<microseconds>(stop - start)).count();
    //a1->setState(gc_init.template cast<double>(), gv_init.template cast<double>());
    server->lockVisualizationServerMutex();
    world->integrate();
    server->unlockVisualizationServerMutex();
    usleep(2500*4);
    if (i%400 == 0)
      std::cout << "s=" << i/400 << std::endl;
  }
  std::cout << "mean time (us): " << time.mean() << std::endl;
 
  std::cout << "--- End ---" << std::endl;
  return 0;
}
