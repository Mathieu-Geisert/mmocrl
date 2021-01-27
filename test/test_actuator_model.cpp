//
// Created by Mathieu Geisert on 15/01/21.
// Test program for terrain generation class.

#include <raisim/World.hpp>
#include "raisim/RaisimServer.hpp"
#include "environment/terrain/TerrainGenerator.hpp"
#include "environment/actuator/ActuatorModelPD.hpp"
#include "environment/actuator/ActuatorModelPNetwork.hpp"
#include "environment/motion/ModelParametersAnymalC100.hpp"
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

  ModelParametersAnymalC100<float> paramsC100;
  terrain::TerrainGenerator terrainGenerator(world, paramsC100);

  int gcDim = anymal->getGeneralizedCoordinateDim();
  int gvDim = anymal->getDOF();
  Eigen::VectorXd gc_init, gv_init, gc_ref;
  gc_init.setZero(gcDim);
  gv_init.setZero(gvDim);
  gc_ref.setZero(gcDim);
  gc_init << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  int max_it = 2000;
  Eigen::VectorXd time(max_it);

  anymal->setState(gc_init, gv_init);
  ActuatorModelPD<double> actuator1(anymal);
  std::cout << "--- Test PD gains double, no actuator model ---" << std::endl;
  sleep(1);
  for (int i=0; i<max_it; i++) {
    //std::cout << "inter: " << i << std::endl;
    auto start = high_resolution_clock::now();
    actuator1.setTargetJointPosition(gc_init.tail<12>());
    actuator1.advance();
    auto stop = high_resolution_clock::now();
    time[i] = std::chrono::duration<double>(duration_cast<microseconds>(stop - start)).count(); 
    server->lockVisualizationServerMutex();
    world->integrate();
    server->unlockVisualizationServerMutex();
    usleep(2500);
    if (i%400 == 0)
      std::cout << "s=" << i/400 << std::endl;
  }
  std::cout << "mean time (us): " << time.mean() << std::endl;
  
  anymal->setState(gc_init, gv_init);
  ActuatorModelPD<float> actuator2(anymal);
  std::cout << "--- Test PD gains float, no actuator model ---" << std::endl;
  sleep(1);
  for (int i=0; i<max_it; i++) {
    //std::cout << "inter: " << i << std::endl;
    auto start = high_resolution_clock::now();
    actuator2.setTargetJointPosition(gc_init.tail<12>().template cast<float>());
    actuator2.advance();
    auto stop = high_resolution_clock::now();
    time[i] = std::chrono::duration<double>(duration_cast<microseconds>(stop - start)).count(); 
    server->lockVisualizationServerMutex();
    world->integrate();
    server->unlockVisualizationServerMutex();
    usleep(2500);
    if (i%400 == 0)
      std::cout << "s=" << i/400 << std::endl;
  }
  std::cout << "mean time (us): " << time.mean() << std::endl;

  ActuatorModelPNetwork<double> actuator3(anymal, actuator_network_path);
  anymal->setState(gc_init.template cast<double>(), gv_init.template cast<double>());
  std::cout << "--- Test actuator network position double---" << std::endl;
  sleep(1);
  for (int i=0; i<max_it; i++) {
    //std::cout << "inter: " << i << std::endl;
    auto start = high_resolution_clock::now();
    actuator3.setTargetJointPosition(gc_init.tail<12>());
    actuator3.advance();
    auto stop = high_resolution_clock::now();
    time[i] = std::chrono::duration<double>(duration_cast<microseconds>(stop - start)).count(); 
    server->lockVisualizationServerMutex();
    world->integrate();
    server->unlockVisualizationServerMutex();
    usleep(2500);
    if (i%400 == 0)
      std::cout << "s=" << i/400 << std::endl;
  }
  std::cout << "mean time (us): " << time.mean() << std::endl;
  
  ActuatorModelPNetwork<float> actuator4(anymal, actuator_network_path);
  anymal->setState(gc_init.template cast<double>(), gv_init.template cast<double>());
  std::cout << "--- Test actuator network position float---" << std::endl;
  sleep(1);
  for (int i=0; i<max_it; i++) {
    //std::cout << "inter: " << i << std::endl;
    auto start = high_resolution_clock::now();
    actuator4.setTargetJointPosition(gc_init.tail<12>().template cast<float>());
    actuator4.advance();
    auto stop = high_resolution_clock::now();
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
  delete server;
  delete world;
  return 0;
}
