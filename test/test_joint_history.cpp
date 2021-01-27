//
// Created by Mathieu Geisert on 15/01/21.
// Test program for terrain generation class.

#include <raisim/World.hpp>
#include "raisim/RaisimServer.hpp"
#include "environment/motion/ModelParametersAnymalC100.hpp"
#include "environment/terrain/TerrainGenerator.hpp"
#include "environment/history/JointHistory.hpp"
#include <chrono> 

using namespace std::chrono; 

int main(int argc, char *argv[]) {

  std::string urdf_path = "/home/mgeisert/git/mmocrl/rsc/robot/c100/urdf/anymal_minimal.urdf";

  raisim::World* world = new raisim::World();
  raisim::RaisimServer* server = new raisim::RaisimServer(world);
  server->launchServer();
  raisim::ArticulatedSystem* anymal = world->addArticulatedSystem(urdf_path);

  ModelParametersAnymalC100<float> paramsC100;
  terrain::TerrainGenerator terrainGenerator(world, paramsC100);

  int gcDim = anymal->getGeneralizedCoordinateDim();
  int gvDim = anymal->getDOF();
  Eigen::VectorXd gc_init, gv_init, gc_ref;
  gc_init.setZero(gcDim);
  gv_init.setZero(gvDim);
  gc_ref.setZero(gcDim);
  gc_init << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  anymal->setState(gc_init, gv_init);

  JointVelPosErrorHistory<double, 20, 12> history(anymal);

  Eigen::Matrix<double, 100, 24> ref_history;

  for (int i=0; i<100; i++) {
    std::cout << "--- it: " << i  << " ---" << std::endl;
    if (i>10) {
      for (int j = 1; j<10; j++) {
        if ( (ref_history.row(i-j).transpose().head<12>() - history.getLastJointPositionError(j)).norm() > 1e-6) {
          std::cout << "Position history does not match. ref: " << ref_history.row(i-j).head<12>()
            << " -- history: " << history.getLastJointPositionError(j).transpose() << std::endl;
        }
        if ( (ref_history.row(i-j).transpose().tail<12>() - history.getLastJointVelocity(j)).norm() > 1e-6) {
          std::cout << "Velocity history does not match. ref: " << ref_history.row(i-j).tail<12>()
            << " -- history: " << history.getLastJointVelocity(j).transpose() << std::endl;
        }
      }
    }

    world->integrate();
    gc_ref = Eigen::VectorXd::Random(12);
    ref_history.row(i).head<12>().transpose() = gc_ref - anymal->getGeneralizedCoordinate().e().tail<12>();
    ref_history.row(i).tail<12>().transpose() = anymal->getGeneralizedVelocity().e().tail<12>();
    history.appendCurrentState(gc_ref);

  }
 
  std::cout << "End" << std::endl; 
  delete world;
  delete server;
  return 0;
}
