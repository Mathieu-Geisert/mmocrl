//
// Created by Mathieu Geisert on 15/01/21.
// Test program for terrain generation class.

#include <raisim/World.hpp>
#include "raisim/RaisimServer.hpp"
#include "environment/terrain/TerrainGenerator.hpp"
#include <chrono> 
using namespace std::chrono; 

int main(int argc, char *argv[]) {

  raisim::World* world = new raisim::World();
  raisim::RaisimServer* server = new raisim::RaisimServer(world);
  server->launchServer();
 
  terrain::TerrainGenerator terrainGenerator(world);

  Eigen::Matrix<double, 3, 1> params;
 
  for (int i=0; i<30; i++) {
    params = Eigen::VectorXd::Random(3);
    params.normalize();
    for (int j=0;j<3;j++) {
      if (params[j]<0)
        params[j] *= -1.;
    }
    std::cout << params.transpose() << std::endl;
    auto start = high_resolution_clock::now();
    terrainGenerator.generateTerrain(terrain::TerrainType(rand()%7), params); 
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start); 
    std::cout << "terrainGeneration duration (ms): " << duration.count() << std::endl;
    sleep(3);
  }
  delete server;
  delete world;
  return 0;
}
