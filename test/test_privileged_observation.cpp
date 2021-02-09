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
#include "environment/observation/PrivilegedObservation.hpp"
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

  int npoint = 9;
  server->lockVisualizationServerMutex();
  std::vector<raisim::Visuals*> terrainVisual;
  for (int i=0; i<4; i++) {
    for (int j=0; j<npoint; j++) {
      terrainVisual.push_back(server->addVisualSphere("foot"+std::to_string(i)+"_p"+std::to_string(j), 0.1 , 1., 0., 0., 1.));
    }
  }

  server->unlockVisualizationServerMutex();

  RandomNumberGenerator<float> rn;
  PushDisturbance disturbance(anymal, rn);
  ModelParametersAnymalC100<float> c100Params;
  State<float> robotState(anymal);
  Terrain terrainGenerator(world, c100Params, rn);

  std::array<float, 9> array;
  //Eigen::Matrix3f::Map(array.data()) = robotState.getRotationMatrix();
  //for (int i=0; i<9; i++)
  //  std::cout << array[i] << std::endl;

  ContactManager contact(anymal, robotState, terrainGenerator, 0.0025);
  PrivilegedObservation<float, 4> observation(contact, terrainGenerator, robotState, disturbance);

  int gcDim = anymal->getGeneralizedCoordinateDim();
  int gvDim = anymal->getDOF();
  Eigen::VectorXf gc_init, gv_init;
  gc_init.setZero(gcDim);
  gv_init.setZero(gvDim);
  gc_init.head(7) << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0;
  gc_init.tail(12) = c100Params.getReferenceJointConfiguration();
  anymal->setState(gc_init.template cast<double>(), gv_init.template cast<double>());
  
  InverseKinematics IK(c100Params);
  ActuatorModelPNetwork<float> actuator(anymal, actuator_network_path);

  FootMotionGenerator footMotion(c100Params, robotState, 2, 0.2, 0.0025);
  Eigen::Vector3f e_g, sol;
  e_g.setZero();
  e_g[2] = 1.;
   
  Eigen::Matrix<float, 12, 1> gc_target, foot_target;
  Eigen::Vector4f deltaFreq;
  deltaFreq.setZero();
  
  int max_it = 3000;
  Eigen::VectorXd time(max_it);
  for (int i=0; i<max_it; i++) {
    //std::cout << "inter: " << i << std::endl;
    auto start = high_resolution_clock::now();
    robotState.advance();
    foot_target = footMotion.advance(deltaFreq);
    auto stop = high_resolution_clock::now();
    for (int j = 0; j < 4; j++) {
      IK.IKSagittal(sol, foot_target.segment(3 * j, 3), j);
      gc_target.segment(3 * j, 3) = sol;
    }
    actuator.setTargetJointPosition(gc_target);
    actuator.advance();
    time[i] = std::chrono::duration<double>(duration_cast<microseconds>(stop - start)).count(); 
    server->lockVisualizationServerMutex();
    contact.advance();
    auto obs = observation.getObservation();
    if (i%500 == 0)
      std::cout << obs.transpose() << std::endl;
    Eigen::Matrix<float, 6, -1> dfoot = contact.getFootHeightWrtLocalTerrain();
    raisim::Vec<3> footPosW;
    for (size_t fid = 0; fid < 4; fid++) {
      int footID = 3 * fid + 3;
      raisim::Vec<3> footPos =anymal->getCollisionBodies()[4 * fid + 4].posOffset;
      //std::cout << "footPos " << fid << " : " << footPos[0] << " " << footPos[1] << " " << footPos[2] << std::endl; 
      anymal->getPosition(footID, footPos, footPosW);
      for (int fp=0; fp<npoint; fp++) {
        raisim::Vec<3> pos;
        pos.setZero();
       pos[0] = dfoot(4, fp);
       pos[1] = dfoot(5, fp);
       pos[2] = -dfoot(fid, fp);
        pos += footPosW;
        terrainVisual[fid*npoint + fp]->setPosition(pos[0], pos[1], pos[2]);
      }
    }
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
