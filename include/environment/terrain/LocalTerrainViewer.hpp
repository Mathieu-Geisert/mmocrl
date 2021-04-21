/*
 * =====================================================================================
 *
 *       Filename:  LocalTerrainViewer.hpp
 *
 *    Description:  Class to handle visualize computation of terrain height around the feet in raisimUnity.
 *                  Code based from RSL published sources of 
 *                  "Learning Quadrupedal Locomotion Over Challenging Terrain"
 *
 *        Version:  1.0
 *        Created:  14/01/21 11:00:51
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Mathieu Geisert, 
 *   Organization:  
 *
 * =====================================================================================
 */

#pragma once

#include "common/message_macros.hpp"
#include "environment/terrain/ContactManager.hpp"

constexpr double Size = 0.15;

template<typename T, int Nlimb>
class LocalTerrainViewer {
public:
  LocalTerrainViewer(raisim::RaisimServer* server, raisim::ArticulatedSystem* anymal, const ContactManager<T, Nlimb>* contact) 
  : server_(server), anymal_(anymal), contact_(contact)
  {
  }
  
  ~LocalTerrainViewer() = default;

  void advance()
  {
    const Eigen::Matrix<T, 6, -1>& dfoot = contact_->getFootHeightWrtLocalTerrain();
    if (dfoot.cols() > terrainVisual_.size())
        resize(dfoot.cols());
    raisim::Vec<3> footPosW;
    for (size_t fid = 0; fid < 4; fid++) {
      int footID = 3 * fid + 3;
      raisim::Vec<3> footPos =anymal_->getCollisionBodies()[4 * fid + 4].posOffset;
      anymal_->getPosition(footID, footPos, footPosW);
      for (int fp=0; fp<dfoot.cols(); fp++) {
        raisim::Vec<3> pos;
        pos.setZero();
        pos[0] = dfoot(4, fp);
        pos[1] = dfoot(5, fp);
        pos[2] = -dfoot(fid, fp);
        pos += footPosW;
        terrainVisual_[fid*dfoot.cols() + fp]->setPosition(pos[0], pos[1], pos[2]);
      }
    }
  }

protected:
  void resize(int newSize)
  {
    int prevSize = terrainVisual_.size()/Nlimb; 
    for (int i=0; i<Nlimb; i++) {
      for (int j=0; j<newSize-prevSize; j++) {
        terrainVisual_.push_back(server_->addVisualSphere("foot"+std::to_string(i)+"_p"+std::to_string(j), Size , 1., 0., 0., 1.));
      }
    }
  }

  raisim::RaisimServer* server_;
  raisim::ArticulatedSystem* anymal_;
  const ContactManager<T, Nlimb>* contact_;
  std::vector<raisim::Visuals*> terrainVisual_;
}; // end of class LocalTerrainViewer

