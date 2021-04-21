/*
 * =====================================================================================
 *
 *       Filename:  ContactManager.hpp
 *
 *    Description:  Class to handle contacts.
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
#include "common/math.hpp"
#include "environment/terrain/Terrain.hpp"

//TODO: adjust footPos to avoid using hardcoded index for the collision bodies and footID.
//TODO: clean code relative to contact. add more getter if necessary.

template<typename T, int Nlimb>
class ContactManager {
public:
  ContactManager(raisim::ArticulatedSystem* anymal, const State<T>* robotState, const Terrain<T, Nlimb>* terrain, double simulation_dt)
   :anymal_(anymal), robotState_(robotState), terrain_(terrain), simulation_dt_(simulation_dt)
  {
    footNormal_.reserve(Nlimb);
    footNormal_b.reserve(Nlimb);
    footPos_.reserve(Nlimb);
    footVel_.reserve(Nlimb);
    footAngVel_.reserve(Nlimb);
    netFootContacts_.reserve(Nlimb);
    netFootContacts_b.reserve(Nlimb);
    netFootContactVels_.reserve(Nlimb);
    footPosInShankFrame_.reserve(Nlimb);
    footVelProjected_.reserve(Nlimb);

    for (int i=0; i<Nlimb; i++) {
      footPosInShankFrame_[i] = anymal_->getCollisionBodies()[4 * i + 4].posOffset;
    }
  } 

  ~ContactManager() = default;

  void advance(bool updateLocalTerrainInfo = true, bool updateContactsInfo = true)
  {
    if (updateLocalTerrainInfo)
      updateFootHeightWrtLocalTerrain();
    if (updateContactsInfo)
      updateContacts();
  }

  // Info priviledged state: FootPos_W - FHs, FootContactState, netFootContact_b, footNormal_b, footFriction, thighContacts, shankContacts.

  void updateContacts() {

    std::vector<size_t> FootContactNums; FootContactNums.resize(Nlimb);
   
    numContact_ = anymal_->getContacts().size();
    numFootContact_ = 0;
    numBaseContact_ = 0;
    numShankContact_ = 0;
    numThighContact_ = 0;
    numInternalContact_ = 0;

    for (int k = 0; k < Nlimb; k++) {
      footContacts_[k] = false;
      shankContacts_[k] = false;
      thighContacts_[k] = false;

      netFootContacts_[k].setZero();
      netFootContacts_b[k].setZero();
      netFootContactVels_[k].setZero();
      footContactNums_[k] = 0;
      footNormal_[k].setZero();
      footNormal_b[k] << 0.0, 0.0, 1.0;
    }
    //netContacts_.setZero();

    raisim::Vec<3> vec3;

    //position of the feet
    for (int k = 0; k < Nlimb; k++) {
      int footID = 3 * k + 3;
      anymal_->getPosition(footID, footPosInShankFrame_[k], footPos_[k]);
      anymal_->getVelocity(footID, footPosInShankFrame_[k], footVel_[k]);
      anymal_->getAngularVelocity(footID, footAngVel_[k]);
    }

    //Classify foot contact
    if (numContact_ > 0) {
      for (int k = 0; k < numContact_; k++) {
        if (!anymal_->getContacts()[k].skip() && anymal_->getContacts()[k].getPairObjectIndex() > 0) {

          int idx = anymal_->getContacts()[k].getlocalBodyIndex();
          //netContacts_ += anymal_->getContacts()[k].getImpulse()->e();

          if (idx == 0) {
            numBaseContact_++;
          } else if (idx == 3 || idx == 6 || idx == 9 || idx == 12) {
            int fid = idx / 3 - 1;
            double err = (footPos_[fid].e() - anymal_->getContacts()[k].getPosition().e()).norm();

            if (err < 0.035) {
              netFootContacts_[fid] +=
                  (anymal_->getContacts()[k].getContactFrame().e() * anymal_->getContacts()[k].getImpulse()->e()).template cast<T>();

              anymal_->getContactPointVel(k, vec3);
              netFootContactVels_[fid] += vec3.e().template cast<T>();
              footNormal_[fid] += anymal_->getContacts()[k].getNormal().e().template cast<T>();
              footContactNums_[fid]++;
              footContacts_[fid] = true;
              numFootContact_++;
            } else {
              numShankContact_++;
              shankContacts_[fid] = true;
            }

          } else if (idx == 2 || idx == 5 || idx == 8 || idx == 11) {
            int fid = idx / 3;
            numThighContact_++;
            thighContacts_[fid] = true;
          }
        } else {
          numInternalContact_++;
        }
      }
    }
   
    auto Rb = robotState_->getRotationMatrix();
    //netContacts_ /= simulation_dt_;
    //netContacts_b = Rb.transpose() * netContacts_;

    for (size_t i = 0; i < 4; i++) {
      if (footContactNums_[i] > 0) {
        netFootContactVels_[i] /= footContactNums_[i];
        netFootContacts_[i] /= simulation_dt_;
        netFootContacts_[i] = netFootContacts_[i].array().min(200.0); // For stability
        netFootContacts_[i] = netFootContacts_[i].array().max(-200.0); // For stability
        footNormal_[i].normalize();
        footNormal_b[i] = Rb.transpose() * footNormal_[i];
        netFootContacts_b[i] = Rb.transpose() * netFootContacts_[i];
        double scale = footNormal_[i].dot(footVel_[i].e().template cast<T>());
        footVelProjected_[i] = footVel_[i].e().template cast<T>() - scale * footNormal_[i];
      } else {
        footVelProjected_[i].setZero();
      }
    }
  }


  void  updateFootHeightWrtLocalTerrain(int nbPoint = 9, double footMargin = 0.13)
  {
    footHeightWrtLocalTerrain_.resize(Nlimb+2, nbPoint); // dz_foot1, ..., dz_footn, dx, dy
    footHeightWrtLocalTerrain_.setZero(); 

    //TODO: avoid recomputing angles? 
    auto Rb = robotState_->getRotationMatrix();
    Eigen::Matrix<T, 2, 1> x, y;
    x = footMargin * (Rb * robotState_->getXHorizontal()).template head<2>();
    y = footMargin * (Rb * robotState_->getYHorizontal()).template head<2>();
    
    float Angle = 2.0 * M_PI / (float) (nbPoint - 1);
    for (size_t i = 1; i < nbPoint; i++) {
      const float Cos = std::cos((i - 1) * Angle);
      const float Sin = std::sin((i - 1) * Angle);
      footHeightWrtLocalTerrain_(Nlimb, i) += (Cos * x[0]);
      footHeightWrtLocalTerrain_(Nlimb, i) += (Sin * y[0]);

      footHeightWrtLocalTerrain_(Nlimb+1, i) += (Cos * x[1]);
      footHeightWrtLocalTerrain_(Nlimb+1, i) += (Sin * y[1]);
    }

    raisim::Vec<3> footPosW;
    for (size_t fid = 0; fid < Nlimb; fid++) {
      int footID = 3 * fid + 3;
      anymal_->getPosition(footID, footPosInShankFrame_[fid], footPosW);
      for (size_t k = 0; k < nbPoint; k++) {
        footHeightWrtLocalTerrain_(fid, k) = footPosW[2] -
            terrain_->getHeight(footPosW[0] + footHeightWrtLocalTerrain_(Nlimb, k), footPosW[1] + footHeightWrtLocalTerrain_(Nlimb+1, k));
      }
    }
  }

  const std::vector<Eigen::Matrix<T, 3, 1>>& getNetFootContactsInBase() const { return netFootContacts_b; }
  const std::vector<Eigen::Matrix<T, 3, 1>>& getFootNormalsInBase() const { return footNormal_b; }
  const std::array<bool, Nlimb>& getFootContacts() const { return footContacts_; }
  const std::array<bool, Nlimb>& getShankContacts() const { return shankContacts_; }
  const std::array<bool, Nlimb>& getThighContacts() const { return thighContacts_; }
  const Eigen::Matrix<T, Nlimb+2, -1>& getFootHeightWrtLocalTerrain() const { return footHeightWrtLocalTerrain_; }
  bool isBaseContact() const { return numBaseContact_ > 0; }
  bool isInternalContact() const { return numInternalContact_ > 0; }

protected:
  Eigen::Matrix<double, 3, 1> xHorizontal_, yHorizontal_; //in body frame

  std::vector<raisim::Vec<3>> footPosInShankFrame_;
  std::vector<raisim::Vec<3>> footPos_;
  std::vector<raisim::Vec<3>> footVel_;
  std::vector<Eigen::Matrix<T, 3, 1>> footVelProjected_;
  std::vector<raisim::Vec<3>> footAngVel_;
  
  std::vector<Eigen::Matrix<T, 3, 1>> footNormal_;
  std::vector<Eigen::Matrix<T, 3, 1>> footNormal_b;
  
  std::array<bool, Nlimb> footContacts_;
  std::array<bool, Nlimb> shankContacts_;
  std::array<bool, Nlimb> thighContacts_;

  size_t numContact_;
  size_t numFootContact_;
  size_t numShankContact_;
  size_t numThighContact_;
  size_t numBaseContact_;
  size_t numInternalContact_;
  std::vector<Eigen::Matrix<T, 3, 1>> netFootContacts_;
  std::vector<Eigen::Matrix<T, 3, 1>> netFootContacts_b;
  std::vector<Eigen::Matrix<T, 3, 1>> netFootContactVels_;
//  Eigen::Vector3d netContacts_;
//  Eigen::Vector3d netContacts_b;
  Eigen::Matrix<uint, 1, Nlimb> footContactNums_;

  Eigen::Matrix<T, Nlimb+2, -1> footHeightWrtLocalTerrain_; // z_foot1, ..., z_footn, dx, dy
  
  raisim::ArticulatedSystem* anymal_;
  const State<T>* robotState_;
  const Terrain<T, Nlimb>* terrain_;
  double simulation_dt_;
}; // end of class State

