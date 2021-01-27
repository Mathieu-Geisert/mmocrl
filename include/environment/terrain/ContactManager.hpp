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
#include "environment/terrain/TerrainGenerator.hpp"

//TODO: adjust footPos to avoid using hardcoded index for the collision bodies and footID.
//TODO: clean code relative to contact. add more getter if necessary.
//TODO: clean footposOffset?

template<typename T, int Nlimb>
class ContactManager {
public:
  ContactManager(raisim::ArticulatedSystem* anymal, const State<T>& robotState, const terrain::TerrainGenerator<T, Nlimb>& terrain, double simulation_dt)
   :anymal_(anymal), robotState_(robotState), terrain_(terrain), simulation_dt_(simulation_dt)
  {
    footNormal_.resize(Nlimb);
    footNormal_b.resize(Nlimb);
    footPos_.resize(Nlimb);
  } 

  ~ContactManager() = default;


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
      FootContactNums_[k] = 0;
      footNormal_[k].setZero();
      footNormal_b[k] << 0.0, 0.0, 1.0;
    }
    //netContacts_.setZero();

    raisim::Vec<3> vec3;

    //position of the feet
    for (int k = 0; k < Nlimb; k++) {
      int footID = 3 * k + 3;
      raisim::Vec<3> footPosInShankFrame = anymal_->getCollisionBodies()[4 * k + 4].posOffset;
      anymal_->getPosition(footID, footPosInShankFrame, footPos_[k]);
      anymal_->getVelocity(footID, footPosInShankFrame, footVel_[k]);
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
                  (anymal_->getContacts()[k].getContactFrame().e() * anymal_->getContacts()[k].getImpulse()->e());

              anymal_->getContactPointVel(k, vec3);
              netFootContactVels_[fid] += vec3.e();
              footNormal_[fid] += anymal_->getContacts()[k].getNormal().e();
              FootContactNums_[fid]++;
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
   
    auto Rb = robotState_.getRotationMatrix();
    //netContacts_ /= simulation_dt_;
    //netContacts_b = Rb.transpose() * netContacts_;

    for (size_t i = 0; i < 4; i++) {
      if (FootContactNums_[i] > 0) {
        netFootContactVels_[i] /= FootContactNums_[i];
        netFootContacts_[i] /= simulation_dt_;
        netFootContacts_[i] = netFootContacts_[i].array().min(200.0); // For stability
        netFootContacts_[i] = netFootContacts_[i].array().max(-200.0); // For stability
        footNormal_[i].normalize();
        footNormal_b[i] = Rb.transpose() * footNormal_[i];
        netFootContacts_b[i] = Rb.transpose() * netFootContacts_[i];
        double scale = footNormal_[i].dot(footVel_[i].e());
        footVelProjected_[i] = footVel_[i].e() - scale * footNormal_[i];
      } else {
        footVelProjected_[i].setZero();
      }
    }
  }


  Eigen::Matrix<T, Nlimb+2, -1>  getFootHeightWrtLocalTerrain(int nbPoint = 9, double footMargin = 0.13)
  {
    Eigen::Matrix<T, Nlimb+2, -1> out(Nlimb+2, nbPoint); // dz_foot1, ..., dz_footn, dx, dy
    out.setZero(); 

    //TODO: avoid recomputing angles? 
    auto Rb = robotState_.getRotationMatrix();
    Eigen::Matrix<T, 2, 1> x, y;
    x = footMargin * (Rb * robotState_.getXHorizontal()).template head<2>();
    y = footMargin * (Rb * robotState_.getYHorizontal()).template head<2>();
    
    float Angle = 2.0 * M_PI / (float) (nbPoint - 1);
    for (size_t i = 1; i < nbPoint; i++) {
      const float Cos = std::cos((i - 1) * Angle);
      const float Sin = std::sin((i - 1) * Angle);
      out(Nlimb, i) += (Cos * x[0]);
      out(Nlimb, i) += (Sin * y[0]);

      out(Nlimb+1, i) += (Cos * x[1]);
      out(Nlimb+1, i) += (Sin * y[1]);
    }

    raisim::Vec<3> footPosW;
    for (size_t fid = 0; fid < Nlimb; fid++) {
      int footID = 3 * fid + 3;
      raisim::Vec<3> footPosInShankFrame = anymal_->getCollisionBodies()[4 * fid + 4].posOffset;
      anymal_->getPosition(footID, footPosInShankFrame, footPosW);
      for (size_t k = 0; k < nbPoint; k++) {
        out(fid, k) = footPosW[2] -
            terrain_.getHeight(footPosW[0] + out(Nlimb, k), footPosW[1] + out(Nlimb+1, k));
      }
    }

    return out;
  }

  const std::vector<Eigen::Matrix<T, 3, 1>>& getNetFootContactsInBase() const { return netFootContacts_b; }
  const std::vector<Eigen::Matrix<T, 3, 1>>& getFootNormalInBase() const { return footNormal_b; }
  const std::array<bool, Nlimb>& getShankContacts() const { return shankContacts_; }
  const std::array<bool, Nlimb>& getThighContacts() const { return thighContacts_; }

  protected:
    Eigen::Matrix<double, 3, 1> xHorizontal_, yHorizontal_; //in body frame
  
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
//    Eigen::Vector3d netContacts_;
//    Eigen::Vector3d netContacts_b;
    std::vector<size_t> FootContactNums_;
  
    //Eigen::Matrix<T, Nlimb+2, -1> terrainLocalHeight_; // z_foot1, ..., z_footn, dx, dy
    
    raisim::ArticulatedSystem* anymal_;
    const State<T>& robotState_;
    const terrain::TerrainGenerator<T, Nlimb>& terrain_;
    double simulation_dt_;
}; // end of class State

