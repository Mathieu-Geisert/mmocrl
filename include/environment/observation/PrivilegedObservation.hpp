/*
 * =====================================================================================
 *
 *       Filename:  PrivilegedObservation.hpp
 *
 *    Description:  Class to handle privileged Information only (i.e. without base/joint info).
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
#include "environment/terrain/Terrain.hpp"
#include "environment/disturbance/PushDisturbance.hpp"
#include "environment/observation/State.hpp"
  
  // Info priviledged privilegedObservation: state, FootPos_W - FHs, FootContactState, netFootContact_b, footNormal_b, footFriction, thighContacts, shankContacts, base disturbance.

// TODO: better hangle scaling, offset, bounds.

template<typename T, int Nlimb>
class PrivilegedObservation 
{
public:

  //constexpr int PrivilegedprivilegedObservationDim = StateDim + sampleN + 4 + 12 + 12 + 4 + 8 + 3; // StateDim = 133, sampleN = 9*Nlimb
  //static constexpr int ObservationSize = 

  PrivilegedObservation(const ContactManager<T, Nlimb>& contact, const Terrain<T, Nlimb>& terrain, const State<T>& robotState, const PushDisturbance<T>& disturbance)
  : contact_(contact), terrain_(terrain), robotState_(robotState), disturbance_(disturbance)
  {
  }

  ~PrivilegedObservation() = default;

  Eigen::Matrix<T, -1, 1> getObservation() {
    Eigen::Matrix<T, -1, 1> privilegedObservation;

    const auto& footPosWrtTerrain = contact_.getFootHeightWrtLocalTerrain();
    int nPointTerrain = footPosWrtTerrain.cols();
    int observationSize = Nlimb * nPointTerrain + Nlimb*10 + 3;
    privilegedObservation.resize(observationSize,1);
    
    int idx = 0;
    //terrain information
    for (int i=0; i<Nlimb; i++) {
      privilegedObservation.segment(idx, nPointTerrain) = footPosWrtTerrain.row(i);
      //Scaling, Offset and bounds
      for (int j=idx; j<idx+nPointTerrain; j++) {
        privilegedObservation[j] = std::max(std::min(privilegedObservation[j], 0.25f), -0.25f);
        privilegedObservation[j] -= 0.05;
        privilegedObservation[j] *= 10.0;
      }
      idx += nPointTerrain;
    }

    //FootContactprivilegedObservation
    const auto& footContacts = contact_.getFootContacts();
    for (int i=0; i<Nlimb; i++) {
      privilegedObservation[idx++] = (footContacts[i] - 0.5) * 3.;
    }

    //netFootContact_b
    const auto& netFootContacts = contact_.getNetFootContactsInBase();
    for (int i=0; i<Nlimb; i++) {
      privilegedObservation.template segment<3>(idx) = netFootContacts[i];
      //Scaling, Offset and bounds
      privilegedObservation[idx + 2] -= 80.0;
      privilegedObservation[idx] = std::max(std::min(privilegedObservation[idx], 50.0f), -50.0f);
      privilegedObservation[idx + 1] = std::max(std::min(privilegedObservation[idx+1], 50.0f), -50.0f);
      privilegedObservation[idx + 2] = std::max(std::min(privilegedObservation[idx+2], 100.0f), -100.0f);
      privilegedObservation[idx] *= 0.01;
      privilegedObservation[idx + 1] *= 0.01;
      privilegedObservation[idx + 2] *= 0.02;
      idx += 3;
    }

    //FootNormal_b
    const auto& footNormals = contact_.getFootNormalsInBase();
    for (int i=0; i<Nlimb; i++) {
      privilegedObservation.template segment<3>(idx) = footNormals[i];
      //Scaling, Offset and bounds
      privilegedObservation[idx] *= 5.0;
      privilegedObservation[idx + 1] *= 5.0;
      privilegedObservation[idx + 2] -= 1.0;
      privilegedObservation[idx + 2] *= 20.0;
      idx += 3;
    }

    //FootFriction
    const auto& footFrictions = terrain_.getFootFriction();
    for (int i=0; i<Nlimb; i++) {
      privilegedObservation[idx++] = (footFrictions[i] - 0.6) * 2.0;
    }

    //ThighContacts
    const auto& thighContacts = contact_.getThighContacts();
    for (int i=0; i<Nlimb; i++) {
      privilegedObservation[idx++] = (thighContacts[i] - 0.5) * 2.0;
    }

    //ShankContacts
    const auto& shankContacts = contact_.getShankContacts();
    for (int i=0; i<Nlimb; i++) {
      privilegedObservation[idx++] = (shankContacts[i] - 0.5) * 2.0;
    }

    //disturbance on base.
    const auto& Rb = robotState_.getRotationMatrix();
    const auto& push = disturbance_.getDisturbance();
    privilegedObservation.template segment<3>(idx) = 0.1 * Rb.transpose() * push;

    return privilegedObservation;
  }

protected:
  const ContactManager<T, Nlimb>& contact_;
  const Terrain<T, Nlimb>& terrain_;
  const State<T>& robotState_;
  const PushDisturbance<T>& disturbance_;
}; // end of class PrivilegedObservation

