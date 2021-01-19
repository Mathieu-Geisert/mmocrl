/*
 * =====================================================================================
 *
 *       Filename:  TerrainGenerator.hpp
 *
 *    Description:  Class to handle generation of the terrains.
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

#include "common/message_macro.hpp"

namespace terrain {

enum TerrainType {
  Flat_,
  Hills,
  Steps,
  Stairs,
  SingleStep,
  UniformSlope
};

class TerrainGenerator {
 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    TerrainGenerator() 
    {
    }

    ~TerrainGenerator() = default;

  protected:

}; // end of clann TerrainGenerator
} // end of namespace terrain

