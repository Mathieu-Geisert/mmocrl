/*
 * =====================================================================================
 *
 *       Filename:  ScalingAndOffset.hpp
 *
 *    Description:  Managment off sclalings and offsets. Save/Load weigths from yaml files.
 *
 *        Version:  1.0
 *        Created:  18/01/21 19:07:51
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Mathieu Geisert, 
 *   Organization:  
 *
 * =====================================================================================
 */

#include "common/message_macros.hpp"
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <fstream>

template<typename T>
class ScalingAndOffset {

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    ScalingAndOffset(Eigen::Matrix<T, -1, 1> scaling, Eigen::Matrix<T, -1, 1> offset)
    : scaling_(scaling),
      offset_(offset) 
    {
      FATAL_IF(scaling.size() != offset.size(), "scaling and offset should have the same size...");
    }

    ScalingAndOffset(std::string file_path) 
    {
      YAML::Node yamlParams = YAML::LoadFile(file_path);
      FATAL_IF(!yamlParams["scaling"], "[ScalingAndOffset] scaling not found in the yaml file.");
      FATAL_IF(!yamlParams["offset"], "[ScalingAndOffset] offset not found in the yaml file.");
      std::vector<T> scaling = yamlParams["scaling"].as<std::vector<T>>();
      std::vector<T> offset = yamlParams["offset"].as<std::vector<T>>();
      scaling_.setZero(scaling.size());
      offset_.setZero(offset.size());
      for (int i=0; i<scaling.size(); i++) {
        scaling_[i] = scaling[i];
        offset_[i] = offset[i];
      }      
    }

    ~ScalingAndOffset() = default;

    Eigen::Matrix<T, -1, 1> apply(const Eigen::Matrix<T, -1, 1>& input)
    {
      FATAL_IF(input.size() != scaling_.size(), "input vector does not have the same size as scaling/offset...");
      return (input - offset_).cwiseProduct(scaling_);
    }

    void setScaling(const Eigen::Matrix<T, -1, 1>& scaling) { scaling_ = scaling; }
    const Eigen::Matrix<T, -1, 1>& getScaling() { return scaling_; }
    void setOffset(const Eigen::Matrix<T, -1, 1>& offset) { offset_ = offset; }
    const Eigen::Matrix<T, -1, 1>& getOffset() { return offset_; }

    void saveToFile(std::string file_path)
    {
      YAML::Node yamlParams;
      for (int i=0; i<scaling_.size(); i++) {
        yamlParams["scaling"].push_back(scaling_[i]);
        yamlParams["offset"].push_back(offset_[i]);
      }
      std::ofstream file(file_path);
      file << yamlParams;
      file.close();
    }

  protected:
    Eigen::Matrix<T, -1, 1> scaling_;
    Eigen::Matrix<T, -1, 1> offset_;
};
