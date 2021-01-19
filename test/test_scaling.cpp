//
// Created by Mathieu Geisert on 15/01/21.
// Test program for terrain generation class.

#include <environment/scaling/ScalingAndOffset.hpp>
#include <environment/motion/IK_c100.hpp>
#include <chrono> 

using namespace std::chrono; 

int main(int argc, char *argv[]) {

  double freqScale_ = 0.0025 * 2.0 * M_PI; // 1 Hz 0.0157
  double h0_ = -0.55;
  Eigen::VectorXd footPositionOffset_, jointNominalConfig_;
  Eigen::VectorXf stateOffset_, stateScale_;
  jointNominalConfig_.setZero(12);
  footPositionOffset_.setZero(12);
  stateOffset_.setZero(133);
  stateScale_.setZero(133);
  

  footPositionOffset_ << 0.3 + 0.1, 0.2, h0_,
                         0.3 + 0.1, -0.2, h0_,
                         -0.3 - 0.1, 0.2, h0_,
                         -0.3 - 0.1, -0.2, h0_;

  InverseKinematics IK_;
  Eigen::Vector3d sol;
  for (int i = 0; i < 4; i++) {
    IK_.IKSagittal(sol, footPositionOffset_.segment(3 * i, 3), i);
    jointNominalConfig_.segment(3 * i, 3) = sol;
  }

  /// state params
  stateOffset_ << 0.0, 0.0, 0.0, /// command
      0.0, 0.0, 1.0, /// gravity axis
      Eigen::VectorXf::Constant(6, 0.0), /// body lin/ang vel
      jointNominalConfig_.template cast<float>(), /// joint position
      Eigen::VectorXf::Constant(12, 0.0),
      Eigen::VectorXf::Constant(12, 0.0),
      Eigen::VectorXf::Constant(4, 0.0), //52
      Eigen::VectorXf::Constant(8, 0.0), // 60
      Eigen::VectorXf::Constant(24, 0.0), // 84
      Eigen::VectorXf::Constant(24, 0.0), // 108
      jointNominalConfig_.template cast<float>(), /// joint position
      jointNominalConfig_.template cast<float>(), /// joint position
      0.0; // 132

  stateScale_ << 1.5, 1.5, 1.5, /// command
      5.0, 5.0, 5.0, /// gravity axis
      Eigen::VectorXf::Constant(3, 2.0),
      Eigen::VectorXf::Constant(3, 2.0),
      Eigen::VectorXf::Constant(12, 2.0), /// joint angles
      0.5, 0.4, 0.3,
      0.5, 0.4, 0.3,
      0.5, 0.4, 0.3,
      0.5, 0.4, 0.3,
      6.5, 4.5, 3.5,
      6.5, 4.5, 3.5,
      6.5, 4.5, 3.5,
      6.5, 4.5, 3.5,
      Eigen::VectorXf::Constant(4, 2.0 / freqScale_),
      Eigen::VectorXf::Constant(8, 1.5),
      Eigen::VectorXf::Constant(24, 5.0), /// joint position errors
      0.5, 0.4, 0.3,
      0.5, 0.4, 0.3,
      0.5, 0.4, 0.3,
      0.5, 0.4, 0.3,
      0.5, 0.4, 0.3,
      0.5, 0.4, 0.3,
      0.5, 0.4, 0.3,
      0.5, 0.4, 0.3,
      Eigen::VectorXf::Constant(12, 2.0), /// prev. action
      Eigen::VectorXf::Constant(12, 2.0),
      2.0 / freqScale_;

  //
  std::cout << "Constructor from vector." << std::endl;
  ScalingAndOffset<float> state(stateScale_, stateOffset_);
  std::cout << "Write file" << std::endl;

  std::string file_path = "/home/mgeisert/git/mmocrl/rsc/scaling/test.yaml";
  state.saveToFile(file_path);

  std::cout << "Constructor from file." << std::endl;
  ScalingAndOffset<float> state2(file_path);

  if ( (state2.getScaling() - stateScale_).norm() > 1e-3)
    std::cout << "--- Diff between input/output scaling ---\nin:  " << stateScale_.transpose() 
      << "\nout: " << state2.getScaling().transpose() 
      << "\ndiff:" << stateScale_.transpose() - state2.getScaling().transpose() << std::endl;
  if ( (state2.getOffset() - stateOffset_).norm() > 1e-3)
    std::cout << "--- Diff between input/output offset ---\nin:  " << stateOffset_.transpose() 
      << "\nout: " << state2.getOffset().transpose()
      << "\ndiff:" << stateOffset_.transpose() - state2.getOffset().transpose() << std::endl;
  
  std::cout << "--- End ---" << std::endl;
  return 0;
}
