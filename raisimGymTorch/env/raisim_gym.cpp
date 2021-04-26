//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "Environment.hpp"
#include "VectorizedEnvironmentTerrainCurriculum.hpp"

namespace py = pybind11;
using namespace raisim;

#ifndef ENVIRONMENT_NAME
  #define ENVIRONMENT_NAME RaisimGymEnv
#endif

typedef VectorizedEnvironmentTerrainCurriculum<ENVIRONMENT> VecEnv;

PYBIND11_MODULE(RAISIMGYM_TORCH_ENV_NAME, m) {
  py::class_<VecEnv>(m, RSG_MAKE_STR(ENVIRONMENT_NAME))
    .def(py::init<std::string, std::string>(), py::arg("resourceDir"), py::arg("cfg"))
    .def("init", &VecEnv::init)
    .def("reset", &VecEnv::reset)
    .def("updateTerrains", &VecEnv::updateTerrains)
    .def("observe", &VecEnv::observe)
    .def("step", &VecEnv::step)
    .def("setSeed", &VecEnv::setSeed)
    .def("rewardInfo", &VecEnv::getRewardInfo)
    .def("close", &VecEnv::close)
    .def("isTerminalState", &VecEnv::isTerminalState)
    .def("setSimulationTimeStep", &VecEnv::setSimulationTimeStep)
    .def("setControlTimeStep", &VecEnv::setControlTimeStep)
    .def("getObDim", &VecEnv::getObDim)
    .def("getActionDim", &VecEnv::getActionDim)
    .def("getNumOfEnvs", &VecEnv::getNumOfEnvs)
    .def("turnOnVisualization", &VecEnv::turnOnVisualization)
    .def("turnOffVisualization", &VecEnv::turnOffVisualization)
    .def("stopRecordingVideo", &VecEnv::stopRecordingVideo)
    .def("startRecordingVideo", &VecEnv::startRecordingVideo)
    .def("curriculumUpdate", &VecEnv::curriculumUpdate)
    .def(py::pickle(
        [](const VecEnv &p) { // __getstate__ --> Pickling to Python
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(p.getResourceDir(), p.getCfgString());
        },
        [](py::tuple t) { // __setstate__ - Pickling from Python
            if (t.size() != 2) {
              throw std::runtime_error("Invalid state!");
            }

            /* Create a new C++ instance */
            VecEnv p(t[0].cast<std::string>(), t[1].cast<std::string>());

            return p;
        }
    ));
}
