cmake_minimum_required( VERSION 3.10 )
project(mmocrl)
set(CMAKE_CXX_STANDARD 17)

find_package(raisim CONFIG REQUIRED)
find_package(TensorFlow CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)
#find_package(Eigen3 REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

if (TensorFlow_FOUND)
    message(STATUS "Found Tensorflow!")
    list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${TensorFlow_INCLUDE_DIRS})
    list(APPEND ${PROJECT_NAME}_LIBRARIES ${TensorFlow_LIBRARIES})
endif()

#if(Eigen3_FOUND)
#  message("Eigen3 found!")
#  message(${EIGEN3_INCLUDE_DIRS})
#endif()



include_directories(include
        ${${PROJECT_NAME}_INCLUDE_DIRS}
        )
      #{EIGEN3_INLCUDE_DIRS})

add_executable(test_terrain_generator
        test/test_terrain_generator.cpp)
target_link_libraries(test_terrain_generator
        ${${PROJECT_NAME}_LIBRARIES} raisim::raisim)
target_compile_options(test_terrain_generator PRIVATE -mtune=native -fPIC -O3 -g -mno-avx2)

add_executable(test_joint_history
        test/test_joint_history.cpp)
target_link_libraries(test_joint_history
        ${${PROJECT_NAME}_LIBRARIES} raisim::raisim)
target_compile_options(test_joint_history PRIVATE -mtune=native -fPIC -O3 -g -mno-avx2)

add_executable(test_actuator_model
        test/test_actuator_model.cpp)
target_link_libraries(test_actuator_model
        ${${PROJECT_NAME}_LIBRARIES} raisim::raisim)
target_compile_options(test_actuator_model PRIVATE -mtune=native -fPIC -O3 -g -mno-avx2)

add_executable(test_scaling
        test/test_scaling.cpp)
target_link_libraries(test_scaling
        ${${PROJECT_NAME}_LIBRARIES} raisim::raisim yaml-cpp)
target_compile_options(test_scaling PRIVATE -mtune=native -fPIC -O3 -g -mno-avx2)

add_executable(test_foot_motion
        test/test_foot_motion.cpp)
target_link_libraries(test_foot_motion
        ${${PROJECT_NAME}_LIBRARIES} raisim::raisim yaml-cpp)
target_compile_options(test_foot_motion PRIVATE -mtune=native -fPIC -O3 -g -mno-avx2)

#add_executable(test_c010
#        applications/test_c010.cpp)
#target_link_libraries(test_c010
#        ${${PROJECT_NAME}_LIBRARIES} raisim::raisim raisim::raisimOgre)

# EOF
