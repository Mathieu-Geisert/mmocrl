cmake_minimum_required( VERSION 3.10 )
project(mmocrl)
set(CMAKE_CXX_STANDARD 17)

option(BUILD_TEST OFF)

find_package(raisim CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include_directories(include
      raisimGymTorch/env
      ${EIGEN3_INCLUDE_DIRS})

if (${BUILD_TEST})
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

add_executable(test_contacts
        test/test_contacts.cpp)
target_link_libraries(test_contacts
        ${${PROJECT_NAME}_LIBRARIES} raisim::raisim yaml-cpp)
target_compile_options(test_contacts PRIVATE -mtune=native -fPIC -O3 -g -mno-avx2)

add_executable(test_privileged_observation
        test/test_privileged_observation.cpp)
target_link_libraries(test_privileged_observation
        ${${PROJECT_NAME}_LIBRARIES} raisim::raisim yaml-cpp)
target_compile_options(test_privileged_observation PRIVATE -mtune=native -fPIC -O3 -g -mno-avx2)

add_executable(test_internal_observation
        test/test_internal_observation.cpp)
target_link_libraries(test_internal_observation
        ${${PROJECT_NAME}_LIBRARIES} raisim::raisim yaml-cpp)
target_compile_options(test_internal_observation PRIVATE -mtune=native -fPIC -O3 -g -mno-avx2)

add_executable(test_rewards
        test/test_rewards.cpp)
target_link_libraries(test_rewards
        ${${PROJECT_NAME}_LIBRARIES} raisim::raisim yaml-cpp)
target_compile_options(test_rewards PRIVATE -mtune=native -fPIC -O3 -g -mno-avx2)

add_executable(test_disturbance
        test/test_disturbance.cpp)
target_link_libraries(test_disturbance
        ${${PROJECT_NAME}_LIBRARIES} raisim::raisim yaml-cpp)
target_compile_options(test_disturbance PRIVATE -mtune=native -fPIC -O3 -g -mno-avx2)
endif()


#RaisimGymTorch
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)

######################
### help functions ###
######################
MACRO(SUBDIRLIST result curdir)
    FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
    SET(dirlist "")
    FOREACH(child ${children})
        IF(IS_DIRECTORY ${curdir}/${child})
            LIST(APPEND dirlist ${child})
        ENDIF()
    ENDFOREACH()
    SET(${result} ${dirlist})
ENDMACRO()

####################
### dependencies ###
####################
set(RAISIM_DIR "/home/pshah/Applications/raisim/raisim_ws/raisimLib")
add_subdirectory(${RAISIM_DIR}/thirdParty/pybind11 pybind11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

#######################
### src and linking ###
#######################
set(RAISIMGYM_ENV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/gyms)
SUBDIRLIST(SUBDIRS ${RAISIMGYM_ENV_DIR})

FOREACH(subdir ${SUBDIRS})
    pybind11_add_module(${subdir} raisimGymTorch/env/raisim_gym.cpp raisimGymTorch/env/Yaml.cpp)
    target_link_libraries(${subdir} PUBLIC yaml-cpp PRIVATE raisim::raisim)
    target_include_directories(${subdir} PUBLIC ${EIGEN3_INCLUDE_DIRS} ${RAISIMGYM_ENV_DIR}/${subdir})
    target_compile_options(${subdir} PRIVATE -mtune=native -fPIC -O3 -g -mno-avx2)
    target_compile_definitions(${subdir} PRIVATE "-DRAISIMGYM_TORCH_ENV_NAME=${subdir}")
    install(TARGETS ${subdir}
        RUNTIME DESTINATION ${RAISIMGYM_ENV_DIR}/${subdir}
        ARCHIVE DESTINATION ${RAISIMGYM_ENV_DIR}/${subdir}
        LIBRARY DESTINATION ${RAISIMGYM_ENV_DIR}/${subdir}
        )

    if (CMAKE_BUILD_TYPE STREQUAL "DEBUG" OR CMAKE_BUILD_TYPE STREQUAL "Debug")
        message("[RAISIM_GYM] BUILDING THE DEBUG APP for ${subdir}")
        add_executable(${subdir}_debug_app raisimGymTorch/env/debug_app.cpp raisimGymTorch/env/Yaml.cpp)
        target_link_libraries(${subdir}_debug_app PRIVATE raisim::raisim yaml-cpp)
        target_include_directories(${subdir}_debug_app PUBLIC ${RAISIMGYM_ENV_DIR}/${subdir} ${EIGEN3_INCLUDE_DIRS})
        if(WIN32)
            target_link_libraries(${subdir}_debug_app PRIVATE Ws2_32)
        else()
            target_compile_options(${subdir}_debug_app PRIVATE -mtune=native -fPIC -g -O0 -mno-avx2)
        endif()
        install(TARGETS ${subdir}_debug_app
            RUNTIME DESTINATION ${RAISIMGYM_ENV_DIR}/${subdir}
            ARCHIVE DESTINATION ${RAISIMGYM_ENV_DIR}/${subdir}
            LIBRARY DESTINATION ${RAISIMGYM_ENV_DIR}/${subdir}
            )
    endif()
ENDFOREACH()
# EOF
