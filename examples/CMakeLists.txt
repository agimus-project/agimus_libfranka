cmake_minimum_required(VERSION 3.22)

project(agimus-libfranka-examples CXX)

if(NOT FRANKA_IS_FOUND)
  find_package(agimus_libfranka REQUIRED CONFIG)
endif()
find_package(Eigen3 REQUIRED CONFIG)
find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC examples_common.cpp)

target_link_libraries(examples_common PRIVATE agimus_libfranka::agimus_libfranka
                                              Eigen3::Eigen)
target_include_directories(
  examples_common PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
set(EXAMPLES
    cartesian_impedance_control
    communication_test
    echo_robot_state
    force_control
    generate_cartesian_pose_motion
    generate_cartesian_pose_motion_external_control_loop
    generate_cartesian_velocity_motion
    generate_cartesian_velocity_motion_external_control_loop
    generate_consecutive_motions
    generate_elbow_motion
    generate_joint_position_motion
    generate_joint_position_motion_external_control_loop
    generate_joint_velocity_motion
    generate_joint_velocity_motion_external_control_loop
    grasp_object
    joint_impedance_control
    joint_point_to_point_motion
    motion_with_control
    motion_with_control_external_control_loop
    print_joint_poses
    vacuum_object)

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_include_directories(
    ${example} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include>)
  target_link_libraries(${example} agimus_libfranka::agimus_libfranka
                        examples_common Eigen3::Eigen)
endforeach()

target_link_libraries(joint_impedance_control Threads::Threads)
target_link_libraries(motion_with_control Poco::Foundation)
target_link_libraries(motion_with_control_external_control_loop
                      Poco::Foundation)

include(GNUInstallDirs)
install(
  TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
