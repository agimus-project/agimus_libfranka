cmake_minimum_required(VERSION 3.22)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(BASE_NAME libfranka)
set(${BASE_NAME}_VERSION 0.9.3)
project(
  agimus_${BASE_NAME}
  VERSION ${${BASE_NAME}_VERSION}
  LANGUAGES CXX)

option(BUILD_TESTING "Build tests" ON)

# Dependencies
find_package(Poco REQUIRED COMPONENTS Net Foundation)
find_package(Eigen3 REQUIRED CONFIG)
find_package(agimus-libfranka-common REQUIRED CONFIG)
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

option(BUILD_COVERAGE "Build with code coverage" OFF)
if(BUILD_COVERAGE)
  add_compile_options(--coverage)
  # Unfortunately, there is no add_link_options...
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
endif()

# Library
add_library(
  ${PROJECT_NAME} SHARED
  src/active_control.cpp
  src/active_motion_generator.cpp
  src/active_torque_control.cpp
  src/control_loop.cpp
  src/control_tools.cpp
  src/control_types.cpp
  src/duration.cpp
  src/errors.cpp
  src/exception.cpp
  src/gripper.cpp
  src/gripper_state.cpp
  src/library_downloader.cpp
  src/library_loader.cpp
  src/load_calculations.cpp
  src/log.cpp
  src/logger.cpp
  src/lowpass_filter.cpp
  src/model.cpp
  src/model_library.cpp
  src/network.cpp
  src/rate_limiting.cpp
  src/robot.cpp
  src/robot_impl.cpp
  src/robot_state.cpp
  src/vacuum_gripper.cpp
  src/vacuum_gripper_state.cpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

set_target_properties(
  ${PROJECT_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON
                             VERSION ${${BASE_NAME}_VERSION})
target_compile_features(
  ${PROJECT_NAME}
  INTERFACE cxx_attribute_deprecated
            cxx_constexpr
            cxx_defaulted_functions
            cxx_deleted_functions
            cxx_generalized_initializers
            cxx_noexcept
            cxx_uniform_initialization)
if(MSVC)
  target_compile_definitions(
    ${PROJECT_NAME} PUBLIC _USE_MATH_DEFINES # for M_PI in cmath
                           NOMINMAX # avoid conflict with std::min
  )
endif()

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} PRIVATE Poco::Foundation Poco::Net
                                              Eigen3::Eigen Threads::Threads)

target_link_libraries(${PROJECT_NAME}
                      PUBLIC agimus-libfranka-common::agimus-libfranka-common)

# Installation
include(GNUInstallDirs)
set(INSTALL_CMAKE_CONFIG_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(
  DIRECTORY include/agimus_franka
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  USE_SOURCE_PERMISSIONS)

export(
  EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)
configure_package_config_file(
  cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_CONFIG_DIR})

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
        DESTINATION ${INSTALL_CMAKE_CONFIG_DIR})
install(
  EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${INSTALL_CMAKE_CONFIG_DIR})

# Subprojects

# Ignore find_package(agimus_libfranka) in subprojects.
set(FRANKA_IS_FOUND TRUE)

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

option(BUILD_EXAMPLES "Build example code" ON)
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

option(BUILD_DOCUMENTATION "Build documentation" OFF)
if(BUILD_DOCUMENTATION)
  add_subdirectory(doc)
endif()

# Tools
file(GLOB_RECURSE SOURCES src/*.cpp examples/*.cpp)
file(GLOB_RECURSE HEADERS include/*.h src/*.h examples/*.h)
file(GLOB_RECURSE TEST_FILES test/*.h test/*.cpp)
file(GLOB_RECURSE TIDY_IGNORED_FILES src/libfcimodels.h)
