# (c) 2021 Copyright, Real-Time Innovations, Inc. (RTI) All rights reserved.
#
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the software solely for use with RTI Connext DDS.  Licensee may
# redistribute copies of the software provided that all such copies are
# subject to this license. The software is provided "as is", with no warranty
# of any type, including any warranty for fitness for any purpose. RTI is
# under no obligation to maintain or support the software.  RTI shall not be
# liable for any incidental or consequential damages arising out of the use or
# inability to use the software.

cmake_minimum_required(VERSION 3.7)
project(EnergyComms C CXX)

# Copy the USER_QOS_PROFILES.xml to build directory
add_custom_target(copy_qos_to_build ALL)

add_custom_command(TARGET copy_qos_to_build PRE_BUILD
  COMMAND
  ${CMAKE_COMMAND} -E copy_if_different
  "${CMAKE_CURRENT_SOURCE_DIR}/USER_QOS_PROFILES.xml"
  "${CMAKE_CURRENT_BINARY_DIR}"
  COMMENT "Copying USER_QOS_PROFILES.xml"
  DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/USER_QOS_PROFILES.xml"
  VERBATIM
  )

# Use the package PkgConfig to detect GTK+ headers/library files
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)

link_directories(
  ${GTKMM_LIBRARY_DIRS})

include_directories(
  ${GTKMM_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../resources/cmake)

# Find the ConnextDDS libraries. This will look fo the core and API libraries
# only
find_package(RTIConnextDDS
  "6.0.1"
  REQUIRED
  COMPONENTS
  core
  )

# Set paths
set(PATH_CODEGEN ${CMAKE_CURRENT_SOURCE_DIR}/generated)
set(PATH_VIS ${CMAKE_CURRENT_SOURCE_DIR}/Visualizer)
set(PATH_LOAD ${CMAKE_CURRENT_SOURCE_DIR}/Load)
set(PATH_MAIN ${CMAKE_CURRENT_SOURCE_DIR}/MainInterconnect)
set(PATH_GEN ${CMAKE_CURRENT_SOURCE_DIR}/Generator)
set(PATH_ES ${CMAKE_CURRENT_SOURCE_DIR}/EnergyStorage)
set(PATH_PV ${CMAKE_CURRENT_SOURCE_DIR}/SolarPV)
set(PATH_SIM ${CMAKE_CURRENT_SOURCE_DIR}/PowerFlowSim)
set(PATH_CONTROLLER ${CMAKE_CURRENT_SOURCE_DIR}/Controller)

# Set files generated from code gen
set(FILES_CODEGEN
  ${PATH_CODEGEN}/EnergyComms.cxx
  ${PATH_CODEGEN}/EnergyCommsPlugin.cxx
  )

# Run Codegen to generate the source code files for the types
include(ConnextDdsCodegen)
connextdds_rtiddsgen_run(
  IDL_FILE ${CMAKE_CURRENT_SOURCE_DIR}/EnergyComms.idl
  OUTPUT_DIRECTORY ${PATH_CODEGEN}
  LANG C++11
  )

add_custom_target(codegen_sources
  DEPENDS ${FILES_CODEGEN})

# Create a common library used by all executables
add_library(common
  OBJECT ${FILES_CODEGEN})

target_link_libraries(common
  RTIConnextDDS::cpp2_api)

add_dependencies(common
  codegen_sources)

# Create the target for the Visualizer executable
add_executable(Visualizer
  ${PATH_VIS}/main.cxx
  ${PATH_VIS}/window.hpp
  ${PATH_VIS}/window.cxx
  $<TARGET_OBJECTS:common>)

# Copy the Visualizer.glade file
add_custom_command(TARGET Visualizer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PATH_VIS}/Visualizer.glade
  ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Copying Visualizer.glade"
  DEPENDS ${PATH_VIS}/Visualizer.glade
  VERBATIM)

# Copy the images directory
add_custom_command(TARGET Visualizer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${PATH_VIS}/img
  ${CMAKE_CURRENT_BINARY_DIR}/img
  COMMENT "Copying Image Directory")

target_link_libraries(Visualizer
  ${GTKMM_LIBRARIES}
  RTIConnextDDS::cpp2_api)

add_executable(Load
  ${PATH_LOAD}/Load.cxx
  $<TARGET_OBJECTS:common>)

target_link_libraries(Load
  RTIConnextDDS::cpp2_api)

add_executable(MainInterconnect
  ${PATH_MAIN}/MainInterconnect.cxx
  $<TARGET_OBJECTS:common>)

target_link_libraries(MainInterconnect
  RTIConnextDDS::cpp2_api)

add_executable(Generator
  ${PATH_GEN}/Generator.cxx
  $<TARGET_OBJECTS:common>)

target_link_libraries(Generator
  RTIConnextDDS::cpp2_api)

add_executable(EnergyStorage
  ${PATH_ES}/ES.cxx
  $<TARGET_OBJECTS:common>)

target_link_libraries(EnergyStorage
  RTIConnextDDS::cpp2_api)

add_executable(SolarPV
  ${PATH_PV}/PV.cxx
  $<TARGET_OBJECTS:common>)

target_link_libraries(SolarPV
  RTIConnextDDS::cpp2_api)

add_executable(PowerFlowSim
  ${PATH_SIM}/PowerFlowSim.cxx
  $<TARGET_OBJECTS:common>)

target_link_libraries(PowerFlowSim
  RTIConnextDDS::cpp2_api)

add_executable(Controller
  ${PATH_CONTROLLER}/Controller.cxx
  $<TARGET_OBJECTS:common>)

target_link_libraries(Controller
  RTIConnextDDS::cpp2_api)
