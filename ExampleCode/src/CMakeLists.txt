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
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
set(PATH_COMMON ${CMAKE_CURRENT_SOURCE_DIR}/common)
set(PATH_VIZ ${CMAKE_CURRENT_SOURCE_DIR}/Visualizer)
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

# Set files from common directory
set(FILES_COMMON
  ${PATH_COMMON}/ConnextCommunicator.cxx
  ${PATH_COMMON}/ConnextEnergy.cxx
  ${PATH_COMMON}/IED.cxx)

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
  OBJECT ${FILES_CODEGEN}
  ${FILES_COMMON})

target_link_libraries(common
  RTIConnextDDS::cpp2_api)

add_dependencies(common
  codegen_sources)

# Create the target for the Visualizer executable
add_executable(Visualizer
  ${PATH_VIZ}/Main.cxx
  ${PATH_VIZ}/Visualizer.cxx
  ${PATH_VIZ}/Window.cxx
  $<TARGET_OBJECTS:common>)

# Copy the Visualizer.glade file
add_custom_command(TARGET Visualizer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PATH_VIZ}/Visualizer.glade
  ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Copying Visualizer.glade"
  DEPENDS ${PATH_VIZ}/Visualizer.glade
  VERBATIM)

# Copy the Visualizer.ini file
add_custom_command(TARGET Visualizer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PATH_VIZ}/Visualizer.ini
  ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Copying Visualizer.ini"
  DEPENDS ${PATH_VIZ}/Visualizer.ini
  VERBATIM)

# Copy the images directory
add_custom_command(TARGET Visualizer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${PATH_VIZ}/img
  ${CMAKE_CURRENT_BINARY_DIR}/img
  COMMENT "Copying Image Directory")

target_link_libraries(Visualizer
  ${GTKMM_LIBRARIES}
  RTIConnextDDS::cpp2_api)

add_executable(Load
  ${PATH_LOAD}/Load.cxx
  ${PATH_LOAD}/Main.cxx
  $<TARGET_OBJECTS:common>)

# Copy the Load.ini file
add_custom_command(TARGET Load POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PATH_LOAD}/Load.ini
  ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Copying Load.ini"
  DEPENDS ${PATH_LOAD}/Load.ini
  VERBATIM)

target_link_libraries(Load
  RTIConnextDDS::cpp2_api)

add_executable(MainInterconnect
  ${PATH_MAIN}/MainInterconnect.cxx
  ${PATH_MAIN}/Main.cxx
  $<TARGET_OBJECTS:common>)

# Copy the MainInterconnect.ini file
add_custom_command(TARGET MainInterconnect POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PATH_MAIN}/MainInterconnect.ini
  ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Copying MainInterconnect.ini"
  DEPENDS ${PATH_MAIN}/MainInterconnect.ini
  VERBATIM)

target_link_libraries(MainInterconnect
  RTIConnextDDS::cpp2_api)

add_executable(Generator
  ${PATH_GEN}/Generator.cxx
  ${PATH_GEN}/Main.cxx
  $<TARGET_OBJECTS:common>)

# Copy the Generator.ini file
add_custom_command(TARGET Generator POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PATH_GEN}/Generator.ini
  ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Copying Generator.ini"
  DEPENDS ${PATH_GEN}/Generator.ini
  VERBATIM)

target_link_libraries(Generator
  RTIConnextDDS::cpp2_api)

add_executable(EnergyStorage
  ${PATH_ES}/EnergyStorage.cxx
  ${PATH_ES}/Main.cxx
  $<TARGET_OBJECTS:common>)

# Copy the EnergyStorage.ini file
add_custom_command(TARGET EnergyStorage POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PATH_ES}/EnergyStorage.ini
  ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Copying EnergyStorage.ini"
  DEPENDS ${PATH_ES}/EnergyStorage.ini
  VERBATIM)

target_link_libraries(EnergyStorage
  RTIConnextDDS::cpp2_api)

add_executable(SolarPV
  ${PATH_PV}/SolarPV.cxx
  ${PATH_PV}/Main.cxx
  $<TARGET_OBJECTS:common>)

# Copy the SolarPV.ini file
add_custom_command(TARGET SolarPV POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PATH_PV}/SolarPV.ini
  ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Copying SolarPV.ini"
  DEPENDS ${PATH_PV}/SolarPV.ini
  VERBATIM)

target_link_libraries(SolarPV
  RTIConnextDDS::cpp2_api)

add_executable(PowerFlowSim
  ${PATH_SIM}/PowerFlowSim.cxx
  ${PATH_SIM}/Main.cxx
  $<TARGET_OBJECTS:common>)

# Copy the PowerFlowSim.ini file
add_custom_command(TARGET PowerFlowSim POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PATH_SIM}/PowerFlowSim.ini
  ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Copying PowerFlowSim.ini"
  DEPENDS ${PATH_SIM}/PowerFlowSim.ini
  VERBATIM)

target_link_libraries(PowerFlowSim
  RTIConnextDDS::cpp2_api)

add_executable(Controller
  ${PATH_CONTROLLER}/Controller.cxx
  ${PATH_CONTROLLER}/Main.cxx
  $<TARGET_OBJECTS:common>)

# Copy the Controller.ini file
add_custom_command(TARGET Controller POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PATH_CONTROLLER}/Controller.ini
  ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Copying Controller.ini"
  DEPENDS ${PATH_CONTROLLER}/Controller.ini
  VERBATIM)

target_link_libraries(Controller
  RTIConnextDDS::cpp2_api)
