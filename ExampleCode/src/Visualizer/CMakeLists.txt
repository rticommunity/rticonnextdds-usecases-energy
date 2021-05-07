# Thanks to @danger89 and @Ilothar for updating the gist.
# Set the name and the supported language of the project
project(Visualizer C CXX)

# Set the minimum version of cmake required to build this project
cmake_minimum_required(VERSION 3.10)

# Use the package PkgConfig to detect GTK+ headers/library files
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)

link_directories(
  ${GTKMM_LIBRARY_DIRS}  )

include_directories(
  ${GTKMM_INCLUDE_DIRS}  )

set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/../../resources/cmake"
  )

# Find the ConnextDDS libraries. This will look fo the core and API libraries
# only
find_package(RTIConnextDDS
  "6.0.1"
  REQUIRED
  COMPONENTS
  core
  )

# Run Codegen to generate the source code files for the types
include(ConnextDdsCodegen)
connextdds_rtiddsgen_run(
  IDL_FILE
  "${CMAKE_CURRENT_SOURCE_DIR}/../EnergyComms.idl"
  OUTPUT_DIRECTORY
  "${CMAKE_CURRENT_SOURCE_DIR}"
  LANG C++11
  )

add_custom_target(codegen_sources
  DEPENDS
  EnergyComms.cxx
  EnergyCommsPlugin.cxx
  )

# Create the target for the executable
add_executable(Visualizer
  "${CMAKE_CURRENT_SOURCE_DIR}/main.cxx"
  EnergyComms.cxx
  EnergyCommsPlugin.cxx
  listeners.hpp
  listeners.cxx
  window.hpp
  window.cxx
  )

target_link_libraries(Visualizer
  ${GTKMM_LIBRARIES}
  RTIConnextDDS::cpp2_api
  )

# Copy the USER_QOS_PROFILES.xml
add_custom_target(copy_qos
  DEPENDS
  "${CMAKE_CURRENT_BINARY_DIR}/USER_QOS_PROFILES.xml"
  )

add_custom_command(
  OUTPUT
  "${CMAKE_CURRENT_BINARY_DIR}/USER_QOS_PROFILES.xml"
  COMMAND
  ${CMAKE_COMMAND} -E copy_if_different
  "${CMAKE_CURRENT_SOURCE_DIR}/../USER_QOS_PROFILES.xml"
  "${CMAKE_CURRENT_BINARY_DIR}"
  COMMENT "Copying USER_QOS_PROFILES.xml"
  DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/../USER_QOS_PROFILES.xml"
  VERBATIM
  )

# Copy the Visualizer.glade file
add_custom_target(copy_glade
  DEPENDS
  "${CMAKE_CURRENT_BINARY_DIR}/Visualizer.glade"
  )

add_custom_command(
  OUTPUT
  "${CMAKE_CURRENT_BINARY_DIR}/Visualizer.glade"
  COMMAND
  ${CMAKE_COMMAND} -E copy_if_different
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualizer.glade"
  "${CMAKE_CURRENT_BINARY_DIR}"
  COMMENT "Copying Visualizer.glade"
  DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualizer.glade"
  VERBATIM
  )

add_custom_target(copy-img-files ALL
  COMMAND
  ${CMAKE_COMMAND} -E copy_directory
  "${CMAKE_CURRENT_SOURCE_DIR}/img"
  "${CMAKE_CURRENT_BINARY_DIR}/img"
  COMMENT "Copying Image Directory"
  DEPENDS
  "${MY_TARGET}"
  )

add_dependencies(Visualizer
  copy_qos
  copy_glade
  copy-img-files
  codegen_sources
  )

# Add other flags to the compiler
#target_compile_definitions(hello PRIVATE ${GTKMM_CFLAGS_OTHER})


# Setup CMake to use GTK+, tell the compiler where to look for headers
# and to the linker where to look for libraries
#target_include_directories(hello PRIVATE ${GTKMM_INCLUDE_DIRS})
#target_link_directories(hello PRIVATE ${GTKMM_LIBRARY_DIRS})
