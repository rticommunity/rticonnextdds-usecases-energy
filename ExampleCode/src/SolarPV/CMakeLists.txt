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
project(EnergyComms)
set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    "${CMAKE_CURRENT_SOURCE_DIR}/../../resources/cmake"
)

# Include ConnextDdsAddExample.cmake from resources/cmake
include(ConnextDdsAddExample)
connextdds_add_example (
    IDL "EnergyComms"
    LANG "C++11"
	QOS_FILENAME "${CMAKE_CURRENT_SOURCE_DIR}/../EnergyComms.xml"
	#NO_REQUIRE_QOS
    #CODEGEN_ARGS
    #LEGACY_PLUGIN
    APPLICATION_NAMES
        "PV"
)