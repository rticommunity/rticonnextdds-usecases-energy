/*
 * (c) Copyright, Real-Time Innovations, 2012.  All rights reserved.
 * RTI grants Licensee a license to use, modify, compile, and create derivative
 * works of the software solely for use with RTI Connext DDS. Licensee may
 * redistribute copies of the software provided that all such copies are subject
 * to this license. The software is provided "as is", with no warranty of any
 * type, including any warranty for fitness for any purpose. RTI is under no
 * obligation to maintain or support the software. RTI shall not be liable for
 * any incidental or consequential damages arising out of the use or inability
 * to use the software.
 */

/* PowerFlowSim.cxx

Super simple power flow simulator.

(1) Compile this file and the other devices and applications

(2) Start the devices and applications

(3) Start the visualization

This file can be used as a starting point for any application interfacing a
load (smart or dumb) to a microgrid.
*/

#include <iostream>
#include <chrono>
#include <thread>
#include <random>

#include <dds/dds.hpp>

#include "PowerFlowSim.hpp"
#include "../generated/EnergyComms.hpp"

using namespace dds::core;
using namespace dds::topic;
using namespace dds::pub;
using namespace dds::sub;

using namespace Energy::Ops;
using namespace Energy::Common;
using namespace Energy::Enums;

/* Controller
 * This is the contstructor for the controller. This sets up communication and gets the class ready
 * to execute threads.
 */
PowerFlowSim::PowerFlowSim(const int domainId, const std::string& entityName, const INIReader& config) :
    ConnextEnergy(domainId, entityName)
{
    runProcesses_ = true;
    currentStatus_ = Energy::Enums::MicrogridStatus::CONNECTED;

    // Pull configuration from ini file
    optimizerID_ = config.Get("PowerFlowSim", "OptimizerID", "SampleOpt");
    interconnectID_ = config.Get("PowerFlowSim", "InterconnectID", "SampleInterconnect");
    deviceID_ = config.Get("PowerFlowSim", "DeviceID", "PowerFlowSim");
    simThreadWait_ = std::chrono::milliseconds(config.GetInteger("PowerFlowSim", "SimThreadWait", 100));
    int32_t strength = config.GetInteger("PowerFlowSim", "Strength", 20000);

    // Initialize Control DataWriters and DataReaders
    WriterControl_Power();
    ReaderMeas_NodePower();
    ReaderVF_Device_Active();
    ReaderStatus_Microgrid();

    // Set up Control qos (including strength)
    using namespace Energy::application;
    SetDataWriterOwnershipStrength(TOPIC_CONTROL_POWER, strength);
}

/* StopSim
 * The global boolean is used to stop all continuous threads and allow the application to shut down
 * gracefully
 */
void PowerFlowSim::StopSim()
{
    runProcesses_.store(false);
}

/* ExecuteSim
 * This is meant to be executed as it's own thread by main. This sets up all threads and executes
 * the waitset
 */
void PowerFlowSim::ExecuteSim()
{
    using std::string;
    /* Create Query Conditions */
    using namespace dds::sub::status;
    using QueryCondition = dds::sub::cond::QueryCondition;
    using Condition = dds::core::cond::Condition;
    // Create query parameters
    std::vector<string> query_parameters = { "'" + OptimizerID() + "'" };
    DataState commonDataState = DataState(
        SampleState::not_read(),
        ViewState::any(),
        InstanceState::alive());

    // Query Condition for Controlling the Microgrid. This is basic
    // functionality for a grid connected device.
    QueryCondition QueryConditionMicrogridStatus(
        dds::sub::Query(this->ReaderStatus_Microgrid(), "Device MATCH %0", query_parameters),
        commonDataState, [this](Condition condition) {
            auto currentStatus = Energy::Enums::MicrogridStatus::CONNECTED;
            auto condition_as_qc = dds::core::polymorphic_cast<QueryCondition>(condition);
            auto samples = this->ReaderStatus_Microgrid().select().condition(condition_as_qc).read();
            for (auto sample : samples)
                if (sample.info().valid())
                    CurrentStatus(sample.data().MicrogridStatus());
        });

    // Launch thread for continuous sim
    std::thread simThread(&PowerFlowSim::SimThread, this);

    // Set up the Waitset
    dds::core::cond::WaitSet waitset;
    waitset += QueryConditionMicrogridStatus;
    
    // Here we are handling our waitset and reactions to inputs
    while (RunProcesses()) {
        // Dispatch will call the handlers associated to the WaitSet conditions
        // when they activate
        waitset.dispatch(dds::core::Duration(1));  // Wait up to 4s each time
    }

    // Once the process is told to exit, rejoin the simThread (which should be done)
    simThread.join();
}

/* SimThread
 * This thread runs continuously to allow changes in the condition of the grid
 * to change the way the powerflow behaves. It is set as virtual to allow for
 * future expansion.
 */
void PowerFlowSim::SimThread()
{
    while (RunProcesses()) {
        switch (currentStatus_) {
        case Energy::Enums::MicrogridStatus::REQUEST_ISLAND:
        case Energy::Enums::MicrogridStatus::CONNECTED:
            this->BalanceConnected();
            break;
        case Energy::Enums::MicrogridStatus::REQUEST_RESYNC:
        case Energy::Enums::MicrogridStatus::ISLANDED:
            this->BalanceIsland();
            break;
        default:
            break;
        }

        std::this_thread::sleep_for(SimThreadWait());
    }
}

/* BalanceConnected
 * This sets the power levels that the Main Interconnect will report based on
 * all device measurements
 */
void PowerFlowSim::BalanceConnected()
{
    // Add up all of the power
    float powerSum = 0.0f;
    for (auto meas : ReaderMeas_NodePower().read())
        if (meas.info().valid() && meas.data().Device() != InterconnectID())
            powerSum += meas.data().Value();

    // Take the result and set the interconnect power
    auto sample = Energy::Common::CNTL_Single_float32(InterconnectID(), DeviceID(), powerSum);
    WriterControl_Power().write(sample);
}

/* BalanceIsland
 * This takes all of the measurements (except for the VF Device) and sets the
 * necessary power output of the VF Device
 */
void PowerFlowSim::BalanceIsland()
{
    // Get the VF Device ID
    std::string VFDeviceID = "";
    for (auto dev : ReaderVF_Device_Active().read())
        if (dev.info().valid())
            VFDeviceID = dev.data().Device();

    // Add up all the power
    float powerSum = 0.0f;
    for (auto meas : ReaderMeas_NodePower().read())
        if (meas.info().valid() && meas.data().Device() != VFDeviceID)
            powerSum -= meas.data().Value();

    // Take the result and set the VF Device power
    auto sample = Energy::Common::CNTL_Single_float32(VFDeviceID, DeviceID(), powerSum);
    WriterControl_Power().write(sample);
}

const std::string& PowerFlowSim::DeviceID() const
{
    return deviceID_;
}

const std::string& PowerFlowSim::OptimizerID() const
{
    return optimizerID_;
}

const std::string& PowerFlowSim::InterconnectID() const
{
    return interconnectID_;
}

const std::chrono::milliseconds& PowerFlowSim::SimThreadWait() const
{
    return simThreadWait_;
}

// Getter for reference to atomic instance of currentStatus_
const Energy::Enums::MicrogridStatus PowerFlowSim::CurrentStatus() const
{
    return currentStatus_.load();
}
// Sets value of atomic global
void PowerFlowSim::CurrentStatus(Energy::Enums::MicrogridStatus status)
{
    currentStatus_.store(status);
}

// Getter for process active status
const bool PowerFlowSim::RunProcesses() const 
{
    return runProcesses_;
}