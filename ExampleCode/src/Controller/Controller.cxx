/*
 * (c) 2021 Copyright, Real-Time Innovations, Inc. (RTI) All rights reserved.
 *
 * RTI grants Licensee a license to use, modify, compile, and create derivative
 * works of the software solely for use with RTI Connext DDS.  Licensee may
 * redistribute copies of the software provided that all such copies are
 * subject to this license. The software is provided "as is", with no warranty
 * of any type, including any warranty for fitness for any purpose. RTI is
 * under no obligation to maintain or support the software.  RTI shall not be
 * liable for any incidental or consequential damages arising out of the use or
 * inability to use the software.
 */

/* Controller.cxx

A simulated Microgrid Controller and Optimizer

(1) Compile this file and the other devices and applications

(2) Start the devices and applications

(3) Start the visualization

This file can be used as a starting point for any application interfacing a
PV system to a microgrid.
*/

#include <iostream>
#include <chrono>
#include <thread>
#include <random>
#include <condition_variable>
#include <fstream>

#include "Controller.hpp"

const std::chrono::duration<float> MinIslandDelay = std::chrono::seconds(5);
const std::chrono::duration<float> MaxTimeToWait = std::chrono::seconds(300);

using namespace Energy::Ops;
using namespace Energy::Common;
using namespace Energy::Enums;

using namespace dds::core;
using namespace dds::topic;
using namespace dds::pub;
using namespace dds::sub;

using namespace std;
using namespace std::chrono;
using std::ref;

/* Controller
 * This is the contstructor for the controller. This sets up communication and gets the class ready
 * to execute threads.
 */
Controller::Controller(const int domainId, const std::string& entityName, const INIReader& config) :
    ConnextEnergy(domainId, entityName)
{
    runProcesses_ = true;

    // Pull configuration from ini file
    optimizerID_ = config.Get("Controller", "OptimizerID", "SampleOpt");
    interconnectID_ = config.Get("Controller", "InterconnectID", "SampleInterconnect");
    vizID_ = config.Get("Controller", "VizID", "Visualizer");
    int32_t strength = config.GetInteger("Controller", "Strength", 1000);

    // Initialize Control DataWriters
    WriterControl_Device();
    WriterControl_Power();
    WriterVF_Device_Active();

    // Set up Control qos (including strength)
    SetDataWriterOwnershipStrength(Energy::application::TOPIC_CONTROL_DEVICE, strength);
    SetDataWriterOwnershipStrength(Energy::application::TOPIC_CONTROL_POWER, strength);
    SetDataWriterOwnershipStrength(Energy::application::TOPIC_VF_DEVICE_ACTIVE, strength);
}

/* StopController
 * The global boolean is used to stop all continuous threads and allow the application to shut down
 * gracefully
 */
void Controller::StopController()
{
    runProcesses_ = false;
}

/* ExecuteController
 * This is meant to be executed as it's own thread by main. This sets up all threads and executes
 * the waitset
 */
void Controller::ExecuteController()
{
    /* Create Query Conditions */
    using namespace dds::sub::status;
    using QueryCondition = dds::sub::cond::QueryCondition;
    using Condition = dds::core::cond::Condition;
    // Create query parameters
    std::vector<std::string> query_parameters = { "'" + optimizerID_ + "'" };
    DataState commonDataState = DataState(
        SampleState::not_read(),
        ViewState::any(),
        InstanceState::alive());

    // Query Condition for Controlling the Microgrid. This is basic
    // functionality for a grid connected device.
    QueryCondition QueryConditionControl_Microgrid(
        dds::sub::Query::Query(this->ReaderControl_Microgrid(), "Device MATCH %0", query_parameters),
        commonDataState, [this](Condition condition) {
            auto condition_as_qc = dds::core::polymorphic_cast<QueryCondition>(condition);
            auto samples = this->ReaderControl_Microgrid().select()
                .condition(condition_as_qc)
                .take();
            for (auto sample : samples)
                if (sample.info().valid())
                    std::thread(
                        &Controller::ProcessVizCommand, this, sample.data().MicrogridStatus())
                    .detach();
        });

    cout << "Created QueryConditionControl_Microgrid." << endl;

    // Query Condition for watching if the Interconnect trips
    query_parameters = { "'" + interconnectID_ + "'" };
    QueryCondition QueryConditionInterconnectStatus(
        dds::sub::Query::Query(this->ReaderStatus_Device(), "Device MATCH %0", query_parameters),
        commonDataState, [this](Condition condition) {
            auto condition_as_qc = dds::core::polymorphic_cast<QueryCondition>(condition);
            auto samples = this->ReaderStatus_Device().select().condition(condition_as_qc).read();
            for (auto sample : samples)
                if (sample.info().valid() && ConnectionStatus::DISCONNECTED == sample.data().ConnectionStatus())
                    std::thread(&Controller::CheckTrip, this).detach();
        });

    cout << "Created QueryConditionInterconnectStatus." << endl;

    // Launch thread for continuous optimization
    std::thread optimizeThread(&Controller::Optimize, this);

    cout << "Launch thread for continuous optimization." << endl;

    // Turn on all devices
    Energy::Ops::Control_Device sampleControlDevice(interconnectID_, optimizerID_, DeviceControl::CONNECT);
    this->WriterControl_Device().write(sampleControlDevice);
    sampleControlDevice.Device("SampleES");
    this->WriterControl_Device().write(sampleControlDevice);
    sampleControlDevice.Device("SamplePV");
    this->WriterControl_Device().write(sampleControlDevice);
    sampleControlDevice.Device("SampleLoad");
    this->WriterControl_Device().write(sampleControlDevice);

    cout << "Sent commands to Turn on all devices." << endl;

    // Write Initial Microgrid status
    Energy::Ops::Status_Microgrid sampleStatusMicrogrid(optimizerID_, MicrogridStatus::CONNECTED);
    this->WriterStatus_Microgrid().write(sampleStatusMicrogrid);

    // Set up the Waitset
    dds::core::cond::WaitSet waitset;
    waitset += QueryConditionControl_Microgrid;
    waitset += QueryConditionInterconnectStatus;

    // Here we are handling our waitset and reactions to inputs
    while (runProcesses_) {
        // Dispatch will call the handlers associated to the WaitSet conditions
        // when they activate
        waitset.dispatch(dds::core::Duration(4));  // Wait up to 4s each time
    }

    optimizeThread.join();
}

/* Protected Methods */

/* IslandOperation
 * This is the root function called by IslandMicrogrid and UnintentionalIsland
 */
void Controller::IslandOperation(bool Immediate)
{
    // Create the Samples for Device Control and VF Active device
    auto sampleControlDevice = Control_Device(
            interconnectID_,
            optimizerID_,
            DeviceControl::DISCONNECT);
    auto sampleVF_Device_Active =
            VF_Device_Active(optimizerID_, "", Timestamp());

    // Get the current strongest VF Device
    std::string VFDeviceID = "";
    for (auto sample : this->ReaderVF_Device().read())
        if (sample.info().valid())
            VFDeviceID = sample.data().Device();
    sampleVF_Device_Active.Device(VFDeviceID);  // Set Device for VF operation

    duration<float> IslandDelay;
    if (Immediate) {
        IslandDelay = seconds(0);
    } else {
        // Get the additional delay needed if the device is a generator else use
        // the MinIslandDelay
        IslandDelay = MinIslandDelay;
        for (auto sample : this->ReaderInfo_Generator().read())
            if (sample.info().valid() && sample.data().Device() == VFDeviceID)
                IslandDelay =
                        seconds(sample.data().RampUpTime()) + MinIslandDelay;
    }

    // Set the time to perform the island operation
    auto targetTime = high_resolution_clock::now() + IslandDelay;
    // Create the timestamp used in DDS communication
    auto ts = Energy::Common::Timestamp(duration_cast<seconds>(targetTime.time_since_epoch()).count(), 0);
    sampleVF_Device_Active.SwitchTime(ts);  // Set time for VF switchover
    // Send the command for active VF Device
    this->WriterVF_Device_Active().write(sampleVF_Device_Active);

    if (!Immediate) {
        // Wait until it's time and then tell the interconnect it's time to
        // open. The actual value may differ from the time for the VF Device
        // switch. This all depends on the delay between a device getting a
        // command to perform a mode switch and it actuially occuring and the
        // delay between the relay being told to open and it actually opening.
        // The DDS communication is incredibly low latency and can be ignored
        // except for unusual circumstances.
        std::this_thread::sleep_until(targetTime);
        // Send the command to switch
        this->WriterControl_Device().write(sampleControlDevice);
    }

    // At this point the microgrid should be islanded and this thread is done.
    return;
}

/* IslandMicrogrid
 * This function goes through the process of islanding the microgrid. The same
 * function will be used within this example for both intentional and
 * unintentional islanding operations. The only difference is whether or not the
 * transition happens immediately or after a short wait.
 */
void Controller::IslandMicrogrid()
{
    auto sampleStatus_Microgrid =
            Status_Microgrid(optimizerID_, MicrogridStatus::REQUEST_ISLAND);
    this->WriterStatus_Microgrid().write(sampleStatus_Microgrid);

    auto ThreadIsland = std::thread(&Controller::IslandOperation, this, false);

    // Here is where code would be called to make the microgrid ready for
    // off-grid operation if the load and generation are not adequately sized.

    // Rejoin the thread, update the status, and return
    ThreadIsland.join();
    sampleStatus_Microgrid.MicrogridStatus(MicrogridStatus::ISLANDED);
    this->WriterStatus_Microgrid().write(sampleStatus_Microgrid);
    return;
}

/* UnintentionalIsland
 * This function is called when it has been detected that the Main Interconnect
 * has tripped off. Some systems can detect events such as this and
 * automatically switch over to voltage and frequency support. As a belt and
 * suspenders, or if the device on the network that should provide this support
 * to the microgrid is incapable of this operation, this function should be
 * utilized.
 */
void Controller::UnintentionalIsland()
{
    auto sampleStatus_Microgrid =
            Status_Microgrid(optimizerID_, MicrogridStatus::ISLANDED);
    this->WriterStatus_Microgrid().write(sampleStatus_Microgrid);

    auto ThreadIsland = std::thread(&Controller::IslandOperation, this, true);

    // Here is where code would be called to perform emergency load shedding or
    // other operations to maintain stability

    // Rejoin the thread and return
    ThreadIsland.join();
    return;
}

/* Resynchronize
 * This function is called to resynchronize the microgrid to the main grid. The
 * main interconnect usually won't close in until there is a phase match and the
 * waveforms are crossing 0, so this function needs to tell the main
 * internconnect to close and then wait till it does to tell the Active VF
 * device to change mode away from VF.
 */
void Controller::Resynchronize()
{
    bool waiting = true;  // variable that allows the while loop to exit

    auto sampleStatus_Microgrid =
            Status_Microgrid(optimizerID_, MicrogridStatus::REQUEST_RESYNC);
    this->WriterStatus_Microgrid().write(sampleStatus_Microgrid);

    // Create the Samples for Device Control and VF Active device
    auto sampleControlDevice =
            Control_Device(interconnectID_, optimizerID_, DeviceControl::CONNECT);
    auto sampleVF_Device_Active =
            VF_Device_Active(optimizerID_, "", Energy::Common::Timestamp(0, 0));

    /* Create Query Condition */
    using namespace dds::core;
    using namespace dds::sub;
    using DataState = dds::sub::status::DataState;
    using SampleState = dds::sub::status::SampleState;
    using ViewState = dds::sub::status::ViewState;
    using InstanceState = dds::sub::status::InstanceState;
    using QueryCondition = dds::sub::cond::QueryCondition;
    using Condition = dds::core::cond::Condition;
    // Create query parameters
    std::vector<std::string> query_parameters = { "'" + interconnectID_ + "'" };
    DataState commonDataState = DataState(
            SampleState::any(),
            ViewState::any(),
            InstanceState::alive());
    // Query Condition for Controlling the device. This is basic functionality
    // for a grid connected device.
    QueryCondition QueryConditionStatus_Device(
            Query(this->ReaderStatus_Device(), "Device MATCH %0", query_parameters),
            commonDataState,
            [ref(this->ReaderStatus_Device()), &waiting, this](Condition condition) {
                auto condition_as_qc =
                        polymorphic_cast<QueryCondition>(condition);
                auto samples = this->ReaderStatus_Device().select()
                                       .condition(condition_as_qc)
                                       .read();
                for (auto sample : samples)
                    if (sample.info().valid()
                        && sample.data().ConnectionStatus()
                                == ConnectionStatus::CONNECTED)
                        waiting = false;
            });

    // Set up the Waitset
    dds::core::cond::WaitSet waitset;
    waitset += QueryConditionStatus_Device;

    // Update the Control_Device topic to make the interconnect start the resync
    // process
    this->WriterControl_Device().write(sampleControlDevice);

    // Here we are waiting forever because this is a simulated environment. Code
    // changes for timeout and probably canceling would be added for a real
    // system.
    while (waiting)
        waitset.dispatch(dds::core::Duration(4));  // Wait up to 4s each time

    // Update VF_Device_Active with the empty DeviceID
    this->WriterVF_Device_Active().write(sampleVF_Device_Active);

    // At this point the microgrid is now part of the main grid.
    sampleStatus_Microgrid.MicrogridStatus(MicrogridStatus::CONNECTED);
    this->WriterStatus_Microgrid().write(sampleStatus_Microgrid);
    return;
}

/* ProcessVizCommand
 * When there is an external command for microgrid operations there is a need to
 * process the command to make sure that nothing weird happens, such as trying
 * to island when you are already islanded.
 */
void Controller::ProcessVizCommand(MicrogridStatus command)
{
    auto currentStatus = MicrogridStatus::CONNECTED;  // This is the reasonable default

    const std::vector<std::string> query_parameters = { "'" + optimizerID_ + "'" };
    dds::sub::cond::QueryCondition query_condition(
            dds::sub::Query(
                    this->ReaderStatus_Microgrid(),
                    "Device MATCH %0",
                    query_parameters),
            dds::sub::status::DataState::any_data());

    for (auto sample :
        this->ReaderStatus_Microgrid().select().condition(query_condition).read()) {
        if (sample.info().valid())
            currentStatus = sample.data().MicrogridStatus();
    }

    switch (currentStatus) {
    case MicrogridStatus::CONNECTED:
        if (MicrogridStatus::REQUEST_ISLAND == command)
            std::thread(&Controller::IslandMicrogrid, this).detach();
        break;
    case MicrogridStatus::ISLANDED:
        if (MicrogridStatus::REQUEST_RESYNC == command)
            std::thread(&Controller::Resynchronize, this).detach();
        break;
    default:
        break;
    }
    return;
}

/* CheckTrip
 * This function takes the change in status of the interconnect and compares it
 * to the microgrid state to determine if a trip event has occurred.
 */
void Controller::CheckTrip()
{
    auto currentStatus = MicrogridStatus::CONNECTED;  // This is the reasonable default

    const std::vector<std::string> query_parameters = { "'" + optimizerID_ + "'" };
    dds::sub::cond::QueryCondition query_condition(
            dds::sub::Query(
                this->ReaderStatus_Microgrid(),
                "Device MATCH %0",
                query_parameters),
            dds::sub::status::DataState::any_data());

    for (auto sample :
        this->ReaderStatus_Microgrid().select().condition(query_condition).read()) {
        if (sample.info().valid())
            currentStatus = sample.data().MicrogridStatus();
    }

    switch (currentStatus) {
    case MicrogridStatus::CONNECTED:
        std::thread(&Controller::UnintentionalIsland, this).detach();
        break;
    default:
        break;
    }
    return;
}

/* Optimize
 * This is the function that pulls in all of the information from the devices on
 * the microgrid and fiddles with the available knobs. Optimization is too big a
 * problem for this example so we are going to only do simple operation
 * parameters based on whether we're on or off grid. A full optimization is
 * going to pull in a lot more information than we're using here, including
 * device limits, voltage, current, and power measurement, forecasts, and price
 * signals. We're just going to charge the battery when we're on-grid and try to
 * use the battery as an intelligent load when we're off grid to run the
 * generator at peak efficiency. We'll also be checking for a change in the VF
 * Device ID due to changes in battery SOC.
 */
void Controller::Optimize()
{
    const std::vector<std::string> query_parameters = { "'" + optimizerID_ + "'" };
    dds::sub::cond::QueryCondition query_condition(
            dds::sub::Query(
                this->ReaderStatus_Microgrid(),
                "Device MATCH %0",
                query_parameters),
            dds::sub::status::DataState::any_data());

    std::string VFDevice = "";
    bool VFDeviceChanged = false;

    auto currentStatus = MicrogridStatus::CONNECTED;  // This is the reasonable default
    auto sampleControl_Power = CNTL_Single_float32("", optimizerID_, 0.0);
    auto sampleVF_Device_Active = VF_Device_Active(optimizerID_, "", Timestamp());

    // Optimization is a continuous process that goes on as long as the
    // Controller is running.
    while (runProcesses_) {
        for (auto sample : this->ReaderStatus_Microgrid().select().condition(query_condition).read()) {
            if (sample.info().valid() && sample.data().Device() == optimizerID_)
                currentStatus = sample.data().MicrogridStatus();
        }
        std::cout << "Current Status: " << currentStatus << std::endl;
        switch (currentStatus) {
        case MicrogridStatus::REQUEST_ISLAND:
            // It's likely there are going to be specific operations that need
            // to be performed during an island request. Just not in this
            // example.
        case MicrogridStatus::CONNECTED:
            // Set all batteries to charge
            for (auto sample : this->ReaderMeas_SOC().take()) {
                if (sample.info().valid()) {
                    // Charge the battery to 95%
                    if (sample.data().Value() < 95.0) {
                        // Find the max charge we can do and do it.
                        for (auto info : this->ReaderInfo_Battery().read()) {
                            if (info.info().valid() && info.data().Device() == sample.data().Device()) {
                                sampleControl_Power.Device(info.data().Device());
                                sampleControl_Power.SetPoint(info.data().MaxLoad());
                                this->WriterControl_Power().write(sampleControl_Power);
                            }
                        }
                    }
                    // We don't want to overcharge it
                    else {
                        sampleControl_Power.Device(sample.data().Device());
                        sampleControl_Power.SetPoint(0.0);
                        this->WriterControl_Power().write(sampleControl_Power);
                    }
                }
            }
            break;
        case MicrogridStatus::REQUEST_RESYNC:
            // This is really still an island case.
        case MicrogridStatus::ISLANDED:
            // Get the VF Device ID once.
            VFDeviceChanged = false;
            for (auto vfdev : this->ReaderVF_Device().read())
                if (vfdev.info().valid()) {
                    // If there is a VF Device change, update the Active VF
                    // Device
                    if (vfdev.data().Device() != VFDevice) {
                        VFDeviceChanged = true;
                        VFDevice = vfdev.data().Device();
                        sampleVF_Device_Active.Device(VFDevice);
                        sampleVF_Device_Active.SwitchTime(
                            Energy::Common::Timestamp(
                                duration_cast<seconds>(high_resolution_clock::now().time_since_epoch()).count() + 
                                duration_cast<seconds>(MinIslandDelay).count(), 0)
                        );
                    }
                }
            // Find out if the VF Device is a Generator
            for (auto gen : this->ReaderInfo_Generator().read())
                if (gen.info().valid() && gen.data().Device() == VFDevice) {
                    if (VFDeviceChanged) {
                        auto ts = sampleVF_Device_Active.SwitchTime().Seconds() + gen.data().RampUpTime();
                        sampleVF_Device_Active.SwitchTime(Timestamp(ts, 0));
                    }
                    // Charge or discharge the battery to maintain generator
                    // efficiency. In a real system load shedding and/or
                    // scheduling could also be used, as well as solar
                    // curtailment (although any system that's artificially
                    // curtailing solar is wasting an investment)
                    float peakSetPoint = 0.0;
                    float peakEfficiency = 0.0;
                    // Go through all the values and find the best
                    for (Energy::Common::EfficiencyPoint val : gen.data().PowerEfficiency())
                        if (val.Efficiency() > peakEfficiency)
                            peakSetPoint = val.Value();
                    // Find out how much we need to modify battery setpoints to
                    // get the generator into peak efficiency
                    float genPowerDelta = 0.0;
                    for (auto power : this->ReaderMeas_NodePower().read())
                        if (power.info().valid() && power.data().Device() == VFDevice)
                            genPowerDelta = power.data().Value() - peakSetPoint;
                    // Go through the batteries and start changing setpoints
                    for (auto batt : this->ReaderInfo_Battery().read())
                        if (genPowerDelta == 0.0)
                            break;
                        else if (batt.info().valid())
                            for (auto power : this->ReaderMeas_NodePower().read())
                                if (power.info().valid() && batt.data().Device() == power.data().Device()) {
                                    float setPoint = power.data().Value() + genPowerDelta;
                                    // Don't go past max load
                                    if (setPoint < batt.data().MaxLoad()) {
                                        genPowerDelta = setPoint - batt.data().MaxLoad();
                                        setPoint = batt.data().MaxLoad();
                                    }
                                    // Don't go past Max Generation
                                    else if (setPoint > batt.data().MaxGeneration()) {
                                        genPowerDelta = setPoint - batt.data().MaxGeneration();
                                        setPoint = batt.data().MaxGeneration();
                                    }
                                    // Didn't go past. We're done.
                                    else
                                        genPowerDelta = 0.0;
                                    sampleControl_Power.Device(batt.data().Device());
                                    sampleControl_Power.SetPoint(setPoint);
                                    this->WriterControl_Power().write(sampleControl_Power);
                                }
                }
            if (VFDeviceChanged)
                this->WriterVF_Device_Active().write(sampleVF_Device_Active);
            break;
        default:
            break;
        }
        std::this_thread::sleep_for(seconds(2));
    }
}
