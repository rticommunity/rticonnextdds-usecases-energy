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

#include <dds/dds.hpp>

#include "EnergyComms.hpp"

const std::string OptimizerID = "SampleOpt";
const std::string InterconnectID = "SampleInterconnect";
const std::string VizID = "SampleViz";
const std::chrono::duration<float> MinIslandDelay = std::chrono::seconds(5);

Energy::Common::Timestamp SwitchTime;
const std::chrono::duration<float> MaxTimeToWait = std::chrono::seconds(300);

/* IslandOperation
* This is the root function called by IslandMicrogrid and UnintentionalIsland
*/
void IslandOperation( bool Immediate,
    dds::sub::DataReader<Energy::Ops::VF_Device> ReaderVF_Device,
    dds::sub::DataReader<Energy::Ops::Info_Generator> ReaderInfo_Generator,
    dds::pub::DataWriter<Energy::Ops::Control_Device> WriterControl_Device,
    dds::pub::DataWriter<Energy::Ops::VF_Device_Active> WriterVF_Device_Active)
{
    using namespace std::chrono;

    // Create the Samples for Device Control and VF Active device
    auto sampleControlDevice = Energy::Ops::Control_Device(InterconnectID, OptimizerID, Energy::Enums::DeviceControl::DISCONNECT);
    auto sampleVF_Device_Active = Energy::Ops::VF_Device_Active(OptimizerID, "", Energy::Common::Timestamp());

    // Get the current strongest VF Device
    std::string VFDeviceID = "";
    for (auto sample : ReaderVF_Device.read())
        if (sample.info().valid())
            VFDeviceID = sample.data().Device();
    sampleVF_Device_Active.Device(VFDeviceID); // Set Device for VF operation

    duration<float> IslandDelay;
    if (Immediate) {
        IslandDelay = seconds(0);
    }
    else {
        // Get the additional delay needed if the device is a generator else use the MinIslandDelay
        IslandDelay = MinIslandDelay;
        for (auto sample : ReaderInfo_Generator.read())
            if (sample.info().valid() && sample.data().Device() == VFDeviceID)
                IslandDelay = seconds(sample.data().RampUpTime()) + MinIslandDelay;
    }

    // Set the time to perform the island operation
    auto targetTime = high_resolution_clock::now() + IslandDelay;
    // Create the timestamp used in DDS communication
    auto ts = Energy::Common::Timestamp(duration_cast<seconds>(targetTime.time_since_epoch()).count(), 0);
    sampleVF_Device_Active.SwitchTime(ts); // Set time for VF switchover
    // Send the command for active VF Device
    WriterVF_Device_Active.write(sampleVF_Device_Active);

    if (!Immediate) {
        // Wait until it's time and then tell the interconnect it's time to open. The actual value may
        // differ from the time for the VF Device switch. This all depends on the delay between a device
        // getting a command to perform a mode switch and it actuially occuring and the delay between
        // the relay being told to open and it actually opening. The DDS communication is incredibly low
        // latency and can be ignored except for unusual circumstances.
        std::this_thread::sleep_until(targetTime);
        // Send the command to switch
        WriterControl_Device.write(sampleControlDevice);
    }

    // At this point the microgrid should be islanded and this thread is done.
    return;
}

/* IslandMicrogrid
* This function goes through the process of islanding the microgrid. The same function will be used within this example
* for both intentional and unintentional islanding operations. The only difference is whether or not the transition
* happens immediately or after a short wait.
*/
void IslandMicrogrid(
    dds::sub::DataReader<Energy::Ops::VF_Device> ReaderVF_Device,
    dds::sub::DataReader<Energy::Ops::Info_Generator> ReaderInfo_Generator,
    dds::pub::DataWriter<Energy::Ops::Control_Device> WriterControl_Device,
    dds::pub::DataWriter<Energy::Ops::VF_Device_Active> WriterVF_Device_Active,
    dds::pub::DataWriter<Energy::Ops::Status_Microgrid> WriterStatus_Microgrid)
{
    auto sampleStatus_Microgrid = Energy::Ops::Status_Microgrid(OptimizerID, Energy::Enums::MicrogridStatus::REQUEST_ISLAND);
    WriterStatus_Microgrid.write(sampleStatus_Microgrid);

    auto ThreadIsland = std::thread(IslandOperation, false, ReaderVF_Device, ReaderInfo_Generator, 
        WriterControl_Device, WriterVF_Device_Active);

    // Here is where code would be called to make the microgrid ready for off-grid operation if the
    // load and generation are not adequately sized.

    // Rejoin the thread, update the status, and return
    ThreadIsland.join();
    sampleStatus_Microgrid.MicrogridStatus(Energy::Enums::MicrogridStatus::ISLANDED);
    WriterStatus_Microgrid.write(sampleStatus_Microgrid);
    return;
}

/* UnintentionalIsland
* This function is called when it has been detected that the Main Interconnect has tripped off. Some systems can detect
* events such as this and automatically switch over to voltage and frequency support. As a belt and suspenders, or if
* the device on the network that should provide this support to the microgrid is incapable of this operation, this
* function should be utilized.
*/
void UnintentionalIsland(
    dds::sub::DataReader<Energy::Ops::VF_Device> ReaderVF_Device,
    dds::sub::DataReader<Energy::Ops::Info_Generator> ReaderInfo_Generator,
    dds::pub::DataWriter<Energy::Ops::Control_Device> WriterControl_Device,
    dds::pub::DataWriter<Energy::Ops::VF_Device_Active> WriterVF_Device_Active,
    dds::pub::DataWriter<Energy::Ops::Status_Microgrid> WriterStatus_Microgrid)
{
    auto sampleStatus_Microgrid = Energy::Ops::Status_Microgrid(OptimizerID, Energy::Enums::MicrogridStatus::ISLANDED);
    WriterStatus_Microgrid.write(sampleStatus_Microgrid);

    auto ThreadIsland = std::thread(IslandOperation, true, ReaderVF_Device, ReaderInfo_Generator,
        WriterControl_Device, WriterVF_Device_Active);

    // Here is where code would be called to perform emergency load shedding or other operations to maintain stability

    // Rejoin the thread and return
    ThreadIsland.join();
    return;
}

/* Resynchronize
* This function is called to resynchronize the microgrid to the main grid. The main interconnect usually won't close in
* until there is a phase match and the waveforms are crossing 0, so this function needs to tell the main internconnect
* to close and then wait till it does to tell the Active VF device to change mode away from VF.
*/
void Resynchronize(
    dds::sub::DataReader<Energy::Ops::Status_Device> ReaderStatus_Device,
    dds::pub::DataWriter<Energy::Ops::Control_Device> WriterControl_Device,
    dds::pub::DataWriter<Energy::Ops::VF_Device_Active> WriterVF_Device_Active,
    dds::pub::DataWriter<Energy::Ops::Status_Microgrid> WriterStatus_Microgrid)
{
    bool waiting = true; // variable that allows the while loop to exit

    auto sampleStatus_Microgrid = Energy::Ops::Status_Microgrid(OptimizerID, Energy::Enums::MicrogridStatus::REQUEST_RESYNC);
    WriterStatus_Microgrid.write(sampleStatus_Microgrid);

    // Create the Samples for Device Control and VF Active device
    auto sampleControlDevice = Energy::Ops::Control_Device(InterconnectID, OptimizerID, Energy::Enums::DeviceControl::CONNECT);
    auto sampleVF_Device_Active = Energy::Ops::VF_Device_Active(OptimizerID, "", Energy::Common::Timestamp(0, 0));

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
    std::vector<std::string> query_parameters = { "'" + InterconnectID + "'" };
    DataState commonDataState = DataState(SampleState::any(), ViewState::any(), InstanceState::alive());
    // Query Condition for Controlling the device. This is basic functionality for a grid connected device.
    QueryCondition QueryConditionStatus_Device(Query(ReaderStatus_Device, "Device MATCH %0", query_parameters),
        commonDataState, [&ReaderStatus_Device, &waiting](Condition condition) {
            auto condition_as_qc = polymorphic_cast<QueryCondition>(condition);
            auto samples = ReaderStatus_Device.select().condition(condition_as_qc).read();
            for (auto sample : samples)
                if (sample.info().valid() && sample.data().ConnectionStatus() == Energy::Enums::ConnectionStatus::CONNECTED)
                    waiting = false;
        }
    );

    // Set up the Waitset
    dds::core::cond::WaitSet waitset;
    waitset += QueryConditionStatus_Device;

    // Update the Control_Device topic to make the interconnect start the resync process
    WriterControl_Device.write(sampleControlDevice);

    // Here we are waiting forever because this is a simulated environment. Code changes for timeout
    // and probably canceling would be added for a real system.
    while (waiting)
        waitset.dispatch(dds::core::Duration(4));  // Wait up to 4s each time

    // Update VF_Device_Active with the empty DeviceID
    WriterVF_Device_Active.write(sampleVF_Device_Active);

    // At this point the microgrid is now part of the main grid.
    sampleStatus_Microgrid.MicrogridStatus(Energy::Enums::MicrogridStatus::CONNECTED);
    WriterStatus_Microgrid.write(sampleStatus_Microgrid);
    return;
}

/* ProcessVizCommand
* When there is an external command for microgrid operations there is a need to process the command to make sure that
* nothing weird happens, such as trying to island when you are already islanded.
*/
void ProcessVizCommand(
    Energy::Enums::MicrogridStatus command,
    dds::sub::DataReader<Energy::Ops::Status_Microgrid> ReaderStatus_Microgrid,
    dds::sub::DataReader<Energy::Ops::VF_Device> ReaderVF_Device,
    dds::sub::DataReader<Energy::Ops::Info_Generator> ReaderInfo_Generator,
    dds::sub::DataReader<Energy::Ops::Status_Device> ReaderStatus_Device,
    dds::pub::DataWriter<Energy::Ops::Control_Device> WriterControl_Device,
    dds::pub::DataWriter<Energy::Ops::VF_Device_Active> WriterVF_Device_Active,
    dds::pub::DataWriter<Energy::Ops::Status_Microgrid> WriterStatus_Microgrid)
{
    auto currentStatus = Energy::Enums::MicrogridStatus::CONNECTED; // This is the reasonable default

    const std::vector<std::string> query_parameters = { "'" + OptimizerID + "'" };
    dds::sub::cond::QueryCondition query_condition(
        dds::sub::Query(ReaderStatus_Microgrid, "Device MATCH %0", query_parameters),
        dds::sub::status::DataState::any_data());

    for (auto sample : ReaderStatus_Microgrid.select().condition(query_condition).read()) {
        if (sample.info().valid())
            currentStatus = sample.data().MicrogridStatus().underlying();
    }

    switch (currentStatus) {
        case Energy::Enums::MicrogridStatus::CONNECTED:
            if (Energy::Enums::MicrogridStatus::REQUEST_ISLAND == command.underlying())
                std::thread(IslandMicrogrid, ReaderVF_Device, ReaderInfo_Generator, WriterControl_Device,
                    WriterVF_Device_Active, WriterStatus_Microgrid).detach();
            break;
        case Energy::Enums::MicrogridStatus::ISLANDED:
            if (Energy::Enums::MicrogridStatus::REQUEST_RESYNC == command.underlying())
                std::thread(Resynchronize, ReaderStatus_Device, WriterControl_Device, WriterVF_Device_Active,
                    WriterStatus_Microgrid).detach();
            break;
        default:
            break;
    }
    return;
}

/* CheckTrip
* This function takes the change in status of the interconnect and compares it to the microgrid state to determine
* if a trip event has occured.
*/
void CheckTrip(
    dds::sub::DataReader<Energy::Ops::Status_Microgrid> ReaderStatus_Microgrid,
    dds::sub::DataReader<Energy::Ops::VF_Device> ReaderVF_Device,
    dds::sub::DataReader<Energy::Ops::Info_Generator> ReaderInfo_Generator,
    dds::pub::DataWriter<Energy::Ops::Control_Device> WriterControl_Device,
    dds::pub::DataWriter<Energy::Ops::VF_Device_Active> WriterVF_Device_Active,
    dds::pub::DataWriter<Energy::Ops::Status_Microgrid> WriterStatus_Microgrid)
{
    auto currentStatus = Energy::Enums::MicrogridStatus::CONNECTED; // This is the reasonable default

    const std::vector<std::string> query_parameters = { "'" + OptimizerID + "'" };
    dds::sub::cond::QueryCondition query_condition(
        dds::sub::Query(ReaderStatus_Microgrid, "Device MATCH %0", query_parameters),
        dds::sub::status::DataState::any_data());

    for (auto sample : ReaderStatus_Microgrid.select().condition(query_condition).read()) {
        if (sample.info().valid())
            currentStatus = sample.data().MicrogridStatus().underlying();
    }

    switch (currentStatus) {
    case Energy::Enums::MicrogridStatus::CONNECTED:
        std::thread(UnintentionalIsland, ReaderVF_Device, ReaderInfo_Generator, WriterControl_Device, WriterVF_Device_Active, 
            WriterStatus_Microgrid).detach();
        break;
    default:
        break;
    }
    return;
}

/* Optimize
* This is the function that pulls in all of the information from the devices on the microgrid and fiddles with the
* available knobs. Optimization is too big a problem for this example so we are going to only do simple operation
* parameters based on whether we're on or off grid. A full optimization is going to pull in a lot more information
* than we're using here, including device limits, voltage, current, and power measurement, forecasts, and price
* signals. We're just going to charge the battery when we're on-grid and try to use the battery as an intelligent
* load when we're off grid to run the generator at peak efficiency. We'll also be checking for a change in the VF
* Device ID due to changes in battery SOC.
*/
void Optimize(
    dds::sub::DataReader<Energy::Ops::Status_Microgrid> ReaderStatus_Microgrid,
    dds::sub::DataReader<Energy::Ops::Info_Generator> ReaderInfo_Generator,
    dds::sub::DataReader<Energy::Ops::Info_Battery> ReaderInfo_Battery,
    dds::sub::DataReader<Energy::Ops::VF_Device> ReaderVF_Device,
    dds::sub::DataReader<Energy::Ops::Meas_NodePower> ReaderMeas_NodePower,
    dds::sub::DataReader<Energy::Common::MMXU_Single_float32> ReaderMeas_SOC,
    dds::pub::DataWriter<Energy::Common::CNTL_Single_float32> WriterControl_Power,
    dds::pub::DataWriter<Energy::Ops::VF_Device_Active> WriterVF_Device_Active)
{
    using namespace std::chrono;
    const std::vector<std::string> query_parameters = { "'" + OptimizerID + "'" };
    dds::sub::cond::QueryCondition query_condition(
        dds::sub::Query(ReaderStatus_Microgrid, "Device MATCH %0", query_parameters),
        dds::sub::status::DataState::any_data());

    std::string VFDevice = "";
    bool VFDeviceChanged = false;

    auto currentStatus = Energy::Enums::MicrogridStatus::CONNECTED; // This is the reasonable default
    auto sampleControl_Power = Energy::Common::CNTL_Single_float32("", OptimizerID, 0.0);
    auto sampleVF_Device_Active = Energy::Ops::VF_Device_Active(OptimizerID, "", Energy::Common::Timestamp());

    // Optimization is a continuous process that goes on as long as the Controller is running.
    while (true) {
        for (auto sample : ReaderStatus_Microgrid.select().condition(query_condition).read()) {
            if (sample.info().valid() && sample.data().Device() == InterconnectID)
                currentStatus = sample.data().MicrogridStatus().underlying();
        }
        switch (currentStatus) {
        case Energy::Enums::MicrogridStatus::REQUEST_ISLAND:
            // It's likely there are going to be specific operations that need to be performed
            // during an island request. Just not in this example.
        case Energy::Enums::MicrogridStatus::CONNECTED:
            // Set all batteries to charge
            for (auto sample : ReaderMeas_SOC.take()) {
                if (sample.info().valid()) {
                    // Charge the battery to 95%
                    if (sample.data().Value() < 95.0) {
                        // Find the max charge we can do and do it.
                        for (auto info : ReaderInfo_Battery.read()) {
                            if (info.info().valid() && info.data().Device() == sample.data().Device()) {
                                sampleControl_Power.Device(info.data().Device());
                                sampleControl_Power.SetPoint(info.data().MaxLoad());
                                WriterControl_Power.write(sampleControl_Power);
                            }
                        }
                    }
                    // We don't want to overcharge it
                    else {
                        sampleControl_Power.Device(sample.data().Device());
                        sampleControl_Power.SetPoint(0.0);
                        WriterControl_Power.write(sampleControl_Power);
                    }
                }
            }
            break;
        case Energy::Enums::MicrogridStatus::REQUEST_RESYNC:
            // This is really still an island case.
        case Energy::Enums::MicrogridStatus::ISLANDED:
            // Get the VF Device ID once.
            VFDeviceChanged = false;
            for (auto vfdev : ReaderVF_Device.read())
                if (vfdev.info().valid()) {
                    // If there is a VF Device change, update the Active VF Device
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
            for (auto gen : ReaderInfo_Generator.read())
                if (gen.info().valid() && gen.data().Device() == VFDevice) {
                    if (VFDeviceChanged) {
                        auto ts = sampleVF_Device_Active.SwitchTime().Seconds() + gen.data().RampUpTime();
                        sampleVF_Device_Active.SwitchTime(Energy::Common::Timestamp(ts, 0));
                    }
                    // Charge or discharge the battery to maintain generator efficiency. In a real
                    // system load shedding and/or scheduling could also be used, as well as solar
                    // curtailment (although any system that's artificialy curtailing solar is
                    // wasting an investment)
                    float peakSetPoint = 0.0;
                    float peakEfficiency = 0.0;
                    // Go through all the values and find the best
                    for (auto val : gen.data().PowerEfficiency())
                        if (val.Efficiency() > peakEfficiency)
                            peakSetPoint = val.Value();
                    // Find out how much we need to modify battery setpoints to get the generator
                    // into peak efficiency
                    float genPowerDelta = 0.0;
                    for (auto power : ReaderMeas_NodePower.read())
                        if (power.info().valid() && power.data().Device() == VFDevice)
                            genPowerDelta = power.data().Value() - peakSetPoint;
                    // Go through the batteries and start changing setpoints
                    for (auto batt : ReaderInfo_Battery.read())
                        if (genPowerDelta == 0.0)
                            break;
                        else if (batt.info().valid())
                            for (auto power : ReaderMeas_NodePower.read())
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
                                    WriterControl_Power.write(sampleControl_Power);
                                }
                }
            if (VFDeviceChanged)
                WriterVF_Device_Active.write(sampleVF_Device_Active);
            break;
        default:
            break;
        }
    }
}

void publisher_main(int domain_id)
{
    using namespace dds::core;
    using namespace dds::topic;
    using namespace dds::pub;
    using namespace dds::sub;

    // Create the Domain Particimant QOS to set Entity Name
    dds::domain::qos::DomainParticipantQos qos_participant = dds::core::QosProvider::Default().participant_qos();
    rti::core::policy::EntityName entityName("Controller-" + OptimizerID);
    qos_participant << entityName;

    // Create a DomainParticipant with default Qos
    dds::domain::DomainParticipant participant(domain_id);

    // Create Topics -- and automatically register the types
    Topic<Energy::Ops::Meas_NodePower> TopicMeas_NodePower(participant, "Meas_NodePower");
    Topic<Energy::Ops::Status_Device> TopicStatus_Device(participant, "Status_Device");
    Topic<Energy::Ops::Info_Battery> TopicInfo_Battery(participant, "Info_Battery");
    Topic<Energy::Ops::Info_Generator> TopicInfo_Generator(participant, "Info_Generator");
    Topic<Energy::Ops::Info_Resource> TopicInfo_Resource(participant, "Info_Resource");
    Topic<Energy::Ops::Control_Device> TopicControl_Device(participant, "Control_Device");
    Topic<Energy::Common::CNTL_Single_float32> TopicControl_Power(participant, "Control_Power");
    Topic<Energy::Common::MMXU_Single_float32> TopicMeas_SOC(participant, "Meas_SOC");
    Topic<Energy::Ops::VF_Device> TopicVF_Device(participant, "VF_Device");
    Topic<Energy::Ops::VF_Device_Active> TopicVF_Device_Active(participant, "VF_Device_Active");
    Topic<Energy::Ops::Status_Microgrid> TopicStatus_Microgrid(participant, "Status_Microgrid");

    // Create Publisher
    Publisher publisher(participant);

    /* Create DataWriters with Qos */
    // Set the ownership strength of the Control Power to 100
    dds::pub::qos::DataWriterQos qos_control_power = QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Control");
    qos_control_power << dds::core::policy::OwnershipStrength(100);
    // Used to control setpoints for ES, Generator, and PV
    DataWriter<Energy::Common::CNTL_Single_float32> WriterControl_Power(publisher, TopicControl_Power, qos_control_power);
    // Actual Status of Microgrid 
    DataWriter<Energy::Ops::Status_Microgrid> WriterStatus_Microgrid(publisher, TopicStatus_Microgrid,
        QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Status"));
    // Used to tell all VF Devices which one will be active
    DataWriter<Energy::Ops::VF_Device_Active> WriterVF_Device_Active(publisher, TopicVF_Device_Active,
        QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Control"));
    // Used to control the Main Interconnect for island/resync
    DataWriter<Energy::Ops::Control_Device> WriterControl_Device(publisher, TopicControl_Device,
        QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Control"));    

    // Create Subscriber
    dds::sub::Subscriber subscriber(participant);

    /* Create DataReaders with Qos */
    // Gets power devices are consuming or supplying
    DataReader<Energy::Ops::Meas_NodePower> ReaderMeas_NodePower(subscriber, TopicMeas_NodePower,
        QosProvider::Default().datareader_qos("EnergyCommsLibrary::Measurement"));
    // Gets SOC for all ES units
    DataReader<Energy::Common::MMXU_Single_float32> ReaderMeas_SOC(subscriber, TopicMeas_SOC,
        QosProvider::Default().datareader_qos("EnergyCommsLibrary::Measurement"));
    // Gets Info for all ES units
    DataReader<Energy::Ops::Info_Battery> ReaderInfo_Battery(subscriber, TopicInfo_Battery,
        QosProvider::Default().datareader_qos("EnergyCommsLibrary::Info"));
    // Gets info for all generators
    DataReader<Energy::Ops::Info_Generator> ReaderInfo_Generator(subscriber, TopicInfo_Generator,
        QosProvider::Default().datareader_qos("EnergyCommsLibrary::Info"));
    // Gets general resource information
    DataReader<Energy::Ops::Info_Resource> ReaderInfo_Resource(subscriber, TopicInfo_Resource,
        QosProvider::Default().datareader_qos("EnergyCommsLibrary::Info"));
    // Gets the active VF Device based on ownership strength
    DataReader<Energy::Ops::VF_Device> ReaderVF_Device(subscriber, TopicVF_Device,
        QosProvider::Default().datareader_qos("EnergyCommsLibrary::Control"));
    // Gets commands from the viz on Microgrid operations
    DataReader<Energy::Ops::Status_Microgrid> ReaderStatus_Microgrid(subscriber, TopicStatus_Microgrid,
        QosProvider::Default().datareader_qos("EnergyCommsLibrary::Status"));
    // Gets generic status of devices
    DataReader<Energy::Ops::Status_Device> ReaderStatus_Device(subscriber, TopicStatus_Device,
        QosProvider::Default().datareader_qos("EnergyCommsLibrary::Status"));

    /* Create Query Conditions */
    using DataState = dds::sub::status::DataState;
    using SampleState = dds::sub::status::SampleState;
    using ViewState = dds::sub::status::ViewState;
    using InstanceState = dds::sub::status::InstanceState;
    using QueryCondition = dds::sub::cond::QueryCondition;
    using Condition = dds::core::cond::Condition;
    // Create query parameters
    std::vector<std::string> query_parameters = { "'" + VizID + "'" };
    DataState commonDataState = DataState( SampleState::not_read(), ViewState::any(), InstanceState::alive());
    // Query Condition for Controlling the Microgrid. This is basic functionality for a grid connected device.
    QueryCondition QueryConditionStatus_Microgrid( Query(ReaderStatus_Microgrid, "Device MATCH %0", query_parameters),
        commonDataState, [&ReaderStatus_Microgrid, &ReaderVF_Device, &ReaderInfo_Generator, &ReaderStatus_Device,
        &WriterControl_Device, &WriterVF_Device_Active, &WriterStatus_Microgrid](Condition condition) {
            auto condition_as_qc = polymorphic_cast<QueryCondition>(condition);
            auto samples = ReaderStatus_Microgrid.select().condition(condition_as_qc).take();
            for (auto sample : samples)
                if (sample.info().valid())
                    std::thread(ProcessVizCommand, sample.data().MicrogridStatus(), ReaderStatus_Microgrid, ReaderVF_Device,
                        ReaderInfo_Generator, ReaderStatus_Device, WriterControl_Device, WriterVF_Device_Active,
                        WriterStatus_Microgrid).detach();
        }
    );
    // Query Condition for watching if the Interconnect trips
    query_parameters = { "'" + InterconnectID + "'" };
    QueryCondition QueryConditionInterconnectStatus( Query(ReaderStatus_Device, "Device MATCH %0", query_parameters),
        commonDataState, [&ReaderStatus_Device, &ReaderStatus_Microgrid, &ReaderVF_Device, &ReaderInfo_Generator,
        &WriterControl_Device, &WriterVF_Device_Active, &WriterStatus_Microgrid](Condition condition) {
            auto condition_as_qc = polymorphic_cast<QueryCondition>(condition);
            auto samples = ReaderStatus_Device.select().condition(condition_as_qc).read();
            for (auto sample : samples)
                if (sample.info().valid() && Energy::Enums::ConnectionStatus::DISCONNECTED == sample.data().ConnectionStatus().underlying())
                    std::thread(CheckTrip, ReaderStatus_Microgrid, ReaderVF_Device, ReaderInfo_Generator, WriterControl_Device,
                        WriterVF_Device_Active, WriterStatus_Microgrid).detach();
        }
    );

    // Launch thread for continuous optimization
    std::thread threadOptimization(&Optimize, ReaderStatus_Microgrid, ReaderInfo_Generator, ReaderInfo_Battery, ReaderVF_Device,
        ReaderMeas_NodePower, ReaderMeas_SOC, WriterControl_Power, WriterVF_Device_Active);

    // Set up the Waitset
    dds::core::cond::WaitSet waitset;
    waitset += QueryConditionStatus_Microgrid;
    waitset += QueryConditionInterconnectStatus;
    
    // Here we are handling our waitset and reactions to inputs
    while (true) {
        // Dispatch will call the handlers associated to the WaitSet conditions when they activate
        waitset.dispatch(dds::core::Duration(4));  // Wait up to 4s each time
    }
}


int main(int argc, char* argv[])
{
    // To turn on additional logging, include <rti/config/Logger.hpp> and
    // uncomment the following line:
    // rti::config::Logger::instance().verbosity(rti::config::Verbosity::STATUS_ALL);

    try {
        publisher_main(0);
    }
    catch (const std::exception& ex) {
        // This will catch DDS exceptions
        std::cerr << "Exception in publisher_main(): " << ex.what() << std::endl;
        return -1;
    }

    // RTI Connext provides a finalize_participant_factory() method
    // if you want to release memory used by the participant factory singleton.
    // Uncomment the following line to release the singleton:
    //
    // dds::domain::DomainParticipant::finalize_participant_factory();

    return 0;
}