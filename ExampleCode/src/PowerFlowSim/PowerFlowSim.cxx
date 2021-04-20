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

#include "EnergyComms.hpp"

const std::string DeviceID = "PowerFlowSim";
const std::string OptimizerID = "SampleOpt";
const std::string InterconnectID = "SampleInterconnect";

/* BalanceConnected
* This sets the power levels that the Main Interconnect will report based on all device measurements
*/
void BalanceConnected(
    dds::sub::DataReader<Energy::Ops::Meas_NodePower> ReaderMeas_NodePower,
    dds::pub::DataWriter<Energy::Common::CNTL_Single_float32> WriterControl_Power)
{
    // Add up all of the power
    float powerSum = 0.0;
    for (auto meas : ReaderMeas_NodePower.read()) {
        if (meas.info().valid())
            powerSum += meas.data().Value();
    }
    // Take the result and set the interconnect power
    auto sample = Energy::Common::CNTL_Single_float32(InterconnectID, DeviceID, powerSum);
    WriterControl_Power.write(sample);
}

/* BalanceIsland
* This takes all of the measurements (except for the VF Device) and sets the necessary power output of the VF Device
*/
void BalanceIsland(
    dds::sub::DataReader<Energy::Ops::VF_Device_Active> ReaderVF_Device_Active,
    dds::sub::DataReader<Energy::Ops::Meas_NodePower> ReaderMeas_NodePower,
    dds::pub::DataWriter<Energy::Common::CNTL_Single_float32> WriterControl_Power)
{
    // Get the VF Device ID
    std::string VFDeviceID = "";
    for (auto dev : ReaderVF_Device_Active.read())
        if (dev.info().valid())
            VFDeviceID = dev.data().Device();

    // Add up all the power
    float powerSum = 0.0;
    for (auto meas : ReaderMeas_NodePower.read()) {
        if (meas.info().valid() && meas.data().Device() != VFDeviceID)
            powerSum += meas.data().Value();
    }

    // Take the result and set the VF Device power
    auto sample = Energy::Common::CNTL_Single_float32(VFDeviceID, DeviceID, powerSum);
    WriterControl_Power.write(sample);
}

void publisher_main(int domain_id)
{
    using namespace dds::core;
    using namespace dds::topic;
    using namespace dds::pub;
    using namespace dds::sub;

    // Create the Domain Particimant QOS to set Entity Name
    dds::domain::qos::DomainParticipantQos qos_participant = dds::core::QosProvider::Default().participant_qos();
    rti::core::policy::EntityName entityName("Sim-" + DeviceID);
    qos_participant << entityName;

    // Create a DomainParticipant with default Qos
    dds::domain::DomainParticipant participant(domain_id);

    // Create Topics -- and automatically register the types
    Topic<Energy::Ops::Meas_NodePower> TopicMeas_NodePower(participant, "Meas_NodePower");
    Topic<Energy::Common::CNTL_Single_float32> TopicControl_Power(participant, "Control_Power");
    Topic<Energy::Ops::VF_Device_Active> TopicVF_Device_Active(participant, "VF_Device_Active");
    Topic<Energy::Ops::Status_Microgrid> TopicStatus_Microgrid(participant, "Status_Microgrid");

    // Create Publisher
    Publisher publisher(participant);

    /* Create DataWriters with Qos */
    // Set the ownership strength of the Control Power to 200
    dds::pub::qos::DataWriterQos qos_control_power = QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Control");
    qos_control_power << dds::core::policy::OwnershipStrength(200);
    // Used to control setpoints for ES, Generator, and PV
    DataWriter<Energy::Common::CNTL_Single_float32> WriterControl_Power(publisher, TopicControl_Power, qos_control_power);
    
    // Create Subscriber
    dds::sub::Subscriber subscriber(participant);

    /* Create DataReaders with Qos */
    // Gets power devices are consuming or supplying
    DataReader<Energy::Ops::Meas_NodePower> ReaderMeas_NodePower(subscriber, TopicMeas_NodePower,
        QosProvider::Default().datareader_qos("EnergyCommsLibrary::Measurement"));
   // Gets the active VF Device based on ownership strength
    DataReader<Energy::Ops::VF_Device_Active> ReaderVF_Device_Active(subscriber, TopicVF_Device_Active,
        QosProvider::Default().datareader_qos("EnergyCommsLibrary::Control"));
    // Gets commands from the viz on Microgrid operations
    DataReader<Energy::Ops::Status_Microgrid> ReaderStatus_Microgrid(subscriber, TopicStatus_Microgrid,
        QosProvider::Default().datareader_qos("EnergyCommsLibrary::Status"));
    
    auto currentStatus = Energy::Enums::MicrogridStatus::CONNECTED; // This is the reasonable default

    const std::vector<std::string> query_parameters = { "'" + OptimizerID + "'" };
    dds::sub::cond::QueryCondition query_condition(
        dds::sub::Query(ReaderStatus_Microgrid, "Device MATCH %0", query_parameters),
        dds::sub::status::DataState::any_data());

    while (true) {
        for (auto sample : ReaderStatus_Microgrid.select().condition(query_condition).read()) {
            if (sample.info().valid())
                currentStatus = sample.data().MicrogridStatus().underlying();
        }

        switch (currentStatus) {
        case Energy::Enums::MicrogridStatus::REQUEST_ISLAND:
        case Energy::Enums::MicrogridStatus::CONNECTED:
            BalanceConnected(ReaderMeas_NodePower, WriterControl_Power);
            break;
        case Energy::Enums::MicrogridStatus::REQUEST_RESYNC:
        case Energy::Enums::MicrogridStatus::ISLANDED:
            BalanceIsland(ReaderVF_Device_Active, ReaderMeas_NodePower, WriterControl_Power);
            break;
        default:
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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