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

 /* MainInterconnect.cxx

 A simulated MainInterconnect Device

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

#include <dds/dds.hpp>

#include "EnergyComms.hpp"

using namespace Energy::Ops;
using namespace Energy::Common;
using namespace Energy::Enums;

using namespace dds::core;
using namespace dds::topic;
using namespace dds::pub;
using namespace dds::sub;

float SimMeasurement = 0;
bool connected = true;
std::string DeviceID = "SampleInterconnect";
std::string NodeID = "001";
std::string NodeIDGrid = "000";
ConnectionStatus connectionStatus = ConnectionStatus::CONNECTED;
OperationStatus operationStatus = OperationStatus::ENABLED_ON;

/* StatusMonitor
* In this example we are watching for the internal status to change, and when it
* does to publish a new status.
*/
void StatusMonitor(DataWriter<Status_Device> WriterStatus_Device)
{
    Status_Device sample(DeviceID, connectionStatus, operationStatus);

    //Perform initial status write
    WriterStatus_Device.write(sample);

    while (true)
    {
        // When there is a change to the global variables, send out a new sample
        if (sample.ConnectionStatus() != connectionStatus ||
            sample.OperationStatus() != operationStatus) {
            sample.ConnectionStatus(connectionStatus);
            sample.OperationStatus(operationStatus);
            WriterStatus_Device.write(sample);
            std::cout << "Status Change.\n";
        }
        // When no change has occured, sleep for 100 ms
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/* InterconnectControl
* In this example we are only changing the published status and setting the
* simulated measurement parameter appropriately.
*/
void InterconnectControl(DeviceControl command)
{
    // Here is where code would go to interface with the actual relay connecting
    // the microgrid to the larger grid. Based on its response, the
    // corresponding status would be updated. If this is a lengthy process, then
    // a thread should  probably be spawned that would allow status updates to
    // be sent out while letting the device process other incoming messages.

    switch (command.underlying())
    {
    case DeviceControl::CONNECT:
        connected = true;
        connectionStatus = ConnectionStatus::CONNECTED;
        operationStatus = OperationStatus::ENABLED_ON;
        break;
    case DeviceControl::DISCONNECT:
        connected = false;
        connectionStatus = ConnectionStatus::DISCONNECTED;
        operationStatus = OperationStatus::DISABLED_READY;
        break;
    default:
        break;
    }
}

/* GetMeasurement
* In this example we have two measurements, one for each side of the
* interconnect. This could be the case when the load is on a single phase or if
* the only thing that needs to be returned (or is available) is the aggregate.
* This, along with the data model, would need to be changed to pass information
* on a 3-phase system.
*/
float GetMeasurement()
{
    // Some sort of communication to the actual system would be here. In our
    // case we're just going to pull from the simulated measurement variable

    // We are adding a delay here to simulate the actual fetch of information
    // from the system
    std::chrono::milliseconds timespan(90 + std::rand() % 21); // 90 - 110 milliseconds
    std::this_thread::sleep_for(timespan);

    if (connected)
        return SimMeasurement;
    else
        return 0.0;
}

/* ContinuousWriter
* In this example we are using a function in a separate thread to continuously
* publish measurement data. Depending on whether or not other interfaces are
* thread safe additional semaphores or locks would need to be introduced when
* accessing outside interfaces between multiple threads. We are not doing that
* here because the data being published is simulated.
*/
void ContinuousWriter(DataWriter<Meas_NodePower> WriterMeas_NodePower)
{
    Meas_NodePower sampleMeas_NodePower(DeviceID, SimMeasurement, NodeID);

    while (true) {
        // Modify the measurement data to be written here
        sampleMeas_NodePower.Value(GetMeasurement()); // This is a blocking call that times the loop.
        // Set NodeID to Microgrid Side
        sampleMeas_NodePower.Node(NodeID);

        // Write the measurement data
        WriterMeas_NodePower.write(sampleMeas_NodePower);

        // Update NodeID for grid side and write sample again
        sampleMeas_NodePower.Node(NodeIDGrid);
        WriterMeas_NodePower.write(sampleMeas_NodePower);
    }
}

void publisher_main(int domain_id)
{
    // Create the Domain Particimant QOS to set Entity Name
    auto qos_default = dds::core::QosProvider::Default();
    auto qos_participant = qos_default.participant_qos();
    rti::core::policy::EntityName entityName("Interconnect-" + DeviceID);
    qos_participant << entityName;

    // Get the QOS for readers and writers
    auto qos_writer_measurement =
        qos_default.datawriter_qos("EnergyCommsLibrary::Measurement");
    auto qos_writer_status =
        qos_default.datawriter_qos("EnergyCommsLibrary::Status");
    auto qos_reader_control =
        qos_default.datareader_qos("EnergyCommsLibrary::Control");

    // Create a DomainParticipant with default Qos
    dds::domain::DomainParticipant participant(domain_id, qos_participant);

    // Create Topics -- and automatically register the types
    Topic<Meas_NodePower> TopicMeas_NodePower(participant, "Meas_NodePower");
    Topic<Status_Device> TopicStatus_Device(participant, "Status_Device");
    Topic<Control_Device> TopicControl_Device(participant, "Control_Device");
    Topic<CNTL_Single_float32> TopicControl_Power(participant, "Control_Power");

    // Create Publisher
    dds::pub::Publisher publisher(participant);

    // Create DataWriters with Qos
    DataWriter<Meas_NodePower> WriterMeas_NodePower(
        publisher, TopicMeas_NodePower, qos_writer_measurement);
    DataWriter<Status_Device> WriterStatus_Device(
        publisher, TopicStatus_Device, qos_writer_status);

    // Create Subscriber
    dds::sub::Subscriber subscriber(participant);

    // Create DataReaders with Qos
    DataReader<Control_Device> ReaderControl_Device(
        subscriber, TopicControl_Device, qos_reader_control);
    DataReader<CNTL_Single_float32> ReaderControl_Power(
        subscriber, TopicControl_Power, qos_reader_control);

    /* Create Query Conditions */
    // Create query parameters
    std::vector<std::string> query_parameters = { "'" + DeviceID + "'" };
    dds::sub::status::DataState commonDataState = dds::sub::status::DataState(
        dds::sub::status::SampleState::not_read(),
        dds::sub::status::ViewState::any(),
        dds::sub::status::InstanceState::alive());
    // Query Condition for Controlling the device. This is basic functionality
    // for a grid connected device.
    dds::sub::cond::QueryCondition QueryConditionControl_Device(
        dds::sub::Query(ReaderControl_Device, "Device MATCH %0", query_parameters),
        commonDataState,
        [&ReaderControl_Device](dds::core::cond::Condition condition) {
            auto condition_as_qc =
                dds::core::polymorphic_cast<dds::sub::cond::QueryCondition>(condition);
            auto samples =
                ReaderControl_Device.select().condition(condition_as_qc).take();
            for (auto sample : samples)
            {
                // All valid samples will be processed and execute the following
                // function
                if (sample.info().valid())
                    InterconnectControl(sample.data().Command());
            }
        }
    );
    // Query Condition for power setting. This is used for sim and comes from
    // the PowerFlowSim
    dds::sub::cond::QueryCondition QueryConditionControl_Power(
        dds::sub::Query(ReaderControl_Power, "Device MATCH %0", query_parameters),
        commonDataState,
        [&ReaderControl_Power](dds::core::cond::Condition condition) {
            auto condition_as_qc =
                dds::core::polymorphic_cast<dds::sub::cond::QueryCondition>(condition);
            auto samples =
                ReaderControl_Power.select().condition(condition_as_qc).read();
            for (auto sample : samples)
                if (sample.info().valid()) {
                    SimMeasurement = sample.data().SetPoint();

                }
        }
    );


    // Launch thread for continuous node measurement writes, status updates, and
    // VF Device Writes
    std::thread threadMeas(&ContinuousWriter, WriterMeas_NodePower);
    std::thread threadStatus(&StatusMonitor, WriterStatus_Device);

    // Set up the Waitset
    dds::core::cond::WaitSet waitset;
    waitset += QueryConditionControl_Device;
    waitset += QueryConditionControl_Power;

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
