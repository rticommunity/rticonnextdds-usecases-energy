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

 /* PV.cxx

 A simulated PV Device

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

#include "../../_Common/EnergyComms.hpp"

float SimIrradiance = 0;
float SimMeasurement = 0;
bool connected = true;
std::string DeviceID = "SamplePV";
std::string NodeID = "002";
float MaxLoad = 0.0;
float MaxGeneration = 8.0;

/* InterconnectControl
* In this example we are only changing the published status and setting the simulated measurement parameter
* appropriately.
*/
void InterconnectControl(Energy::Enums::DeviceControl command, dds::pub::DataWriter<Energy::Ops::Status_Device> WriterStatus_Device)
{
    //Here is where code would go to interface with the actual relay connecting the device to the grid. Based on its
    //response, the corresponding status would be updated. If this is a lengthy process, then a thread should probably
    //be spawned that would allow status updates to be sent out while letting the device process other incoming
    //messages.

    Energy::Ops::Status_Device sampleStatus_Device(DeviceID,
        Energy::Enums::ConnectionStatus::CONNECTED,
        Energy::Enums::OperationStatus::ENABLED_ON);

    switch (command.underlying())
    {
    case Energy::Enums::DeviceControl::CONNECT:
        connected = true;
        break;
    case Energy::Enums::DeviceControl::DISCONNECT:
        connected = false;
        sampleStatus_Device.ConnectionStatus(Energy::Enums::ConnectionStatus::DISCONNECTED);
        sampleStatus_Device.OperationStatus(Energy::Enums::OperationStatus::DISABLED_READY);
        break;
    default:
        break;
    }

    WriterStatus_Device.write(sampleStatus_Device);
}

/* GetMeasurement
* In this example we have a single measurement. This could be the case when the load is on a single phase or if the
* only thing that needs to be returned (or is available) is the aggregate. This, along with the data model, would need
* to be changed to pass information on a 3-phase system.
*/
float GetMeasurement()
{
    // Some sort of communication to the actual system would be here. In our case we're just going to calculate the
    // power based on curtailment and irradiance

    // We are adding a delay here to simulate the actual fetch of information from the system
    std::chrono::milliseconds timespan(90 + std::rand() % 21); // 90 - 110 milliseconds
    std::this_thread::sleep_for(timespan);

    float curtailedValue = SimMeasurement < MaxGeneration ? SimMeasurement : MaxGeneration;
    curtailedValue = curtailedValue > 0.0 ? curtailedValue : 0.0;
    float radiantValue = MaxGeneration * SimIrradiance / 1000.0;
    radiantValue = radiantValue > 0.0 ? radiantValue : 0.0;

    if (connected)
        return curtailedValue < radiantValue ? curtailedValue : radiantValue;
    else
        return 0.0;
}

/* ContinuousWriter
* In this example we are using a function in a seperate thread to continously publish measurement data. Depending on
* whether or not other interfaces are thread safe additional semaphores or locks would need to be introduced when
* accessing outside interfaces between multiple threads. We are not doing that here because the data being published
* is simulated.
*/
void ContinuousWriter(dds::pub::DataWriter<Energy::Ops::Meas_NodePower> WriterMeas_NodePower)
{
    Energy::Ops::Meas_NodePower sampleMeas_NodePower(DeviceID, SimMeasurement, NodeID);

    while (true) {
        // Modify the measurement data to be written here
        sampleMeas_NodePower.Value(GetMeasurement());

        // Write the measurement data
        WriterMeas_NodePower.write(sampleMeas_NodePower);
    }
}

void publisher_main(int domain_id)
{
    // Create the Domain Particimant QOS to set Entity Name
    dds::domain::qos::DomainParticipantQos qos_participant = dds::core::QosProvider::Default().participant_qos();
    rti::core::policy::EntityName entityName("PV-" + DeviceID);
    qos_participant << entityName;

    // Create a DomainParticipant with default Qos
    dds::domain::DomainParticipant participant(domain_id);

    // Create Topics -- and automatically register the types
    dds::topic::Topic<Energy::Ops::Meas_NodePower> TopicMeas_NodePower(participant, "Meas_NodePower");
    dds::topic::Topic<Energy::Ops::Info_Resource> TopicInfo_Resource(participant, "Info_Resource");
    dds::topic::Topic<Energy::Ops::Status_Device> TopicStatus_Device(participant, "Status_Device");
    dds::topic::Topic<Energy::Ops::Control_Device> TopicControl_Device(participant, "Control_Device");
    dds::topic::Topic<Energy::Common::CNTL_Single_float32> TopicControl_Power(participant, "Control_Power");
    dds::topic::Topic<Energy::Common::CNTL_Single_float32> TopicControl_Irradiance(participant, "Control_Irradiance");


    // Create Publisher
    dds::pub::Publisher publisher(participant);

    // Create DataWriters with Qos
    dds::pub::DataWriter<Energy::Ops::Meas_NodePower> WriterMeas_NodePower(publisher, TopicMeas_NodePower,
        dds::core::QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Measurement"));
    dds::pub::DataWriter<Energy::Ops::Info_Resource> WriterInfo_Resource(publisher, TopicInfo_Resource,
        dds::core::QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Info"));
    dds::pub::DataWriter<Energy::Ops::Status_Device> WriterStatus_Device(publisher, TopicStatus_Device,
        dds::core::QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Status"));
    // Set the ownership strength of the Control Load to 0 to start with
    dds::pub::qos::DataWriterQos qos_control_load = dds::core::QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Control");
    qos_control_load << dds::core::policy::OwnershipStrength(0);
    dds::pub::DataWriter<Energy::Common::CNTL_Single_float32> WriterControl_Power(publisher, TopicControl_Power, qos_control_load);
    dds::pub::DataWriter<Energy::Common::CNTL_Single_float32> WriterControl_Irradiance(publisher, TopicControl_Irradiance, qos_control_load);

    // Create Subscriber
    dds::sub::Subscriber subscriber(participant);

    // Create DataReaders with Qos
    dds::sub::DataReader<Energy::Ops::Control_Device> ReaderControl_Device(subscriber, TopicControl_Device,
        dds::core::QosProvider::Default().datareader_qos("EnergyCommsLibrary::Control"));
    dds::sub::DataReader<Energy::Common::CNTL_Single_float32> ReaderControl_Power(subscriber, TopicControl_Power,
        dds::core::QosProvider::Default().datareader_qos("EnergyCommsLibrary::Control"));
    dds::sub::DataReader<Energy::Common::CNTL_Single_float32> ReaderControl_Irradiance(subscriber, TopicControl_Irradiance,
        dds::core::QosProvider::Default().datareader_qos("EnergyCommsLibrary::Control"));

    /* Create Query Conditions */
    // Create query parameters
    std::vector<std::string> query_parameters = { "'" + DeviceID + "'" };
    // Query Condition for Controlling the device. This is basic functionality for a grid connected device.
    dds::sub::cond::QueryCondition QueryConditionControl_Device(
        dds::sub::Query(ReaderControl_Device, "Device MATCH %0", query_parameters),
        dds::sub::status::DataState(
            dds::sub::status::SampleState::not_read(),
            dds::sub::status::ViewState::any(),
            dds::sub::status::InstanceState::alive()),
        [&ReaderControl_Device, &WriterStatus_Device](dds::core::cond::Condition condition) {
            auto condition_as_qc = dds::core::polymorphic_cast<dds::sub::cond::QueryCondition>(condition);
            auto samples = ReaderControl_Device.select().condition(condition_as_qc).read();
            for (auto sample : samples)
            {
                // All valid samples will be processed and execute the following function
                if (sample.info().valid())
                    InterconnectControl(sample.data().Command(), WriterStatus_Device);
            }
        }
    );
    // Query Condition for power curtailment. This is basic functionality for a PV system
    dds::sub::cond::QueryCondition QueryConditionControl_Power(
        dds::sub::Query(ReaderControl_Power, "Device MATCH %0", query_parameters),
        dds::sub::status::DataState(
            dds::sub::status::SampleState::not_read(),
            dds::sub::status::ViewState::any(),
            dds::sub::status::InstanceState::alive()),
        [&ReaderControl_Power](dds::core::cond::Condition condition) {
            auto condition_as_qc = dds::core::polymorphic_cast<dds::sub::cond::QueryCondition>(condition);
            auto samples = ReaderControl_Power.select().condition(condition_as_qc).read();
            for (auto sample : samples)
            {
                // All valid samples will be processed and set the global variable
                if (sample.info().valid())
                    SimMeasurement = sample.data().SetPoint();
            }
        }
    );
    // Query Condition for Irradiance. This is used for simulation only
    dds::sub::cond::QueryCondition QueryConditionControl_Irradiance(
        dds::sub::Query(ReaderControl_Irradiance, "Device MATCH %0", query_parameters),
        dds::sub::status::DataState(
            dds::sub::status::SampleState::not_read(),
            dds::sub::status::ViewState::any(),
            dds::sub::status::InstanceState::alive()),
        [&ReaderControl_Irradiance](dds::core::cond::Condition condition) {
            auto condition_as_qc = dds::core::polymorphic_cast<dds::sub::cond::QueryCondition>(condition);
            auto samples = ReaderControl_Irradiance.select().condition(condition_as_qc).read();
            for (auto sample : samples)
            {
                // All valid samples will be processed and set the global variable
                if (sample.info().valid())
                    SimIrradiance = sample.data().SetPoint();
            }
        }
    );

    // Create Sample objects for datawriters (except for Meas_NodePower, which is handled in another thread)
    Energy::Ops::Info_Resource sampleInfo_Resource(DeviceID, NodeID, MaxLoad, MaxGeneration);
    Energy::Ops::Status_Device sampleStatus_Device(DeviceID,
        Energy::Enums::ConnectionStatus::CONNECTED,
        Energy::Enums::OperationStatus::ENABLED_ON);
    Energy::Common::CNTL_Single_float32 sampleControl_Power(DeviceID, DeviceID, 8.0);
    Energy::Common::CNTL_Single_float32 sampleControl_Irradiance(DeviceID, DeviceID, 700.0);

    // Write Info, Status, and Load Control. Each of these topics should only change due to exception.
    WriterInfo_Resource.write(sampleInfo_Resource);
    WriterStatus_Device.write(sampleStatus_Device);
    WriterControl_Power.write(sampleControl_Power);
    WriterControl_Irradiance.write(sampleControl_Irradiance);

    // Get status conditions
    dds::core::cond::StatusCondition StatusConditionControl_Device(ReaderControl_Device);
    dds::core::cond::StatusCondition StatusConditionControl_Power(ReaderControl_Power);
    // Set enabled statuses
    StatusConditionControl_Device.enabled_statuses(
        dds::core::status::StatusMask::liveliness_changed());
    StatusConditionControl_Power.enabled_statuses(
        dds::core::status::StatusMask::liveliness_changed());
    // Lambda functions for the status conditions
    // If there is a liveliness change for device control, 
    StatusConditionControl_Device->handler(
        [&ReaderControl_Device, &WriterStatus_Device](dds::core::cond::Condition condition) {
            dds::core::status::StatusMask status_mask = ReaderControl_Device.status_changes();
            if ((status_mask & dds::core::status::StatusMask::liveliness_changed()).any()) {
                auto condition_as_qc = dds::core::polymorphic_cast<dds::sub::cond::QueryCondition>(condition);
                auto samples = ReaderControl_Device.select().condition(condition_as_qc).read();
                for (auto sample : samples)
                {
                    // All valid samples will be processed and execute the following function
                    if (sample.info().valid())
                        InterconnectControl(sample.data().Command(), WriterStatus_Device);
                }
            }
        }
    );
    // If there is a liveliness change for power control,
    StatusConditionControl_Power->handler(
        [&ReaderControl_Power](dds::core::cond::Condition condition) {
            dds::core::status::StatusMask status_mask = ReaderControl_Power.status_changes();
            if ((status_mask & dds::core::status::StatusMask::liveliness_changed()).any()) {
                auto condition_as_qc = dds::core::polymorphic_cast<dds::sub::cond::QueryCondition>(condition);
                auto samples = ReaderControl_Power.select().condition(condition_as_qc).read();
                for (auto sample : samples)
                {
                    // All valid samples will be processed and execute the following function
                    if (sample.info().valid())
                        SimMeasurement = sample.data().SetPoint();
                }
            }
        }
    );

    // Launch thread for continuous node measurement writes
    std::thread threadObj(&ContinuousWriter, WriterMeas_NodePower);

    // Set up the Waitset
    dds::core::cond::WaitSet waitset;
    waitset += QueryConditionControl_Device;
    waitset += QueryConditionControl_Power;
    waitset += QueryConditionControl_Irradiance;
    //waitset += StatusConditionControl_Device;
    //waitset += StatusConditionControl_Power;

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