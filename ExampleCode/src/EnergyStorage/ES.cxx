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

 /* ES.cxx

 A simulated Energy Storage Device

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

float SimSOC = 50.0;
float SimMeasurement = 0;
bool connected = true;
bool ActiveVF = false;
Energy::Common::Timestamp SwitchTime;
const std::string DeviceID = "SampleES";
const std::string NodeID = "002";
const float MaxLoad = -8.0;
const float MaxGeneration = 8.0;
const float Capacity = 8.0;
Energy::Enums::ConnectionStatus ConnectionStatus = Energy::Enums::ConnectionStatus::DISCONNECTED;
Energy::Enums::OperationStatus OperationStatus = Energy::Enums::OperationStatus::DISABLED_OFF;

const std::chrono::duration<float> MaxTimeToWait = std::chrono::seconds(300);

/* StatusMonitor
* In this example we are watching for the internal status to change, and when it does to publish a new status.
*/
void StatusMonitor(dds::pub::DataWriter<Energy::Ops::Status_Device> WriterStatus_Device)
{
    Energy::Ops::Status_Device sample(DeviceID, ConnectionStatus, OperationStatus);

    //Perform initial status write
    WriterStatus_Device.write(sample);

    while (true)
    {
        // When there is a change to the global variables, send out a new sample
        if (sample.ConnectionStatus() != ConnectionStatus || sample.OperationStatus() != OperationStatus) {
            sample.ConnectionStatus(ConnectionStatus);
            sample.OperationStatus(OperationStatus);
            WriterStatus_Device.write(sample);
        }
        // When no change has occured, sleep for 100 ms
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


/* InterconnectControl
* In this example we are responding to a command to connect or disconnect and changing the appropriate status.
*/
void InterconnectControl(Energy::Enums::DeviceControl command)
{
    //Here is where code would go to interface with the actual relay connecting the device to the grid. Based on its
    //response, the corresponding status would be updated. If this is a lengthy process, then a thread should probably
    //be spawned that would allow status updates to be sent out while letting the device process other incoming
    //messages.

    switch (command.underlying())
    {
    case Energy::Enums::DeviceControl::CONNECT:
        connected = true;
        ConnectionStatus = Energy::Enums::ConnectionStatus::CONNECTED;
        OperationStatus = Energy::Enums::OperationStatus::ENABLED_ON;
        break;
    case Energy::Enums::DeviceControl::DISCONNECT:
        connected = false;
        ConnectionStatus = Energy::Enums::ConnectionStatus::DISCONNECTED;
        OperationStatus = Energy::Enums::OperationStatus::DISABLED_READY;
        break;
    default:
        break;
    }
}

/* GetMeasurement
* In this example we have a single measurement. This could be the case when the load is on a single phase or if the
* only thing that needs to be returned (or is available) is the aggregate. This, along with the data model, would need
* to be changed to pass information on a 3-phase system.
*/
float GetMeasurement()
{
    // Some sort of communication to the actual system would be here. In our case we're just going to pull from the
    // simulated measurement variable

    // We are adding a delay here to simulate the actual fetch of information from the system
    std::chrono::milliseconds timespan(90 + std::rand() % 21); // 90 - 110 milliseconds
    std::this_thread::sleep_for(timespan);

    float meas = SimMeasurement < MaxGeneration ? SimMeasurement : MaxGeneration;
    meas = meas > MaxLoad ? meas : MaxLoad;

    if (SimSOC >= 100.0)
        meas = meas > 0.0 ? meas : 0.0;
    else if (SimSOC <= 0.0)
        meas = meas < 0.0 ? meas : 0.0;

    // We are also going to adjust the SOC based on the measured value and the timespan.
    SimSOC = SimSOC - meas * ((float)timespan.count() / 3600000) / Capacity;

    if (connected)
        return meas;
    else
        return 0.0;
}

float GetSOC()
{
    if (SimSOC >= 100.0)
        SimSOC = 100.0;
    else if (SimSOC <= 0.0)
        SimSOC = 0.0;

    return SimSOC;
}

/* ContinuousWriter
* In this example we are using a function in a seperate thread to continously publish measurement data. Depending on
* whether or not other interfaces are thread safe additional semaphores or locks would need to be introduced when
* accessing outside interfaces between multiple threads. We are not doing that here because the data being published
* is simulated.
*
* For this we are doing Node Measurement and SOC. Many times SOC is seen within status type topics or messages. Here we
* treat it as a measurement.
*/
void ContinuousWriter(dds::pub::DataWriter<Energy::Ops::Meas_NodePower> WriterMeas_NodePower,
    dds::pub::DataWriter<Energy::Common::MMXU_Single_float32> WriterMeas_SOC)
{
    Energy::Ops::Meas_NodePower sampleMeas_NodePower(DeviceID, SimMeasurement, NodeID);
    Energy::Common::MMXU_Single_float32 sampleMeas_SOC(DeviceID, SimSOC);

    while (true) {
        // Modify the measurement data to be written here
        sampleMeas_NodePower.Value(GetMeasurement());
        sampleMeas_SOC.Value(GetSOC());

        // Write the measurement data
        WriterMeas_NodePower.write(sampleMeas_NodePower);
        WriterMeas_SOC.write(sampleMeas_SOC);
    }
}

/* ContinuousVFStrength
* This helps support VF device switching by having every device provide a relative strength as a VF device. The actual
* math used in a given system will probably vary, but something like this should be appropriate. The goal of this
* particular math is to force the generator to take over at some point when SOC is below 20% and for the battery to
* take over when SOC goes above 80%.
*/
void ContinuousVFStrength(dds::pub::DataWriter<Energy::Ops::VF_Device> WriterVF_Device)
{
    Energy::Ops::VF_Device dev(DeviceID);
    int32_t str = 0;
    dds::pub::qos::DataWriterQos QosVF_Device = WriterVF_Device.qos();

    while (true) {
        if (SimSOC <= 20.0)
            str = (int32_t)(SimSOC * MaxGeneration);
        else if (SimSOC >= 80.0)
            str = (int32_t)(SimSOC * MaxGeneration * 100);
        else
            str = (int32_t)(SimSOC * MaxGeneration * 10);

        // Let specified operation status conditions modify strength
        if (OperationStatus == Energy::Enums::OperationStatus::DISABLED_ERROR)
            str = 0;
        if (OperationStatus == Energy::Enums::OperationStatus::DISABLED_OFF)
            str = 0;

        QosVF_Device << dds::core::policy::OwnershipStrength(str);
        WriterVF_Device.qos(QosVF_Device);
        WriterVF_Device.write(dev);

        // We are adding a delay here of 500 milliseconds. In reality this is probably overkill
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void VFDeviceActivity(Energy::Common::Timestamp ts)
{
    using Clock = std::chrono::high_resolution_clock;
    using Seconds = std::chrono::seconds;
    using Nanoseconds = std::chrono::nanoseconds;
    using Duration = std::chrono::duration<float>;

    std::chrono::time_point<Clock> targetTime(Seconds(ts.Seconds()) + Nanoseconds(ts.Fraction()));

    // Check to make sure that something isn't wrong and the scheduled time to wait to transition is greater that the
    // configured max time to wait. For an actual application this would need some kind of status feedback for safety.
    if (std::chrono::duration_cast<Duration>(targetTime - Clock::now()) > MaxTimeToWait) {
        std::cerr << "Time to switch to VF greater than Max Allowed Time.\n";
        return;
    }

    // Return VF Ready status to system
    OperationStatus == Energy::Enums::OperationStatus::ENABLED_VF_READY;

    std::this_thread::sleep_until(targetTime);

    // At this point the device has become the VF device
    OperationStatus == Energy::Enums::OperationStatus::ENABLED_VF_ON;

    while (ActiveVF) {
        // Here is where device monitoring specific to being the active VF Device would occur.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Active VF has become false. Time to close everything out. There should still be a time specified to make the
    // switch.
    std::this_thread::sleep_until(targetTime);
    OperationStatus == Energy::Enums::OperationStatus::ENABLED_ON;

    return;
}

void publisher_main(int domain_id)
{
    // Create the Domain Particimant QOS to set Entity Name
    dds::domain::qos::DomainParticipantQos qos_participant = dds::core::QosProvider::Default().participant_qos();
    rti::core::policy::EntityName entityName("ES-" + DeviceID);
    qos_participant << entityName;

    // Create a DomainParticipant with default Qos
    dds::domain::DomainParticipant participant(domain_id, qos_participant);

    // Create Topics -- and automatically register the types
    dds::topic::Topic<Energy::Ops::Meas_NodePower> TopicMeas_NodePower(participant, "Meas_NodePower");
    dds::topic::Topic<Energy::Ops::Info_Battery> TopicInfo_Battery(participant, "Info_Battery");
    dds::topic::Topic<Energy::Ops::Status_Device> TopicStatus_Device(participant, "Status_Device");
    dds::topic::Topic<Energy::Ops::Control_Device> TopicControl_Device(participant, "Control_Device");
    dds::topic::Topic<Energy::Common::CNTL_Single_float32> TopicControl_Power(participant, "Control_Power");
    dds::topic::Topic<Energy::Common::CNTL_Single_float32> TopicControl_SOC(participant, "Control_SOC");
    dds::topic::Topic<Energy::Common::MMXU_Single_float32> TopicMeas_SOC(participant, "Meas_SOC");
    dds::topic::Topic<Energy::Ops::VF_Device> TopicVF_Device(participant, "VF_Device");
    dds::topic::Topic<Energy::Ops::VF_Device_Active> TopicVF_Device_Active(participant, "VF_Device_Active");

    // Create Publisher
    dds::pub::Publisher publisher(participant);

    // Create DataWriters with Qos
    dds::pub::DataWriter<Energy::Ops::Meas_NodePower> WriterMeas_NodePower(publisher, TopicMeas_NodePower,
        dds::core::QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Measurement"));
    dds::pub::DataWriter<Energy::Common::MMXU_Single_float32> WriterMeas_SOC(publisher, TopicMeas_SOC,
        dds::core::QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Measurement"));
    dds::pub::DataWriter<Energy::Ops::Info_Battery> WriterInfo_Battery(publisher, TopicInfo_Battery,
        dds::core::QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Info"));
    dds::pub::DataWriter<Energy::Ops::Status_Device> WriterStatus_Device(publisher, TopicStatus_Device,
        dds::core::QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Status"));
    dds::pub::DataWriter<Energy::Ops::VF_Device> WriterVF_Device(publisher, TopicVF_Device,
        dds::core::QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Control"));
    // Set the ownership strength of the Control Load to 0 to start with
    dds::pub::qos::DataWriterQos qos_control_load = dds::core::QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Control");
    qos_control_load << dds::core::policy::OwnershipStrength(0);
    dds::pub::DataWriter<Energy::Common::CNTL_Single_float32> WriterControl_Power(publisher, TopicControl_Power, qos_control_load);

    // Create Subscriber
    dds::sub::Subscriber subscriber(participant);

    // Create DataReaders with Qos
    dds::sub::DataReader<Energy::Ops::Control_Device> ReaderControl_Device(subscriber, TopicControl_Device,
        dds::core::QosProvider::Default().datareader_qos("EnergyCommsLibrary::Control"));
    dds::sub::DataReader<Energy::Common::CNTL_Single_float32> ReaderControl_Power(subscriber, TopicControl_Power,
        dds::core::QosProvider::Default().datareader_qos("EnergyCommsLibrary::Control"));
    dds::sub::DataReader<Energy::Common::CNTL_Single_float32> ReaderControl_SOC(subscriber, TopicControl_SOC,
        dds::core::QosProvider::Default().datareader_qos("EnergyCommsLibrary::Control"));
    dds::sub::DataReader<Energy::Ops::VF_Device_Active> ReaderVF_Device_Active(subscriber, TopicVF_Device_Active,
        dds::core::QosProvider::Default().datareader_qos("EnergyCommsLibrary::Control"));

    /* Create Query Conditions */
    // Create query parameters
    std::vector<std::string> query_parameters = { "'" + DeviceID + "'" };
    dds::sub::status::DataState commonDataState = dds::sub::status::DataState(
        dds::sub::status::SampleState::not_read(),
        dds::sub::status::ViewState::any(),
        dds::sub::status::InstanceState::alive());
    // Query Condition for Controlling the device. This is basic functionality for a grid connected device.
    dds::sub::cond::QueryCondition QueryConditionControl_Device(
        dds::sub::Query(ReaderControl_Device, "Device MATCH %0", query_parameters),
        commonDataState,
        [&ReaderControl_Device](dds::core::cond::Condition condition) {
            auto condition_as_qc = dds::core::polymorphic_cast<dds::sub::cond::QueryCondition>(condition);
            auto samples = ReaderControl_Device.select().condition(condition_as_qc).read();
            for (auto sample : samples)
            {
                // All valid samples will be processed and execute the following function
                if (sample.info().valid())
                    InterconnectControl(sample.data().Command());
            }
        }
    );
    // Query Condition for power setting. This is basic functionality for a ES system
    dds::sub::cond::QueryCondition QueryConditionControl_Power(
        dds::sub::Query(ReaderControl_Power, "Device MATCH %0", query_parameters),
        commonDataState,
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
    // Query Condition for Setting SOC. This is used for simulation only
    dds::sub::cond::QueryCondition QueryConditionControl_SOC(
        dds::sub::Query(ReaderControl_SOC, "Device MATCH %0", query_parameters),
        commonDataState,
        [&ReaderControl_SOC](dds::core::cond::Condition condition) {
            auto condition_as_qc = dds::core::polymorphic_cast<dds::sub::cond::QueryCondition>(condition);
            auto samples = ReaderControl_SOC.select().condition(condition_as_qc).read();
            for (auto sample : samples)
            {
                // All valid samples will be processed and set the global variable
                if (sample.info().valid())
                    SimSOC = sample.data().SetPoint();
            }
        }
    );

    /* Create Read Conditions */
    // We are using a read condition for the VF_Device_Active because we have different behavior based on whether or
    // not the device is becoming a VF_Device or passing off being a VF device.
    dds::sub::cond::ReadCondition ReadConditionVF_Device_Active(
        ReaderVF_Device_Active, commonDataState, [&ReaderVF_Device_Active, &WriterStatus_Device]() {
            auto samples = ReaderVF_Device_Active.take();
            for (auto sample : samples) {
                if (sample.info().valid()) {
                    if (DeviceID == sample.data().Device()) {
                        ActiveVF = true;
                        std::thread(VFDeviceActivity, sample.data().SwitchTime()).detach();
                    }
                    else {
                        SwitchTime = sample.data().SwitchTime();
                        ActiveVF = false;
                    }
                }
            }
        }
    );
    // Create Sample objects for datawriters (except for Meas_NodePower, which is handled in another thread)
    Energy::Ops::Info_Battery sampleInfo_Battery(DeviceID, NodeID, MaxLoad, MaxGeneration, Capacity);
    Energy::Common::CNTL_Single_float32 sampleControl_Power(DeviceID, DeviceID, -5.0);

    // Write Info, Status, and Load Control. Each of these topics should only change due to exception.
    WriterInfo_Battery.write(sampleInfo_Battery);
    WriterControl_Power.write(sampleControl_Power);

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
        [&ReaderControl_Device](dds::core::cond::Condition condition) {
            dds::core::status::StatusMask status_mask = ReaderControl_Device.status_changes();
            if ((status_mask & dds::core::status::StatusMask::liveliness_changed()).any()) {
                auto condition_as_qc = dds::core::polymorphic_cast<dds::sub::cond::QueryCondition>(condition);
                auto samples = ReaderControl_Device.select().condition(condition_as_qc).read();
                for (auto sample : samples)
                {
                    // All valid samples will be processed and execute the following function
                    if (sample.info().valid())
                        InterconnectControl(sample.data().Command());
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

    // Launch thread for continuous node measurement writes, status updates, and VF Device Writes
    std::thread threadMeas(&ContinuousWriter, WriterMeas_NodePower, WriterMeas_SOC);
    std::thread threadStatus(&StatusMonitor, WriterStatus_Device);
    std::thread threadVF(&ContinuousVFStrength, WriterVF_Device);

    // Set up the Waitset
    dds::core::cond::WaitSet waitset;
    waitset += QueryConditionControl_Device;
    waitset += QueryConditionControl_Power;
    waitset += QueryConditionControl_SOC;
    waitset += ReadConditionVF_Device_Active;
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
