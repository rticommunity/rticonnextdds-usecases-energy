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

 /* Generator.cxx

 A simulated Generation Device

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

#include "../../_Common/EnergyComms.hpp"

float SimMeasurement = 0;
bool connected = true;
bool ActiveVF = false;
Energy::Common::Timestamp SwitchTime;
const std::string DeviceID = "SampleGen";
const std::string NodeID = "004";
const float MaxLoad = 0.0;
const float MaxGeneration = 8.0;
const std::chrono::duration<std::chrono::seconds> RampUpTime = std::chrono::seconds(10);
rti::core::bounded_sequence<Energy::Common::EfficiencyPoint, 1024> EfficiencyCurve;
Energy::Enums::ConnectionStatus ConnectionStatus = Energy::Enums::ConnectionStatus::DISCONNECTED;
Energy::Enums::OperationStatus OperationStatus = Energy::Enums::OperationStatus::DISABLED_OFF;

const std::chrono::duration<std::chrono::seconds> MaxTimeToWait = std::chrono::seconds(300);

/* InitializeEfficiencyCureve
* We are puting a simple efficiency curve into the example. This would probably have more points and could even change
* based on coditions.
*/
void InitializeEfficiencyCurve()
{
    EfficiencyCurve[0] = Energy::Common::EfficiencyPoint(0.0, 0.00);
    EfficiencyCurve[1] = Energy::Common::EfficiencyPoint(1.0, 0.30);
    EfficiencyCurve[2] = Energy::Common::EfficiencyPoint(2.0, 0.40);
    EfficiencyCurve[3] = Energy::Common::EfficiencyPoint(3.0, 0.52);
    EfficiencyCurve[4] = Energy::Common::EfficiencyPoint(4.0, 0.65);
    EfficiencyCurve[5] = Energy::Common::EfficiencyPoint(5.0, 0.82);
    EfficiencyCurve[6] = Energy::Common::EfficiencyPoint(6.0, 0.88);
    EfficiencyCurve[7] = Energy::Common::EfficiencyPoint(7.0, 0.79);
    EfficiencyCurve[8] = Energy::Common::EfficiencyPoint(8.0, 0.55);

    EfficiencyCurve.resize(9);
}

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
        OperationStatus = Energy::Enums::OperationStatus::ENABLED_STARTING;
        // 10 second wait to simulate generator ramp-up
        std::this_thread::sleep_for(RampUpTime);
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

    if (connected)
        return meas;
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

/* ContinuousVFStrength
* This helps support VF device switching by having every device provide a relative strength as a VF device. The actual
* math used in a given system will probably vary, but something like this should be appropriate. The goal of this
* particular math is to force the generator to take over at some point when SOC is below 20% and for the battery to
* take over when SOC goes above 80%.
* 
* This is incredibly simple for the sample Generator. In reality strength could also include effects such as time of
* use or time of day restrictions.
*/
void ContinuousVFStrength(dds::pub::DataWriter<Energy::Ops::VF_Device> WriterVF_Device)
{
    Energy::Ops::VF_Device dev(DeviceID);
    int32_t str;
    dds::pub::qos::DataWriterQos QosVF_Device = WriterVF_Device.qos();

    while (true) {
        if (ActiveVF)
            str = 80 + MaxGeneration * 100; // This keeps the generator at a higher strength
        else
            str = 20 * MaxGeneration;

        QosVF_Device << dds::core::policy::OwnershipStrength(str);
        WriterVF_Device.qos(QosVF_Device);
        WriterVF_Device.write(dev);

        // We are adding a delay here of 1 second. In reality this is probably overkill
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void VFDeviceActivity(Energy::Common::Timestamp ts)
{
    using namespace std::chrono;

    time_point<high_resolution_clock> targetTime(seconds(ts.Seconds()) + nanoseconds(ts.Fraction()));

    // Check to make sure that something isn't wrong and the scheduled time to wait to transition is greater that the
    // configured max time to wait. For an actual application this would need some kind of status feedback for safety.
    if (duration_cast<duration<seconds>>(targetTime - high_resolution_clock::now()) > MaxTimeToWait) {
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
        std::this_thread::sleep_for(milliseconds(100));
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
    rti::core::policy::EntityName entityName("Generator-" + DeviceID);
    qos_participant << entityName;

    // Create a DomainParticipant with default Qos
    dds::domain::DomainParticipant participant(domain_id);

    // Create Topics -- and automatically register the types
    dds::topic::Topic<Energy::Ops::Meas_NodePower> TopicMeas_NodePower(participant, "Meas_NodePower");
    dds::topic::Topic<Energy::Ops::Info_Generator> TopicInfo_Generator(participant, "Info_Generator");
    dds::topic::Topic<Energy::Ops::Status_Device> TopicStatus_Device(participant, "Status_Device");
    dds::topic::Topic<Energy::Ops::Control_Device> TopicControl_Device(participant, "Control_Device");
    dds::topic::Topic<Energy::Common::CNTL_Single_float32> TopicControl_Power(participant, "Control_Power");
    dds::topic::Topic<Energy::Ops::VF_Device> TopicVF_Device(participant, "VF_Device");
    dds::topic::Topic<Energy::Ops::VF_Device_Active> TopicVF_Device_Active(participant, "VF_Device_Active");

    // Create Publisher
    dds::pub::Publisher publisher(participant);

    // Create DataWriters with Qos
    dds::pub::DataWriter<Energy::Ops::Meas_NodePower> WriterMeas_NodePower(publisher, TopicMeas_NodePower,
        dds::core::QosProvider::Default().datawriter_qos("EnergyCommsLibrary::Measurement"));
    dds::pub::DataWriter<Energy::Ops::Info_Generator> WriterInfo_Generator(publisher, TopicInfo_Generator,
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
                    std::thread(InterconnectControl, sample.data().Command()).detach();
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
    InitializeEfficiencyCurve();
    Energy::Ops::Info_Generator sampleInfo_Generator(DeviceID, NodeID, MaxLoad, MaxGeneration, EfficiencyCurve, 
        std::chrono::duration_cast<std::chrono::duration<uint32_t>>(RampUpTime).count());
    Energy::Common::CNTL_Single_float32 sampleControl_Power(DeviceID, DeviceID, 0.0);

    // Write Info, Status, and Load Control. Each of these topics should only change due to exception.
    WriterInfo_Generator.write(sampleInfo_Generator);
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
    std::thread threadStatus(&StatusMonitor, WriterStatus_Device);
    std::thread threadVF(&ContinuousVFStrength, WriterVF_Device);

    // Set up the Waitset
    dds::core::cond::WaitSet waitset;
    waitset += QueryConditionControl_Device;
    waitset += QueryConditionControl_Power;
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