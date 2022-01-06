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

#include "IED.hpp"
#include <thread>
#include <random>

IED::IED(const int domainId, const std::string& entityName, const INIReader& config) : 
    ConnextEnergy(domainId, entityName)
{
    runProcesses_ = true;

    // Default initialization of class members
    SimMeasurement(0.0);
    ActiveVf(false);
    ConnectionStatus(Energy::Enums::ConnectionStatus::DISCONNECTED);
    OperationStatus(Energy::Enums::OperationStatus::DISABLED_OFF);
    waitset_ = dds::core::cond::WaitSet();
    // Values from configuration file
    VfDevice(config.GetBoolean("IED", "VFDevice", false));
    DeviceID(config.Get("IED", "DeviceID", "SampleDevice"));
    NodeID(config.Get("IED", "NodeID", "000"));
    MaxLoad(config.GetFloat("IED", "MaxLoad", -0.0));
    MaxGeneration(config.GetFloat("IED", "MaxGeneration", 0.0));
    MaxTimeToWait(config.GetInteger("IED", "MaxTimeToWait", 0));

    // Initialize IED DataWriters and IED DataReaders
    ReaderControl_Device();
    ReaderControl_Power();
    ReaderVF_Device_Active();
    WriterControl_Power();
    WriterStatus_Device();
    WriterMeas_NodePower();
    WriterVF_Device();
    WriterInfo_Resource();
}

void IED::StopIED()
{
    runProcesses_ = false;
}

void IED::Execute()
{
    // Create thread vector to allow for cleanup
    vector<std::thread> threadVector;

    /* Create Query Conditions */
    // Create query parameters
    using dds::sub::Query;
    using dds::core::cond::Condition;
    using dds::sub::cond::QueryCondition;
    using dds::core::polymorphic_cast;
    std::vector<std::string> query_parameters = { "'" + DeviceID() + "'" };
    auto commonDataState = dds::sub::status::DataState(
        dds::sub::status::SampleState::not_read(),
        dds::sub::status::ViewState::any(),
        dds::sub::status::InstanceState::alive());
    
    // Query Condition for Controlling the device. This is basic functionality
    // for a grid connected device.
    QueryCondition QueryConditionControl_Device(
        Query(ReaderControl_Device(), "Device MATCH %0", query_parameters), commonDataState, [this](Condition condition) {
            auto&& condition_as_qc = polymorphic_cast<QueryCondition>(condition);
            auto&& samples = this->ReaderControl_Device().select().condition(condition_as_qc).read();
            for (auto&& sample : samples) {
                // All valid samples will be processed
                if (sample.info().valid())
                    // This will call the most derived function in preference to the base function
                    InterconnectControl(sample.data().Command());
            }
        });
    AttachCondition(QueryConditionControl_Device);

    // Query Condition for power setting. This is basic functionality for an IED
    QueryCondition QueryConditionControl_Power(
        Query(ReaderControl_Power(), "Device MATCH %0", query_parameters), commonDataState, [this](Condition condition) {
            auto&& condition_as_qc = polymorphic_cast<QueryCondition>(condition);
            auto&& samples = this->ReaderControl_Power().select().condition(condition_as_qc).read();
            for (auto&& sample : samples) {
                // All valid samples will be processed and set the global
                // variable
                if (sample.info().valid())
                    // This will call the most derived function in preference to the base function
                    this->SimMeasurement(sample.data().SetPoint());
            }
        });
    AttachCondition(QueryConditionControl_Power);

    /* Create Read Conditions */
    using dds::sub::cond::ReadCondition;
    /* We are using a read condition for the VF_Device_Active because we have
    * different behavior based on whether or not the device is becoming a
    * VF_Device or passing off being a VF device. Only applicable for IEDs that
    * have the capability of providing VF support.
    */
    if (VfDevice()) {
        ReadCondition ReadConditionVF_Device_Active(ReaderVF_Device_Active(), commonDataState, [this]() {
            auto samples = ReaderVF_Device_Active().take();
            for (auto sample : samples) {
                if (sample.info().valid()) {
                    /* The decision on a device becoming a VF Device must be
                    * defined within the system. Within this system, it is done
                    * external to the device taking over the behavior.
                    */
                    if (DeviceID() == sample.data().Device()) {
                        ActiveVf(true);
                        std::thread(&IED::VFDeviceActivity, this, sample.data().SwitchTime()).detach();
                    }
                    /* This is the method of allowing this device to gracefully
                    * relinquish VF control in the case in which the microgrid
                    * is resynchronizing or VF control is being passed between
                    * devices.
                    */
                    else {
                        switchTime_ = sample.data().SwitchTime();
                        ActiveVf(false);
                    }
                }
            }
        });
        AttachCondition(ReadConditionVF_Device_Active);
        // This thread only needs to be active if the IED is a VF Device
        threadVector.push_back(std::thread(&IED::ContinuousVFStrength, this));
    }
    
    this->SetInfo();

    Energy::Common::CNTL_Single_float32 sampleControl_Power(DeviceID(), DeviceID(), 0.0f);
    WriterControl_Power().write(sampleControl_Power);

    // Launch thread for continuous node measurement writes, status updates, and
    // VF Device Writes
    
    threadVector.push_back(std::thread(&IED::ContinuousWriter, this));
    threadVector.push_back(std::thread(&IED::StatusMonitor, this));

    // Here we are handling our waitset and reactions to inputs
    while (RunProcesses()) {
        // Dispatch will call the handlers associated to the WaitSet conditions
        // when they activate
        waitset_.dispatch(dds::core::Duration(4));  // Wait up to 4s each time
    }

    // After RunProcess returns false all other threads should also exit
    for (auto&& th : threadVector)
        th.join();
}

/* SimMeasurement
 * In this example we have a single measurement. This could be the case when the
 * load is on a single phase or if the only thing that needs to be returned (or
 * is available) is the aggregate. This, along with the data model, would need
 * to be changed to pass information on a 3-phase system.
 */
float IED::SimMeasurement()
{
    using namespace Energy::Enums;

    // An IED that is not connected provides no load or generation
    if (ConnectionStatus() == ConnectionStatus::CONNECTED &&
        (OperationStatus() == OperationStatus::ENABLED_ON
            || OperationStatus() == OperationStatus::ENABLED_VF_ON))
        return simMeasurement_;
    else
        return 0.0;
}

// Returns the Device ID associated with the IED. This is determined in configuration and should be
// static.
const string& IED::DeviceID() const
{
    return deviceID_;
}

// Answers the question as to whether or not the device is capable of providing voltage and
// frequency support in the event the microgrid islands.
const bool IED::VfDevice() const
{
    return vfDevice_;
}

// The Node identifier of the electrical connection of the IED to the microgrid.
const string& IED::NodeID() const
{
    return nodeID_;
}

// Maximum load that the IED can provide to the grid.
const float& IED::MaxLoad() const
{
    return maxLoad_;
}

// Maximum power generation that the IED can provide to the grid.
const float& IED::MaxGeneration() const
{
    return maxGeneration_;
}

// The maximum time the IED will wait to transition to VF support.
const chrono::seconds& IED::MaxTimeToWait() const
{
    return maxTimeToWait_;
}

// Update of the current simulated measurement value
void IED::SimMeasurement(const float& kW)
{
    // Keep measurement between Max Generation and Max Load
    if (kW > MaxGeneration())
        simMeasurement_ = MaxGeneration();
    else if (kW < MaxLoad())
        simMeasurement_ = MaxLoad();
    else
        simMeasurement_ = kW;
}

// Whether or not the IED is actively supporting voltage and frequency on an islanded
// microgrid.
bool IED::ActiveVf() const
{
    return activeVF_;
}

// Changes the behavior of the IED when the grid islands or resynchronizes to the
// grid.
void IED::ActiveVf(bool isActiveVf)
{
    activeVF_ = isActiveVf;
}

// The current connection status of the IED to the microgrid
Energy::Enums::ConnectionStatus IED::ConnectionStatus() const
{
    return connectionStatus_;
}

// Update the connection status of the IED to the microgrid
void IED::ConnectionStatus(Energy::Enums::ConnectionStatus newStatus)
{
    connectionStatus_ = newStatus;
}

// Update the operation status of the IED
void IED::OperationStatus(Energy::Enums::OperationStatus newStatus)
{
    operationStatus_ = newStatus;
}

// The current operation status of the IED
Energy::Enums::OperationStatus IED::OperationStatus() const
{
    return operationStatus_;
}

// The Device ID associated with the IED.
void IED::DeviceID(const string& id)
{
    deviceID_ = id;
}

// Answers the question as to whether or not the device is capable of providing voltage and
// frequency support in the event the microgrid islands.
void IED::VfDevice(bool isVfDevice)
{
    vfDevice_ = isVfDevice;
}

// The Node identifier of the electrical connection of the IED to the microgrid.
void IED::NodeID(const string& id)
{
    nodeID_ = id;
}

// Maximum load that the IED can provide to the grid.
void IED::MaxLoad(const float& kW)
{
    // Load must be negative
    maxLoad_ = -abs(kW);
}

// Maximum power generation that the IED can provide to the grid.
void IED::MaxGeneration(const float& kW)
{
    // Generation must be positive
    maxGeneration_ = abs(kW);
}

// Sets the maximum time the IED will wait to transition to VF support.
void IED::MaxTimeToWait(const int seconds)
{
    maxTimeToWait_ = chrono::seconds(seconds);
}

// Performs a short delay and returns how long the delay was
std::chrono::milliseconds IED::HardwareAccessDelay(const int minMs, const int maxMs) const
{
    // Get a function that provides a random distribution between the min and max values
    default_random_engine generator;
    uniform_int_distribution<int> distribution(minMs, maxMs);

    std::chrono::milliseconds timespan(distribution(generator));
    std::this_thread::sleep_for(timespan);

    return timespan;
}

const bool IED::RunProcesses() const
{
    return runProcesses_.load();
}

/* StatusMonitor
 * In this example we are watching for the internal status to change, and when
 * it does to publish a new status.
 */
void IED::StatusMonitor()
{
    Energy::Ops::Status_Device sample(DeviceID(), ConnectionStatus(), OperationStatus());

    auto&& writer = WriterStatus_Device();

    // Perform initial status write
    writer.write(sample);

    while (RunProcesses()) {
        // When there is a change to the global variables, send out a new sample
        if (sample.ConnectionStatus() != ConnectionStatus()
            || sample.OperationStatus() != OperationStatus())
        {
            sample.ConnectionStatus(ConnectionStatus());
            sample.OperationStatus(OperationStatus());
            writer.write(sample);
        }
        // When no change has occured, sleep for 100 ms
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


/* InterconnectControl
 * In this example we are responding to a command to connect or disconnect and
 * changing the appropriate status.
 */
void IED::InterconnectControl(Energy::Enums::DeviceControl command)
{
    // Here is where code would go to interface with the actual relay connecting
    // the device to the grid. Based on its response, the corresponding status
    // would be updated. If this is a lengthy process, then a thread should
    // probably be spawned that would allow status updates to be sent out while
    // letting the device process other incoming messages.

    using namespace Energy::Enums;

    switch (command) {
    case DeviceControl::CONNECT:
        ConnectionStatus(ConnectionStatus::CONNECTED);
        OperationStatus(OperationStatus::ENABLED_ON);
        break;
    case DeviceControl::DISCONNECT:
        ConnectionStatus(ConnectionStatus::DISCONNECTED);
        OperationStatus(OperationStatus::DISABLED_READY);
        break;
    default:
        break;
    }
}

/* ContinuousWriter
 * In this example we are using a function in a seperate thread to continuously
 * publish measurement data. Depending on whether or not other interfaces are
 * thread safe additional semaphores or locks would need to be introduced when
 * accessing outside interfaces between multiple threads. We are not doing that
 * here because the data being published is simulated.
 */
void IED::ContinuousWriter()
{
    using namespace Energy::Ops;

    Meas_NodePower sampleMeas_NodePower(DeviceID(), SimMeasurement(), NodeID());
    
    while (RunProcesses()) {
        // Some sort of communication to the actual system would be here. In
        // our case we're just going to pull from the simulated measurement
        // variable
        sampleMeas_NodePower.Value(SimMeasurement());

        // We are adding a delay here to simulate the actual fetch of information
        // from the system
        IED::HardwareAccessDelay(99, 101);

        // Write the measurement data
        WriterMeas_NodePower().write(sampleMeas_NodePower);
    }
}

/* ContinuousVFStrength
 * This helps support VF device switching by having every device provide a
 * relative strength as a VF device. The actual math used in a given system will
 * probably vary, but something like this should be appropriate. The goal of
 * this particular math is to force the generator to take over at some point
 * when SOC is below 20% and for the battery to take over when SOC goes above
 * 80%.
 * 
 * For the generic case, we want the strength to be 0. This function should be
 * overridden in device classes EnergyStorage and Generator.
 */
void IED::ContinuousVFStrength()
{
    using namespace Energy::Ops;
    using namespace Energy::Enums;

    VF_Device dev(DeviceID());
    int32_t str = 0;
    dds::pub::qos::DataWriterQos QosVF_Device = WriterVF_Device().qos();

    QosVF_Device << dds::core::policy::OwnershipStrength(str);
    std::cout << "Strength = " << str << "." << std::endl;
    WriterVF_Device().qos(QosVF_Device);
    WriterVF_Device().write(dev);
}

/* VFDeviceActivity
* Devices behave in a much more active manner when providing voltage and 
* frequency support to a system. For a simulated device this isn't all that
* difficult, but a real device may need more monitoring and require different
* behavior from its controller. This function is a fill-in for that control.
* 
* This provides base functionality, but would proabably be overridden 
* completely in a real system. This function would be a good template though.
*/
void IED::VFDeviceActivity(Energy::Common::Timestamp ts)
{
    using namespace std::chrono;
    using namespace Energy::Enums;

    std::chrono::time_point<high_resolution_clock> targetTime(
        seconds(ts.Seconds()) + nanoseconds(ts.Fraction()));

    // Check to make sure that something isn't wrong and the scheduled time to
    // wait to transition is greater that the configured max time to wait. For
    // an actual application this would need some kind of status feedback for
    // safety.
    if (duration_cast<duration<float>>(targetTime - high_resolution_clock::now())
        > MaxTimeToWait()) {
        std::cerr << "Time to switch to VF greater than Max Allowed Time.\n";
        return;
    }

    // Start Up Device first
    this->InterconnectControl(DeviceControl::CONNECT);

    // Return VF Ready status to system
    OperationStatus(OperationStatus::ENABLED_VF_READY);

    std::this_thread::sleep_until(targetTime);

    // At this point the device has become the VF device
    OperationStatus(OperationStatus::ENABLED_VF_ON);

    while (ActiveVf()) {
        // Here is where device monitoring specific to being the active VF
        // Device would occur.
        std::this_thread::sleep_for(milliseconds(100));
    }

    // Pull information from the global at once
    auto switchTime = switchTime_.load();

    // This variable should have been updated when ActiveVf was made false
    targetTime = std::chrono::time_point<high_resolution_clock>(
        seconds(switchTime.Seconds()) + nanoseconds(switchTime.Fraction()));

    // Active VF has become false. Time to close everything out. There should
    // still be a time specified to make the switch.
    std::this_thread::sleep_until(targetTime);
    OperationStatus(OperationStatus::ENABLED_ON);

    return;
}

/* SetInfo
* All generic information for the IED is set here. This is different from 
* status data in that it is intrinsic to the device and generally does not
* change.
* 
* EnergyStorage and Generator Info will override this function without calling
* the base function.
*/
void IED::SetInfo()
{
    using namespace Energy::Ops;

    // Create Sample objects for datawriter(s)
    Info_Resource sampleInfo_Resource(
        DeviceID(),
        NodeID(),
        MaxLoad(),
        MaxGeneration());

    // Write Info. the Info QoS will keep this information available to late
    // joiners
    WriterInfo_Resource().write(sampleInfo_Resource);
}


