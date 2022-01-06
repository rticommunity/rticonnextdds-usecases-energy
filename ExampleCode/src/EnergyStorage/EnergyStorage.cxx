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

/* EnergyStorage.cxx

A simulated Energy Storage Device

This file can be used as a starting point for any application interfacing a
Energy Storage system to a microgrid, either as a replacement or through
inheritance.
*/

#include <chrono>
#include <thread>

#include <dds/dds.hpp>

#include "EnergyStorage.hpp"

/* EnergyStorage
* This inherits from IED and adds a few functions that differentiate Energy
* Storage systems from generic IEDs
*/
EnergyStorage::EnergyStorage(const int domainId, const std::string& entityName, const INIReader& config) :
    IED(domainId, entityName, config)
{
    //Pull Configuration
    simSOC_ = config.GetFloat("ES", "SimSOC", 50.0);
    capacity_ = config.GetFloat("ES", "Capacity", 1.0);

    //Iniitalize Datawriters and Datareaders
    WriterInfo_Battery();
    WriterControl_SOC();
    ReaderControl_SOC();
    WriterMeas_SOC();
}

/* Execute
* a virtual override for the IED::Execute. Since we want to inherit the
* behavior of the base function we make sure and call it as well.
*/
void EnergyStorage::Execute()
{
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

    // Query Condition for Setting SOC. This is used for simulation only
    QueryCondition QueryConditionControl_SOC(
        Query(ReaderControl_SOC(), "Device MATCH %0", query_parameters), commonDataState, [this](Condition condition) {
            auto&& condition_as_qc = polymorphic_cast<QueryCondition>(condition);
            auto&& samples = ReaderControl_SOC().select().condition(condition_as_qc).read();
            for (auto sample : samples) {
                // All valid samples will be processed and set the global
                // variable
                if (sample.info().valid())
                    SimSOC(sample.data().SetPoint());
            }
        });
    AttachCondition(QueryConditionControl_SOC);

    // The IED class handles writing Node Measurement, this is for writing out
    // the SOC
    std::thread threadMeas(&EnergyStorage::ContinuousWriter, this);

    // This needs to be done at the end, as it handles the waitset dispatch
    IED::Execute();

    threadMeas.join();
}

/* SimMeasurement
 * In this example we are overloading the base IED SimMeasurement method to add
 * funtionality specific to the SOC being too high or low
 */
float EnergyStorage::SimMeasurement()
{
    using namespace std::chrono;
    using namespace Energy::Enums;

    milliseconds timespan(100ms);
    
    // Call of base function
    auto&& meas = IED::SimMeasurement();

    // A full ES can only generate and an empty ES can only load.
    if (SimSOC() >= 100.0f)
        meas = meas > 0.0f ? meas : 0.0f;
    else if (SimSOC() <= 0.0f)
        meas = meas < 0.0f ? meas : 0.0f;

    // SOC only changes if the IED is connected
    if (ConnectionStatus() == ConnectionStatus::CONNECTED) {
        // We are also going to adjust the SOC based on the measured value and
        // the time span.
        SimSOC(SimSOC() - meas * timespan / (duration_cast<milliseconds>(1h) * Capacity()));
        return meas;
    }
    else
        return 0.0;
}

// State of charge of the system, as a percentage between 0.0-100.0.
float EnergyStorage::SimSOC() const
{
    return simSOC_;
}

// State of charge of the system, as a percentage between 0.0-100.0.
void EnergyStorage::SimSOC(const float& soc)
{
    // Keep the stored value in range
    if (soc < 0.0)
        simSOC_ = 0.0;
    else if (soc > 100.0)
        simSOC_ = 100.0;
    else
        simSOC_ = soc;
}

// The energy capacity of the unit, in kWh
float EnergyStorage::Capacity() const
{
    return capacity_;
}

// The energy capacity of the unit, in kWh
void EnergyStorage::Capacity(const float& kWh)
{
    // The math requires this to be a positive value
    capacity_ = abs(kWh);
}

/* ContinuousWriter
 * In this example we are using a function in a seperate thread to continuously
 * publish measurement data. Depending on whether or not other interfaces are
 * thread safe additional semaphores or locks would need to be introduced when
 * accessing outside interfaces between multiple threads. We are not doing that
 * here because the data being published is simulated.
 *
 * For this we are doing SOC. Many times SOC is seen within status type topics 
 * or messages. Here we treat it as a measurement.
 */
void EnergyStorage::ContinuousWriter()
{
    using namespace Energy::Common;

    MMXU_Single_float32 sampleMeas_SOC(DeviceID(), SimSOC());

    while (RunProcesses()) {
        // Modify the measurement data to be written here
        sampleMeas_SOC.Value(SimSOC());

        // Write the measurement data
        WriterMeas_SOC().write(sampleMeas_SOC);
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
 * This override completely overrides the base function, changing how it works.
 * Given this, the base function is not called.
 */
void EnergyStorage::ContinuousVFStrength()
{
    using namespace Energy::Ops;
    using namespace Energy::Enums;

    VF_Device dev(DeviceID());
    int32_t str = 0;
    dds::pub::qos::DataWriterQos QosVF_Device = WriterVF_Device().qos();

    while (RunProcesses()) {
        if (SimSOC() <= 20.0)
            str = (int32_t)(SimSOC() * MaxGeneration());
        else if (SimSOC() >= 80.0)
            str = (int32_t)(SimSOC() * MaxGeneration() * 100);
        else
            str = (int32_t)(SimSOC() * MaxGeneration() * 10);

        // Let specified operation status conditions modify strength
        if (OperationStatus() == OperationStatus::DISABLED_ERROR)
            str = 0;
        if (OperationStatus() == OperationStatus::DISABLED_OFF)
            str = 0;

        QosVF_Device << dds::core::policy::OwnershipStrength(str);
        std::cout << "Strength = " << str << "." << std::endl;
        WriterVF_Device().qos(QosVF_Device);
        WriterVF_Device().write(dev);

        // We are adding a delay here of 500 milliseconds. In reality this is
        // probably overkill (meaning this calculation does not need to occur
        // even this often).
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

/* SetInfo
* Information setting function specific to EnergyStorage rather than the 
* generic IED. This overrides the base function without any need to call the
* base function.
*/
void EnergyStorage::SetInfo()
{
    using namespace Energy::Ops;

    // Create Sample objects for info datawriter(s)
    Info_Battery sampleInfo_Battery(
        DeviceID(),
        NodeID(),
        MaxLoad(),
        MaxGeneration(),
        Capacity());

    // Write Info. the Info QoS will keep this information available to late
    // joiners
    WriterInfo_Battery().write(sampleInfo_Battery);
}

