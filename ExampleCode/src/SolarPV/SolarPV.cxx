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

/* SolarPV.cxx

A simulated PV Device

This file can be used as a starting point for any application interfacing a
PV system to a microgrid, either as a replacement or through inheritance.
*/

#include <chrono>
#include <thread>

#include <dds/dds.hpp>

#include "SolarPV.hpp"

/* SolarPV
* Solar devices are almost generic IEDs. The main difference is that their max
* generation is a function of irradiance rather than being relatively constant.
*/
SolarPV::SolarPV(const int domainId, const std::string& entityName, const INIReader& config) :
    IED(domainId, entityName, config)
{
    //Pull Configuration
    SimIrradiance(config.GetFloat("SolarPV", "SimIrradiance", 500.0));

    //Iniitalize Datawriters and Datareaders
    WriterControl_Irradiance();
    ReaderControl_Irradiance();
}

/* Execute
* a virtual override for the IED::Execute. Since we want to inherit the
* behavior of the base function we make sure and call it as well.
*/
void SolarPV::Execute()
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

    // Query Condition for Setting Irradiance. This is used for simulation 
    // only. In real systems the sun and clouds take care of this for us.
    QueryCondition QueryConditionControl_Irradiance(
        Query(ReaderControl_Irradiance(), "Device MATCH %0", query_parameters), commonDataState, [this](Condition condition) {
            auto&& condition_as_qc = polymorphic_cast<QueryCondition>(condition);
            auto&& samples = ReaderControl_Irradiance().select().condition(condition_as_qc).read();
            for (auto sample : samples) {
                // All valid samples will be processed and set the global
                // variable
                if (sample.info().valid())
                    SimIrradiance(sample.data().SetPoint());
            }
        });
    AttachCondition(QueryConditionControl_Irradiance);

    Energy::Common::CNTL_Single_float32 sampleControl_Irradiance(DeviceID(), DeviceID(), SimIrradiance());
    WriterControl_Irradiance().write(sampleControl_Irradiance);

    // This needs to be done at the end, as it handles the waitset dispatch
    IED::Execute();
}

/* GetMeasurement
 * In this example we have a single measurement. This could be the case when the
 * load is on a single phase or if the only thing that needs to be returned (or
 * is available) is the aggregate. This, along with the data model, would need
 * to be changed to pass information on a 3-phase system.
 */
float SolarPV::SimMeasurement()
{
    using namespace Energy::Enums;

    // Call of base function, this is getting the max generation that the
    // system is allowing due to curtailment
    auto&& curtailedValue = IED::SimMeasurement();

    // A disconnected unit provides no power
    if (ConnectionStatus() == ConnectionStatus::CONNECTED)
        // Return the smallest of the curtailed value and the radiant value
        return curtailedValue < RadiantValue() ? curtailedValue : RadiantValue();
    else
        return 0.0;
}

float SolarPV::SimIrradiance() const
{
    return simIrradiance_;
}

void SolarPV::SimIrradiance(const float& irradiance)
{
    simIrradiance_ = irradiance;
    radiantValue_ = MaxGeneration() * irradiance / 1000.0f;
}

const float& SolarPV::RadiantValue() const
{
    return radiantValue_;
}