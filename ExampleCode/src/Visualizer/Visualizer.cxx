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

 This file can be used as a starting point for any application handling the
 operation of a microgrid.
 */

#include <chrono>
#include <thread>

#include "Visualizer.hpp"

const std::chrono::duration<float> MinIslandDelay = std::chrono::seconds(5);

/* Controller
 * This is the contstructor for the controller. This sets up communication and gets the class ready
 * to execute threads.
 */
Visualizer::Visualizer(const int domainId, const std::string& entityName, const INIReader& config) :
    ConnextEnergy(domainId, entityName)
{
    runProcesses_ = true;

    // Pull configuration from ini file
    vizID_ = config.Get("Viz", "VizID", "Visualizer");
    optimizerID_ = config.Get("Viz", "OptimizerID", "SampleOpt");
    int32_t strength = config.GetInteger("Viz", "Strength", 100000);

    InitDDS(strength);
}
Visualizer::Visualizer() : ConnextEnergy(0, "Visualizer")
{
    runProcesses_ = true;

    INIReader config("Visualizer.ini");
    // Pull configuration from ini file
    vizID_ = config.Get("Viz", "VizID", "Visualizer");
    optimizerID_ = config.Get("Viz", "OptimizerID", "SampleOpt");
    int32_t strength = config.GetInteger("Viz", "Strength", 100000);

    InitDDS(strength);
}

void Visualizer::InitDDS(int32_t strength)
{
    // Initialize Control DataWriters
    WriterControl_Microgrid();
    WriterControl_Device();
    WriterControl_Irradiance();
    WriterControl_SOC();
    WriterControl_Power();

    // Initialize DataReaders
    ReaderInfo_Battery();
    ReaderInfo_Generator();
    ReaderInfo_Resource();
    ReaderMeas_NodePower();
    ReaderMeas_SOC();
    ReaderStatus_Device();
    ReaderStatus_Microgrid();

    // Set up Control qos (including strength)
    SetDataWriterOwnershipStrength(Energy::application::TOPIC_CONTROL_DEVICE, strength);
    SetDataWriterOwnershipStrength(Energy::application::TOPIC_CONTROL_POWER, strength);
    SetDataWriterOwnershipStrength(Energy::application::TOPIC_CONTROL_MICROGRID, strength);
    SetDataWriterOwnershipStrength(Energy::application::TOPIC_CONTROL_IRRADIANCE, strength);
    SetDataWriterOwnershipStrength(Energy::application::TOPIC_CONTROL_SOC, strength);
}

/* ExecuteViz
* Handles setting up all callbacks for DataReaders and running all Connext 
* pieces
*/
void Visualizer::ExecuteViz()
{
    /* Create Query Conditions */
    using namespace dds::sub::status;
    using ReadCondition = dds::sub::cond::ReadCondition;
    using Condition = dds::core::cond::Condition;
    // Create query parameters
    auto&& commonDataState = DataState(
        SampleState::not_read(),
        ViewState::any(),
        InstanceState::alive());

    // Readcondition vector used to include all readconditions at once
    auto&& readConditionVector = std::vector<ReadCondition>();

    /* Each ReadCondition calls functions initialy defined in this class, but 
    * meant to be overriden in the derived class.
    */
    readConditionVector.push_back(ReadCondition(ReaderInfo_Battery(), commonDataState,
        [this]() { this->Callback_ReaderInfo_Battery(); }));
    
    readConditionVector.push_back(ReadCondition(ReaderInfo_Generator(), commonDataState,
        [this]() { this->Callback_ReaderInfo_Generator(); }));
    
    readConditionVector.push_back(ReadCondition(ReaderMeas_NodePower(), commonDataState,
        [this]() { this->Callback_ReaderMeas_NodePower(); }));
    
    readConditionVector.push_back(ReadCondition(ReaderMeas_SOC(), commonDataState,
        [this]() { this->Callback_ReaderMeas_SOC(); }));

    readConditionVector.push_back(ReadCondition(ReaderStatus_Device(), commonDataState,
        [this]() { this->Callback_ReaderStatus_Device(); }));

    readConditionVector.push_back(ReadCondition(ReaderStatus_Microgrid(), commonDataState,
        [this]() { this->Callback_ReaderStatus_Microgrid(); }));

    // This is the only oddball for now. We want the overriden behavior, but
    // we also have some stuff we need to do in this class
    readConditionVector.push_back(ReadCondition(ReaderInfo_Resource(), commonDataState,
        [this]() { 
            this->Callback_ReaderInfo_Resource(); 
            this->GetIEDs();
        }));

    dds::core::cond::WaitSet waitset;
    // attach all read conditions
    for (auto&& cond : readConditionVector) waitset += cond;

    // Here we are handling our waitset and reactions to inputs
    while (true) {
        // Dispatch will call the handlers associated to the WaitSet conditions
        // when they activate
        waitset.dispatch(dds::core::Duration(4));  // Wait up to 4s each time
    }

    // detach all read conditions and close references
    for (auto&& cond : readConditionVector) {
        waitset -= cond;
        cond.close();
    }
}

/* GetIEDs
* This function monitors the Info_Resource datareader to determine which device
* ID(s) correspond to device types.
* 
* Right now the visualizer only supports a single instance of each type. As this
* changes, this function will need to be updated.
*/
void Visualizer::GetIEDs()
{
    using Energy::Enums::DeviceType;

    for (auto&& sample : ReaderInfo_Resource().read()) {
        if (sample.info().valid()) {
            std::lock_guard<std::mutex> guard(id_mutex_);
            switch (sample.data().DeviceType()) {
            case DeviceType::ENERGY_STORAGE:
                esID_ = sample.data().Device();
                break;
            case DeviceType::GENERATOR:
                genID_ = sample.data().Device();
                break;
            case DeviceType::INTERCONNECT:
                interconnectID_ = sample.data().Device();
                break;
            case DeviceType::LOAD:
                loadID_ = sample.data().Device();
                break;
            case DeviceType::SOLAR_PV:
                solarID_ = sample.data().Device();
                break;
            default:
                break;
            }
        }
    }
}

/* StopViz
* All this does is set the atomic runProcesses_ boolean to false. All threads
* checking the bool should stop
*/
void Visualizer::StopViz()
{
    runProcesses_.store(false);
}

/* RunProcesses
* return the current state of the atomic boolean that keeps all threads running
*/
const bool Visualizer::RunProcesses() const
{
    return runProcesses_.load();
}

const std::string& Visualizer::EnergyStorageID()
{
    std::lock_guard<std::mutex> guard(id_mutex_);
    return esID_;
}

const std::string& Visualizer::LoadID()
{
    std::lock_guard<std::mutex> guard(id_mutex_);
    return loadID_;
}

const std::string& Visualizer::GeneratorID()
{
    std::lock_guard<std::mutex> guard(id_mutex_);
    return genID_;
}

const std::string& Visualizer::SolarID()
{
    std::lock_guard<std::mutex> guard(id_mutex_);
    return solarID_;
}

const std::string& Visualizer::MainInterconnectID()
{
    std::lock_guard<std::mutex> guard(id_mutex_);
    return interconnectID_;
}

const std::string& Visualizer::OptimizerID() const
{
    return optimizerID_;
}

const std::string& Visualizer::VizID() const
{
    return vizID_;
}