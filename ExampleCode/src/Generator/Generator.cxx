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

#include "Generator.hpp"

using namespace dds::core;
using namespace dds::topic;
using namespace dds::pub;
using namespace dds::sub;

/* Generator
* This inherits from IED and adds a few functions that differentiate Generator
* systems from generic IEDs
*/
Generator::Generator(const int domainId, const std::string& entityName, const INIReader& config) :
    IED(domainId, entityName, Energy::Enums::DeviceType::GENERATOR, config)
{
    //Pull Configuration
    RampUpTime(config.GetInteger("Generator", "RampUpTime", 10));

    //Run Configuration
    InitializeEfficiencyCurve();

    //Iniitalize Datawriters and Datareaders
    WriterInfo_Generator();
}

/* InitializeEfficiencyCureve
 * We are puting a simple efficiency curve into the example. This would probably
 * have more points and could even change based on conditions.
 */
void Generator::InitializeEfficiencyCurve()
{
    using rti::core::bounded_sequence;
    using Energy::Common::EfficiencyPoint;

    efficiencyCurve_ = new bounded_sequence<EfficiencyPoint, 1024> {
        EfficiencyPoint(0.0f, 0.00f),
        EfficiencyPoint(1.0f, 0.30f),
        EfficiencyPoint(2.0f, 0.40f),
        EfficiencyPoint(3.0f, 0.52f),
        EfficiencyPoint(4.0f, 0.65f),
        EfficiencyPoint(5.0f, 0.82f),
        EfficiencyPoint(6.0f, 0.88f),
        EfficiencyPoint(7.0f, 0.79f),
        EfficiencyPoint(8.0f, 0.55f)
    };

    efficiencyCurve_->resize(9);
}

/* InterconnectControl
 * In this example we are responding to a command to connect or disconnect and
 * changing the appropriate status.
 * 
 * This is overriding the default IED behavior to add in a pause for ramp up.
 */
void Generator::InterconnectControl(Energy::Enums::DeviceControl command)
{
    // Here is where code would go to interface with the actual relay connecting
    // the device to the grid. Based on its response, the corresponding status
    // would be updated. If this is a lengthy process, then a thread should
    // probably be spawned that would allow status updates to be sent out while
    // letting the device process other incoming messages.

    using namespace Energy::Enums;

    switch (command) {
    case DeviceControl::CONNECT:
        OperationStatus(OperationStatus::ENABLED_STARTING);
        // wait to simulate generator ramp-up
        std::this_thread::sleep_for(RampUpTime());
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

/* ContinuousVFStrength
 * This helps support VF device switching by having every device provide a
 * relative strength as a VF device. The actual math used in a given system will
 * probably vary, but something like this should be appropriate. The goal of
 * this particular math is to force the generator to take over at some point
 * when SOC is below 20% and for the battery to take over when SOC goes above
 * 80%.
 *
 * This is incredibly simple for the sample Generator. In reality strength could
 * also include effects such as time of use or time of day restrictions.
 */
void Generator::ContinuousVFStrength()
{
    using namespace Energy::Ops;
    using namespace Energy::Enums;

    VF_Device dev(DeviceID());
    int32_t str = 0;
    dds::pub::qos::DataWriterQos QosVF_Device = WriterVF_Device().qos();

    while (RunProcesses()) {
        if (ActiveVf())
            // This keeps the generator at a higher strength during VF mode
            str = (int32_t)(80 * MaxGeneration() * 100);
        else
            str = (int32_t)(20 * MaxGeneration());

        QosVF_Device << dds::core::policy::OwnershipStrength(str);
        std::cout << "Strength = " << str << "\n";
        WriterVF_Device().qos(QosVF_Device);
        WriterVF_Device().write(dev);

        // We are adding a delay here of 500ms. In reality this is probably
        // overkill
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

/* SetInfo
* Information setting function specific to Genaerators rather than the generic
* IED. This overrides the base function.
*/
void Generator::SetInfo()
{
    using namespace Energy::Ops;

    // Create Sample objects for info datawriter(s)
    Info_Generator sampleInfo_Generator(
        DeviceID(),
        NodeID(),
        MaxLoad(),
        MaxGeneration(),
        Energy::Enums::DeviceType::GENERATOR,
        EfficiencyCurve(),
        std::chrono::duration_cast<std::chrono::duration<uint32_t>>(
            RampUpTime())
        .count());

    // Write Info. the Info QoS will keep this information available to late
    // joiners
    WriterInfo_Generator().write(sampleInfo_Generator);

    ::IED::SetInfo();
}

const chrono::seconds& Generator::RampUpTime() const
{
    return rampUpTime_;
}

void Generator::RampUpTime(const int seconds)
{
    rampUpTime_ = chrono::seconds(seconds);
}

const rti::core::bounded_sequence<Energy::Common::EfficiencyPoint, 1024>& Generator::EfficiencyCurve() const
{
    return *efficiencyCurve_;
}