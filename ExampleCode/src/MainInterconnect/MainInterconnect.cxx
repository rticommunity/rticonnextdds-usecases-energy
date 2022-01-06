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

#include "../generated/EnergyComms.hpp"
#include "MainInterconnect.hpp"

MainInterconnect::MainInterconnect(const int domainId, const std::string& entityName, const INIReader& config)
    : IED(domainId, entityName, config)
{
    NodeIDGrid(config.Get("MainInterconnect", "NodeIDGrid", "000"));
}

/* Execute
* an override for the IED::Execute. Since we want to inherit the behavior of 
* the base function we make sure and call it as well.
*/
void MainInterconnect::Execute()
{
    // The IED class handles writing Node Measurement, this is for writing out
    // the grid node measurement
    std::thread threadMeas(&MainInterconnect::ContinuousWriter, this);

    // This needs to be done at the end, as it handles the waitset dispatch
    IED::Execute();

    threadMeas.join();
}

/* ContinuousWriter
 * In this example we are using a function in a separate thread to continuously
 * publish measurement data. Depending on whether or not other interfaces are
 * thread safe additional semaphores or locks would need to be introduced when
 * accessing outside interfaces between multiple threads. We are not doing that
 * here because the data being published is simulated.
 * 
 * The base IED class handles writing the measurement data for the Microgrid
 * side. Given that we are only returning power flow and not voltage or 
 * frequency, the value at the grid and microgrid side should always be the 
 * same. If voltage measurements were added to the simulation, additional logic
 * would need to be added.
 */
void MainInterconnect::ContinuousWriter()
{
    using namespace Energy::Ops;

    Meas_NodePower sampleMeas_NodePower(DeviceID(), SimMeasurement(), NodeIDGrid());

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

const std::string& MainInterconnect::NodeIDGrid() const
{
    return nodeIDGrid_;
}

void MainInterconnect::NodeIDGrid(const std::string& id)
{
    nodeIDGrid_ = id;
}
