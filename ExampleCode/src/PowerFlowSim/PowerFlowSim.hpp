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

#ifndef POWER_FLOW_SIM_H
#define POWER_FLOW_SIM_H

#include <atomic>

#include "../common/ConnextEnergy.hpp"
#include "../generated/EnergyComms.hpp"
#include "../../../submodules/inih/INIReader.h"

using namespace std;

class PowerFlowSim : ConnextEnergy {
public:
    PowerFlowSim(const int domainId, const std::string& entityName, const INIReader& config);

    void StopSim();
    void ExecuteSim();

    // Getter for process active status
    const bool RunProcesses() const;

protected:
    // Getters for static members
    const std::string& DeviceID() const;
    const std::string& OptimizerID() const;
    const std::string& InterconnectID() const;
    const chrono::milliseconds& SimThreadWait() const;

    // Microgrid State change access functions
    const Energy::Enums::MicrogridStatus CurrentStatus() const;
    void CurrentStatus(Energy::Enums::MicrogridStatus status);

    // Internal functions
    virtual void SimThread();
    virtual void BalanceConnected();
    virtual void BalanceIsland();

private:
    // Global used to stop all threads
    atomic<bool> runProcesses_;

    // Global used to modify code execution during microgrid state change
    atomic<Energy::Enums::MicrogridStatus> currentStatus_;

    // Static values from config
    std::string deviceID_;
    std::string optimizerID_;
    std::string interconnectID_;
    chrono::milliseconds simThreadWait_;
};

#endif