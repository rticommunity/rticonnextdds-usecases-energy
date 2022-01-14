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

#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <atomic>
#include <mutex>

#include "../common/ConnextEnergy.hpp"
#include "../generated/EnergyComms.hpp"
#include "../../../submodules/inih/INIReader.h"

class Visualizer : protected ConnextEnergy {
public:
    Visualizer(const int domainId, const std::string& entityName, const INIReader& config);
    Visualizer();
    // Call to stop all visualizer threads
    void StopViz();
    void ExecuteViz();

protected:
    // Continuous function that looks at info topics to determine the type and
    // id of all IEDs on the system
    void GetIEDs();
    // Returns true until StopViz is called
    const bool RunProcesses() const;

    // Virtual functions that are called by read conditions attached to each 
    // Connext DataReader. By default, these functions do nothing.
    virtual void Callback_ReaderInfo_Resource() {};
    virtual void Callback_ReaderInfo_Battery() {};
    virtual void Callback_ReaderInfo_Generator() {};
    virtual void Callback_ReaderMeas_NodePower() {};
    virtual void Callback_ReaderMeas_SOC() {};
    virtual void Callback_ReaderStatus_Device() {};
    virtual void Callback_ReaderStatus_Microgrid() {};

    // Accessors for private members
    const std::string& EnergyStorageID();
    const std::string& LoadID();
    const std::string& GeneratorID();
    const std::string& SolarID();
    const std::string& MainInterconnectID();
    const std::string& OptimizerID() const;
    const std::string& VizID() const;

private:
    atomic<bool> runProcesses_;
    // TODO: These should be changed to vectors when updating viz to support
    // multiple IEDs of these types.
    std::string esID_;
    std::string loadID_;
    std::string genID_;
    std::string solarID_;
    std::string interconnectID_;
    // Mutexes for strings
    std::mutex id_mutex_;

    // Static globals set during config and not changed
    std::string optimizerID_;
    std::string vizID_;

    // Initializer used by constructors
    void InitDDS(int32_t strength);
};

#endif