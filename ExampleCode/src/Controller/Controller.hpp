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

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../common/ConnextEnergy.hpp"
#include "../generated/EnergyComms.hpp"
#include "../../../submodules/inih/INIReader.h"

class Controller : ConnextEnergy {
public:
    Controller(const int domainId, const std::string& entityName, const INIReader& config);

    void StopController();
    void ExecuteController();

protected:
    void CheckTrip();
    void IslandMicrogrid();
    void IslandOperation(bool Immediate);
    void Optimize();
    void ProcessVizCommand(Energy::Enums::MicrogridStatus command);
    void Resynchronize();
    void UnintentionalIsland();

private:
    bool runProcesses_;

    std::string optimizerID_;
    std::string interconnectID_;
    std::string vizID_;
};

#endif