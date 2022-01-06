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

#ifndef MAIN_INTERCONNECT_H
#define MAIN_INTERCONNECT_H

#include "../common/IED.hpp"
#include "../generated/EnergyComms.hpp"

class MainInterconnect : public IED {
public:
    MainInterconnect(const int domainId, const std::string& entityName, const INIReader& config);
    void ContinuousWriter();
    void Execute() override;

protected:
    // Getters of static members
    const std::string& NodeIDGrid() const;

private:
    std::string nodeIDGrid_;

    // Setters of static members
    void NodeIDGrid(const std::string& id);
};

#endif