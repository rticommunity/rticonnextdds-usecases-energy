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

#ifndef ENERGY_STORAGE_H
#define ENERGY_STORAGE_H

#include "../common/IED.hpp"
#include "../generated/EnergyComms.hpp"

class EnergyStorage : public IED {
public:
    EnergyStorage(const int domainId, const std::string& entityName, const INIReader& config);

    void Execute() override;
    void ContinuousWriter();
    void ContinuousVFStrength() override;
    void SetInfo() override;
    
protected:
    float SimMeasurement() override; // overloaded from IED class

    // Getters and Setters for dynamic members
    float SimSOC() const;
    void SimSOC(const float& soc);

    // Getters for static members
    float Capacity() const;

private:
    float simSOC_;
    float capacity_;

    // Setters for static members
    void Capacity(const float& kWh);
};

#endif