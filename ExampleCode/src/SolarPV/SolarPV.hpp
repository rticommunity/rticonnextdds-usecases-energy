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

#ifndef SOLAR_PV_H
#define SOLAR_PV_H

#include "../common/IED.hpp"
#include "../generated/EnergyComms.hpp"

class SolarPV : public IED {
public:
    SolarPV(const int domainId, const std::string& entityName, const INIReader& config);

    //void ();
    void Execute() override;
    
protected:
    float SimMeasurement() override; // overloaded from IED class

    // Getters and Setters for dynamic members
    float SimIrradiance() const;
    void SimIrradiance(const float& irradiance);

private:
    float simIrradiance_;
    float radiantValue_;

    // Radiant value is a combination of Irradiance and MaxGeneration
    const float& RadiantValue() const;
};

#endif