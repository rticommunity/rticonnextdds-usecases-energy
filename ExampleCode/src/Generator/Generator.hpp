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

#ifndef GENERATOR_H
#define GENERATOR_H

#include "../common/IED.hpp"
#include "../generated/EnergyComms.hpp"

class Generator : public IED {
public:
    Generator(const int domainId, const std::string& entityName, const INIReader& config);

    // Override of base function to add RampUpTime
    void InterconnectControl(Energy::Enums::DeviceControl command) override;
    void ContinuousVFStrength() override;
    void SetInfo() override;

protected:
    // Getters for static members
    const chrono::seconds& RampUpTime() const;
    const rti::core::bounded_sequence<Energy::Common::EfficiencyPoint, 1024>& EfficiencyCurve() const;

private:
    chrono::seconds rampUpTime_;
    rti::core::bounded_sequence<Energy::Common::EfficiencyPoint, 1024>* efficiencyCurve_;

    // Setters for constant private memebers
    void RampUpTime(const int seconds);
    void InitializeEfficiencyCurve();
};

#endif