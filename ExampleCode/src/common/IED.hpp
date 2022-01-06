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

#ifndef IED_H
#define IED_H

#include "../common/ConnextEnergy.hpp"
#include "../../../submodules/inih/INIReader.h"

class IED : public ConnextEnergy {
public:
    IED(const int domainId, const std::string& entityName, const INIReader& config);

    void StopIED();
    virtual void Execute();
    virtual void StatusMonitor();
    virtual void InterconnectControl(Energy::Enums::DeviceControl command);
    void ContinuousWriter();
    virtual void ContinuousVFStrength();
    virtual void VFDeviceActivity(Energy::Common::Timestamp ts);
    virtual void SetInfo();

protected:
    // Getters for private members
    const string& DeviceID() const;
    const bool VfDevice() const;
    const string& NodeID() const;
    const float& MaxLoad() const;
    const float& MaxGeneration() const;
    const chrono::seconds& MaxTimeToWait() const;

    // Getters and Setters for dynamic members
    virtual float SimMeasurement();
    virtual void SimMeasurement(const float& kW);
    virtual bool ActiveVf() const;
    virtual void ActiveVf(bool isActiveVf);
    virtual Energy::Enums::ConnectionStatus ConnectionStatus() const;
    virtual void ConnectionStatus(Energy::Enums::ConnectionStatus newStatus);
    virtual Energy::Enums::OperationStatus OperationStatus() const;
    virtual void OperationStatus(Energy::Enums::OperationStatus newStatus);

    /* Utility functions */
    // Performs a short delay and returns how long the delay was
    virtual std::chrono::milliseconds HardwareAccessDelay(int minMs, int maxMs) const;
    const bool RunProcesses() const;
    template<typename T>
    void AttachCondition(T condition) { waitset_.attach_condition(condition); }

private:
    std::atomic<bool> runProcesses_;
    std::atomic<Energy::Common::Timestamp> switchTime_;
    dds::core::cond::WaitSet waitset_;

    string deviceID_;
    float simMeasurement_;
    bool vfDevice_;
    bool activeVF_;
    string nodeID_;
    float maxLoad_;
    float maxGeneration_;
    Energy::Enums::ConnectionStatus connectionStatus_;
    Energy::Enums::OperationStatus operationStatus_;
    chrono::seconds maxTimeToWait_;

    // Setters for constant private memebers
    void DeviceID(const string& id);
    void VfDevice(bool isVfDevice);
    void NodeID(const string& id);
    void MaxLoad(const float& kW);
    void MaxGeneration(const float& kW);
    void MaxTimeToWait(const int seconds);
};


#endif