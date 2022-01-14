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

#include <atomic>
#include <mutex>

#include "../common/ConnextEnergy.hpp"
#include "../../../submodules/inih/INIReader.h"

class IED : public ConnextEnergy {
public:
    IED(const int domainId, 
        const std::string& entityName, 
        const Energy::Enums::DeviceType type, 
        const INIReader& config);

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
    const Energy::Enums::DeviceType DeviceType() const;
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
    // Dynamic private members (potentially shared between threads)
    std::atomic<bool> runProcesses_;
    std::atomic<float> simMeasurement_;
    Energy::Common::Timestamp switchTime_;
    std::mutex switchTime_mutex_;
    std::atomic<Energy::Enums::ConnectionStatus> connectionStatus_;
    std::atomic<Energy::Enums::OperationStatus> operationStatus_;

    // Static private members (set once during initial config)
    string deviceID_;
    bool vfDevice_;
    bool activeVF_;
    string nodeID_;
    float maxLoad_;
    float maxGeneration_;
    Energy::Enums::DeviceType deviceType_;
    chrono::seconds maxTimeToWait_;
    dds::core::cond::WaitSet waitset_;

    // Setters for constant private memebers
    void DeviceID(const string& id);
    void DeviceType(const Energy::Enums::DeviceType type);
    void VfDevice(bool isVfDevice);
    void NodeID(const string& id);
    void MaxLoad(const float& kW);
    void MaxGeneration(const float& kW);
    void MaxTimeToWait(const int seconds);
};

#endif