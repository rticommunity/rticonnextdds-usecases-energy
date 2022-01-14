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

#ifndef ENERGY_ENUM_HELPER_H
#define ENERGY_ENUM_HELPER_H

#include <string>
#include "../generated/EnergyComms.hpp"

using namespace Energy::Enums;

namespace Energy {
    namespace utils {
        // Returns text version of ConnectionStatus enum
        const static std::string EnumName(ConnectionStatus status)
        {
            switch (status) {
            case ConnectionStatus::CONNECTED:
                return std::string("CONNECTED");
            case ConnectionStatus::DISCONNECTED:
                return std::string("DISCONNECTED");
            case ConnectionStatus::INVALID:
                return std::string("INVALID");
            default:
                return std::string();
            }
        }
        // Returns text version of MicrogridStatus enum
        const static std::string EnumName(MicrogridStatus status)
        {
            switch (status) {
            case MicrogridStatus::CONNECTED:
                return std::string("CONNECTED");
            case MicrogridStatus::INVALID:
                return std::string("INVALID");
            case MicrogridStatus::ISLANDED:
                return std::string("ISLANDED");
            case MicrogridStatus::REQUEST_ISLAND:
                return std::string("REQUEST_ISLAND");
            case MicrogridStatus::REQUEST_RESYNC:
                return std::string("REQUEST_RESYNC");
            default:
                return std::string();
            }
        }
        // Returns text version of OperationStatus enum
        const static std::string EnumName(OperationStatus status)
        {
            switch (status) {
            case OperationStatus::DISABLED_ERROR:
                return std::string("DISABLED_ERROR");
            case OperationStatus::DISABLED_OFF:
                return std::string("DISABLED_OFF");
            case OperationStatus::DISABLED_READY:
                return std::string("DISABLED_READY");
            case OperationStatus::ENABLED_ON:
                return std::string("ENABLED_ON");
            case OperationStatus::ENABLED_STARTING:
                return std::string("ENABLED_STARTING");
            case OperationStatus::ENABLED_VF_ON:
                return std::string("ENABLED_VF_ON");
            case OperationStatus::ENABLED_VF_READY:
                return std::string("ENABLED_VF_READY");
            case OperationStatus::INVALID:
                return std::string("INVALID");
            default:
                return std::string();
            }
        }
        // Returns text version of DeviceControl enum
        const static std::string EnumName(DeviceControl command)
        {
            switch (command) {
            case DeviceControl::CONNECT:
                return std::string("CONNECT");
            case DeviceControl::DISCONNECT:
                return std::string("DISCONNECT");
            case DeviceControl::INVALID:
                return std::string("INVALID");
            default:
                return std::string();
            }
        }
        // Returns text version of DeviceType enum
        const static std::string EnumName(DeviceType type)
        {
            switch (type) {
            case DeviceType::ENERGY_STORAGE:
                return std::string("ENERGY_STORAGE");
            case DeviceType::GENERATOR:
                return std::string("GENERATOR");
            case DeviceType::IED:
                return std::string("IED");
            case DeviceType::INTERCONNECT:
                return std::string("INTERCONNECT");
            case DeviceType::INVALID:
                return std::string("INVALID");
            case DeviceType::LOAD:
                return std::string("LOAD");
            case DeviceType::SOLAR_PV:
                return std::string("SOLAR_PV");
            default:
                return std::string();
            }
        }

        // Returns pretty text version of MicrogridStatus enum
        const static std::string PrettyName(MicrogridStatus status)
        {
            switch (status) {
            case MicrogridStatus::CONNECTED:
                return std::string("Connected");
            case MicrogridStatus::INVALID:
                return std::string("Invalid");
            case MicrogridStatus::ISLANDED:
                return std::string("Islanded");
            case MicrogridStatus::REQUEST_ISLAND:
                return std::string("Preparing to Island");
            case MicrogridStatus::REQUEST_RESYNC:
                return std::string("Preparing to Resynchronize");
            default:
                return std::string();
            }
        }
    }
}

#endif