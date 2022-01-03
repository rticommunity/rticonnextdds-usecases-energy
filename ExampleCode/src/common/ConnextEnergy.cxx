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

#include "ConnextEnergy.hpp"

ConnextEnergy::ConnextEnergy(const int domainId, const std::string& entityName) : 
    ConnextCommunicator(domainId, entityName)
{
    readers_ = new map<std::string, dds::sub::AnyDataReader*>();
    writers_ = new map<std::string, dds::pub::AnyDataWriter*>();
}

// ------------------ Getters for DataWriters -------------------

// Used to control the Main Interconnect for island/resync
dds::pub::DataWriter<Energy::Ops::Control_Device>
ConnextEnergy::WriterControl_Device()
{
    return Writer<Energy::Ops::Control_Device>(
        Energy::application::TOPIC_CONTROL_DEVICE,
        Energy::application::QOS_CONTROL);
}

// Used to set simulated irradiance
dds::pub::DataWriter<Energy::Common::CNTL_Single_float32>
ConnextEnergy::WriterControl_Irradiance()
{
    return Writer<Energy::Common::CNTL_Single_float32>(
        Energy::application::TOPIC_CONTROL_IRRADIANCE,
        Energy::application::QOS_CONTROL);
}

// Commands for the viz on Microgrid operations
dds::pub::DataWriter<Energy::Ops::Status_Microgrid>
ConnextEnergy::WriterControl_Microgrid()
{
    return Writer<Energy::Ops::Status_Microgrid>(
        Energy::application::TOPIC_CONTROL_MICROGRID,
        Energy::application::QOS_CONTROL);
}
// Used to control setpoints for ES, Generator, and PV
dds::pub::DataWriter<Energy::Common::CNTL_Single_float32>
ConnextEnergy::WriterControl_Power()
{
    return Writer<Energy::Common::CNTL_Single_float32>(
        Energy::application::TOPIC_CONTROL_POWER,
        Energy::application::QOS_CONTROL);
}

// Used to set simulated SOC
dds::pub::DataWriter<Energy::Common::CNTL_Single_float32>
ConnextEnergy::WriterControl_SOC()
{
    return Writer<Energy::Common::CNTL_Single_float32>(
        Energy::application::TOPIC_CONTROL_SOC,
        Energy::application::QOS_CONTROL);
}

// Sets Info specific to Energy Storage units
dds::pub::DataWriter<Energy::Ops::Info_Battery>
ConnextEnergy::WriterInfo_Battery()
{
    return Writer<Energy::Ops::Info_Battery>(
        Energy::application::TOPIC_INFO_BATTERY,
        Energy::application::QOS_INFO);
}

// Sets Info specific to Generators
dds::pub::DataWriter<Energy::Ops::Info_Generator>
ConnextEnergy::WriterInfo_Generator()
{
    return Writer<Energy::Ops::Info_Generator>(
        Energy::application::TOPIC_INFO_GENERATOR,
        Energy::application::QOS_INFO);
}

// Sets general IED device info
dds::pub::DataWriter<Energy::Ops::Info_Resource>
ConnextEnergy::WriterInfo_Resource()
{
    return Writer<Energy::Ops::Info_Resource>(
        Energy::application::TOPIC_INFO_RESOURCE,
        Energy::application::QOS_INFO);
}

// Used to update node measurements
dds::pub::DataWriter<Energy::Ops::Meas_NodePower>
ConnextEnergy::WriterMeas_NodePower()
{
    return Writer<Energy::Ops::Meas_NodePower>(
        Energy::application::TOPIC_MEAS_NODEPOWER,
        Energy::application::QOS_MEAS);
}

// Used to update state of charge measurement for energy storage
dds::pub::DataWriter<Energy::Common::MMXU_Single_float32>
ConnextEnergy::WriterMeas_SOC()
{
    return Writer<Energy::Common::MMXU_Single_float32>(
        Energy::application::TOPIC_MEAS_SOC,
        Energy::application::QOS_MEAS);
}

// Sets generic status of device
dds::pub::DataWriter<Energy::Ops::Status_Device>
ConnextEnergy::WriterStatus_Device()
{
    return Writer<Energy::Ops::Status_Device>(
        Energy::application::TOPIC_STATUS_DEVICE,
        Energy::application::QOS_STATUS);
}

// Actual Status of Microgrid
dds::pub::DataWriter<Energy::Ops::Status_Microgrid>
ConnextEnergy::WriterStatus_Microgrid()
{
    return Writer<Energy::Ops::Status_Microgrid>(
        Energy::application::TOPIC_STATUS_MICROGRID,
        Energy::application::QOS_STATUS);
}

// Sets the active VF Device based on ownership strength
dds::pub::DataWriter<Energy::Ops::VF_Device>
ConnextEnergy::WriterVF_Device()
{
    return Writer<Energy::Ops::VF_Device>(
        Energy::application::TOPIC_VF_DEVICE,
        Energy::application::QOS_VF_DEVICE);
}

// Used to tell all VF Devices which one will be active
dds::pub::DataWriter<Energy::Ops::VF_Device_Active>
ConnextEnergy::WriterVF_Device_Active()
{
    return Writer<Energy::Ops::VF_Device_Active>(
        Energy::application::TOPIC_VF_DEVICE_ACTIVE,
        Energy::application::QOS_VF_DEVICE_ACTIVE);
}

// ----------------- Getters for DataReaders -----------------------------
// Used to control a device for connection or disconnection of a relay
// associated with the device
dds::sub::DataReader<Energy::Ops::Control_Device>
ConnextEnergy::ReaderControl_Device()
{
    return Reader<Energy::Ops::Control_Device>(
        Energy::application::TOPIC_CONTROL_DEVICE,
        Energy::application::QOS_CONTROL);
}

// Used to set simulated irradiance
dds::sub::DataReader<Energy::Common::CNTL_Single_float32>
ConnextEnergy::ReaderControl_Irradiance()
{
    return Reader<Energy::Common::CNTL_Single_float32>(
        Energy::application::TOPIC_CONTROL_IRRADIANCE,
        Energy::application::QOS_CONTROL);
}

// Gets commands from the viz on Microgrid operations
dds::sub::DataReader<Energy::Ops::Status_Microgrid>
ConnextEnergy::ReaderControl_Microgrid()
{
    return Reader<Energy::Ops::Status_Microgrid>(
        Energy::application::TOPIC_CONTROL_MICROGRID,
        Energy::application::QOS_CONTROL);
}

// Used to get setpoints for ES, Generator, and PV
dds::sub::DataReader<Energy::Common::CNTL_Single_float32>
ConnextEnergy::ReaderControl_Power()
{
    return Reader<Energy::Common::CNTL_Single_float32>(
        Energy::application::TOPIC_CONTROL_POWER,
        Energy::application::QOS_CONTROL);
}

// Used to set simulated SOC
dds::sub::DataReader<Energy::Common::CNTL_Single_float32>
ConnextEnergy::ReaderControl_SOC()
{
    return Reader<Energy::Common::CNTL_Single_float32>(
        Energy::application::TOPIC_CONTROL_SOC,
        Energy::application::QOS_CONTROL);
}

// Gets Info for all ES units
dds::sub::DataReader<Energy::Ops::Info_Battery>
ConnextEnergy::ReaderInfo_Battery()
{
    return Reader<Energy::Ops::Info_Battery>(
        Energy::application::TOPIC_INFO_BATTERY,
        Energy::application::QOS_INFO);
}

// Gets info for all generators
dds::sub::DataReader<Energy::Ops::Info_Generator>
ConnextEnergy::ReaderInfo_Generator()
{
    return Reader<Energy::Ops::Info_Generator>(
        Energy::application::TOPIC_INFO_GENERATOR,
        Energy::application::QOS_INFO);
}

// Gets general resource information
dds::sub::DataReader<Energy::Ops::Info_Resource>
ConnextEnergy::ReaderInfo_Resource()
{
    return Reader<Energy::Ops::Info_Resource>(
        Energy::application::TOPIC_INFO_RESOURCE,
        Energy::application::QOS_INFO);
}

// Gets power devices are consuming or supplying
dds::sub::DataReader<Energy::Ops::Meas_NodePower>
ConnextEnergy::ReaderMeas_NodePower()
{
    return Reader<Energy::Ops::Meas_NodePower>(
        Energy::application::TOPIC_MEAS_NODEPOWER,
        Energy::application::QOS_MEAS);
}

// Gets SOC for all ES units
dds::sub::DataReader<Energy::Common::MMXU_Single_float32>
ConnextEnergy::ReaderMeas_SOC()
{
    return Reader<Energy::Common::MMXU_Single_float32>(
        Energy::application::TOPIC_MEAS_SOC,
        Energy::application::QOS_MEAS);
}

// Gets generic status of devices
dds::sub::DataReader<Energy::Ops::Status_Device>
ConnextEnergy::ReaderStatus_Device()
{
    return Reader<Energy::Ops::Status_Device>(
        Energy::application::TOPIC_STATUS_DEVICE,
        Energy::application::QOS_STATUS);
}

// Need to be able to read back the current Microgrid Status
dds::sub::DataReader<Energy::Ops::Status_Microgrid>
ConnextEnergy::ReaderStatus_Microgrid()
{
    return Reader<Energy::Ops::Status_Microgrid>(
        Energy::application::TOPIC_STATUS_MICROGRID,
        Energy::application::QOS_STATUS);
}

// Gets the active VF Device based on ownership strength
dds::sub::DataReader<Energy::Ops::VF_Device>
ConnextEnergy::ReaderVF_Device()
{
    return Reader<Energy::Ops::VF_Device>(
        Energy::application::TOPIC_VF_DEVICE,
        Energy::application::QOS_VF_DEVICE);
}

// Used for all VF Devices to tell which one will be active
dds::sub::DataReader<Energy::Ops::VF_Device_Active>
ConnextEnergy::ReaderVF_Device_Active()
{
    return Reader<Energy::Ops::VF_Device_Active>(
        Energy::application::TOPIC_VF_DEVICE_ACTIVE,
        Energy::application::QOS_VF_DEVICE_ACTIVE);
}

// ------------------------- Configuration Setters -----------------------
void ConnextEnergy::SetDataWriterOwnershipStrength(const std::string& topicString, const int32_t strength)
{
    try {
        auto temp_qos = (*writers_)[topicString]->qos();
        temp_qos << dds::core::policy::OwnershipStrength(strength);
        (*writers_)[topicString]->qos(temp_qos);
    }
    catch (string message) {
        cerr << "DataWriters must be initialized before setting ownership strength." << endl << message << endl;
    }

    return;
}