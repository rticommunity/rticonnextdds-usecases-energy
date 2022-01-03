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

#ifndef CONNEXT_ENERGY_H
#define CONNEXT_ENERGY_H

#include <map>

#include "ConnextCommunicator.hpp"
#include "../generated/EnergyComms.hpp"

using namespace std;

class ConnextEnergy : ConnextCommunicator {
public:
    // --- Constructor ---
    ConnextEnergy(const int domainId, const std::string& entityName);

    // ------------------ Getters for DataWriters -------------------
    // Used to control the Main Interconnect for island/resync
    dds::pub::DataWriter<Energy::Ops::Control_Device> WriterControl_Device();
    // Sets simulated irradiance
    dds::pub::DataWriter<Energy::Common::CNTL_Single_float32> WriterControl_Irradiance();
    // Commands for the viz on Microgrid operations
    dds::pub::DataWriter<Energy::Ops::Status_Microgrid> WriterControl_Microgrid();
    // Used to control setpoints for ES, Generator, and PV
    dds::pub::DataWriter<Energy::Common::CNTL_Single_float32> WriterControl_Power();
    // Used to set simulated SOC
    dds::pub::DataWriter<Energy::Common::CNTL_Single_float32> WriterControl_SOC();
    // Sets Info specific to Energy Storage units
    dds::pub::DataWriter<Energy::Ops::Info_Battery> WriterInfo_Battery();
    // Sets Info specific to Generators
    dds::pub::DataWriter<Energy::Ops::Info_Generator> WriterInfo_Generator();
    // Sets general IED device info
    dds::pub::DataWriter<Energy::Ops::Info_Resource> WriterInfo_Resource();
    // Used to update node measurements
    dds::pub::DataWriter<Energy::Ops::Meas_NodePower> WriterMeas_NodePower();
    // Used to update state of charge measurement for energy storage
    dds::pub::DataWriter<Energy::Common::MMXU_Single_float32> WriterMeas_SOC();
    // Sets generic status of device
    dds::pub::DataWriter<Energy::Ops::Status_Device> WriterStatus_Device();
    // Actual Status of Microgrid
    dds::pub::DataWriter<Energy::Ops::Status_Microgrid> WriterStatus_Microgrid();
    // Sets the active VF Device based on ownership strength
    dds::pub::DataWriter<Energy::Ops::VF_Device> WriterVF_Device();
    // Used to tell all VF Devices which one will be active
    dds::pub::DataWriter<Energy::Ops::VF_Device_Active> WriterVF_Device_Active();

    // ----------------- Getters for DataReaders -----------------------------
    // Used to control a device for connection or disconnection of a relay
    // associated with the device
    dds::sub::DataReader<Energy::Ops::Control_Device> ReaderControl_Device();
    // Sets simulated Irradiance
    dds::sub::DataReader<Energy::Common::CNTL_Single_float32> ReaderControl_Irradiance();
    // Gets commands from the viz on Microgrid operations
    dds::sub::DataReader<Energy::Ops::Status_Microgrid> ReaderControl_Microgrid();
    // Used to get setpoints for ES, Generator, and PV
    dds::sub::DataReader<Energy::Common::CNTL_Single_float32> ReaderControl_Power();
    // Used to set simulated SOC
    dds::sub::DataReader<Energy::Common::CNTL_Single_float32> ReaderControl_SOC();
    // Gets Info for all ES units
    dds::sub::DataReader<Energy::Ops::Info_Battery> ReaderInfo_Battery();
    // Gets info for all generators
    dds::sub::DataReader<Energy::Ops::Info_Generator> ReaderInfo_Generator();
    // Gets general resource information
    dds::sub::DataReader<Energy::Ops::Info_Resource> ReaderInfo_Resource();
    // Gets power devices are consuming or supplying
    dds::sub::DataReader<Energy::Ops::Meas_NodePower> ReaderMeas_NodePower();
    // Gets SOC for all ES units
    dds::sub::DataReader<Energy::Common::MMXU_Single_float32> ReaderMeas_SOC();
    // Gets generic status of devices
    dds::sub::DataReader<Energy::Ops::Status_Device> ReaderStatus_Device();
    // Need to be able to read back the current Microgrid Status
    dds::sub::DataReader<Energy::Ops::Status_Microgrid> ReaderStatus_Microgrid();
    // Gets the active VF Device based on ownership strength
    dds::sub::DataReader<Energy::Ops::VF_Device> ReaderVF_Device();
    // Used for all VF Devices to tell which one will be active
    dds::sub::DataReader<Energy::Ops::VF_Device_Active> ReaderVF_Device_Active();

    // --------------------------- Configuration Setters --------------------------
    void SetDataWriterOwnershipStrength(const std::string& topicString, const int32_t strength);

private:

    template <typename T>
    dds::sub::DataReader<T> Reader(const std::string& topicString, const std::string& qosString)
    {

        auto it = readers_->find(topicString);
        if (it != readers_->end())
            return (*readers_)[topicString]->get<T>();
        else {
            auto topic = dds::topic::find<dds::topic::Topic<T> >(Participant(), topicString);
            if (topic.is_nil())
                topic = dds::topic::Topic<T>(Participant(), topicString);
            dds::sub::qos::DataReaderQos datareaderQos(Qos().datareader_qos(qosString));

            dds::sub::DataReader<T> reader(Subscriber(), topic, datareaderQos);
            (*readers_)[topicString] = new dds::sub::AnyDataReader(reader);
            return reader;
        }
    }

    template <typename T>
    dds::pub::DataWriter<T> Writer(const std::string& topicString, const std::string& qosString)
    {
        auto it = writers_->find(topicString);
        if (it != writers_->end())
            return (*writers_)[topicString]->get<T>();
        else {
            auto topic = dds::topic::find<dds::topic::Topic<T> >(Participant(), topicString);
            if (topic.is_nil())
                topic = dds::topic::Topic<T>(Participant(), topicString);
            dds::pub::qos::DataWriterQos datawriterQos(Qos().datawriter_qos(qosString));

            dds::pub::DataWriter<T> writer(Publisher(), topic, datawriterQos);
            (*writers_)[topicString] = new dds::pub::AnyDataWriter(writer);
            return writer;
        }
    }

    // DataReaders
    map<std::string, dds::sub::AnyDataReader*>* readers_;
    // DataWriters
    map<std::string, dds::pub::AnyDataWriter*>* writers_;
};

#endif