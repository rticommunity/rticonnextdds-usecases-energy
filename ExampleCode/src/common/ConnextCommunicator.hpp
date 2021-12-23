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

#ifndef CONNEXT_COMMUNICATOR_H
#define CONNEXT_COMMUNICATOR_H

// DDS Communicator
// Base behavior and initialization for the Energy Comms applications. Given
// that they all use the same IDL and QOS file, this simplifies initialization
// to the point where it can be mostly automated. The only difference between
// applications being which topics they publish or subscribe to and their 
// behavior as a result of available data.

#include <dds/dds.hpp>

class ConnextCommunicator {
public:
    // --- Constructor ---
    ConnextCommunicator(const int domainId, const std::string& entityName);

protected:
    dds::core::QosProvider& Qos();
    dds::domain::DomainParticipant& Participant();
    dds::sub::Subscriber& Subscriber();
    dds::pub::Publisher& Publisher();

private:
    // --- Private Members ---
    dds::core::QosProvider* qos_;
    dds::domain::DomainParticipant* participant_;
    dds::sub::Subscriber* subscriber_;
    dds::pub::Publisher* publisher_;
};

#endif