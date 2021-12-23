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

#include "ConnextCommunicator.hpp"

using namespace std;

ConnextCommunicator::ConnextCommunicator(const int domainId, const string& entityName)
{
    // Set up the QOS Provider with default configuration
    qos_ = new dds::core::QosProvider(dds::core::null);
    *qos_ = dds::core::QosProvider::Default();

    // Specify the Entity name for the participant
    auto qos_participant = qos_->participant_qos();
    rti::core::policy::EntityName setting(entityName);
    qos_participant << setting;

    // Initialize the Participant
    participant_ = new dds::domain::DomainParticipant(domainId, qos_participant);

    // Initialize the Subscriber and Publisher
    subscriber_ = new dds::sub::Subscriber(*participant_, qos_->subscriber_qos());
    publisher_ = new dds::pub::Publisher(*participant_, qos_->publisher_qos());
}

dds::pub::Publisher& ConnextCommunicator::Publisher()
{
    return *publisher_;
}

dds::sub::Subscriber& ConnextCommunicator::Subscriber()
{
    return *subscriber_;
}

dds::domain::DomainParticipant& ConnextCommunicator::Participant()
{
    return *participant_;
}

dds::core::QosProvider& ConnextCommunicator::Qos()
{
    return *qos_;
}