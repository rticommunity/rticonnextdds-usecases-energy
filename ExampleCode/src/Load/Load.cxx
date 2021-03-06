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

/* Load.cxx

A simulated Load Device

This file can be used as a starting point for any application interfacing a
load (smart or dumb) to a microgrid.
*/

#include "Load.hpp"

/* Load
* Load devices are generic IEDs. The big thing is that they only Load
*/
Load::Load(const int domainId, const std::string& entityName, const INIReader& config) :
    IED(domainId, entityName, Energy::Enums::DeviceType::LOAD, config)
{
    
}