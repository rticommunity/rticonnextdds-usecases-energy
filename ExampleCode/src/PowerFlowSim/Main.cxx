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

#include "PowerFlowSim.hpp"

#include <thread>
#include "../common/filesystem.hpp"
#include "../../../submodules/inih/INIReader.h"

using namespace std;

// ----------------------------------------------------------------------------
// PrintHelp:
// Function that prints out the help if somebody runs the application with
// --help
//
void PrintHelp()
{
    cout << "Valid options are: " << endl;
    cout << "    --domain [number]"
        << " Valid values 0-232. Domain this app " << endl
        << "                               "
        << "operates on. Default is 0." << endl;
    cout << "    --config [path]"
        << " Config file used to define device behavior." << endl
        << "                               "
        << "Default is PowerFlowSim.ini." << endl;
}

int main(int argc, char* argv[])
{
    int domainId = 0;
    std::string configFile = "PowerFlowSim.ini";

    for (int i = 0; i < argc; i++) {
        // Look for a Domain ID tag
        if (0 == strcmp(argv[i], "--domain")) {
            ++i;
            if (i == argc) {
                cout << "Bad parameter: Did not pass a domain number"
                    << endl;
                return -1;
            }
            domainId = atoi(argv[i]);

            // Only curently have 5 types of controller
            if (domainId < 0 || domainId > 232) {
                cout << "Bad parameter: valid domain IDs are between 0 and 232"
                    << endl;
                return -1;
            }
        }
        else if (0 == strcmp(argv[i], "--help")) {
            PrintHelp();
            return 0;
        }
        else if (0 == strcmp(argv[i], "--config")) {
            ++i;
            if (i == argc) {
                cout << "Bad parameter: Did not pass a configuration file"
                    << endl;
                return -1;
            }

            configFile = argv[i];

            // Check that specified file exists
            if (filesystem::exists(configFile)) {
                cout << "Bad parameter: config file does not exist"
                    << endl;
                return -1;
            }
        }
        else if (i > 0) {
            // If we have a parameter that is not the first one, and is not
            // recognized, return an error.
            cout << "Bad parameter: " << argv[i] << endl;
            PrintHelp();
            return -1;
        }
    }

    // To turn on additional logging, include <rti/config/Logger.hpp> and
    // uncomment the following line:
    // rti::config::Logger::instance().verbosity(rti::config::Verbosity::STATUS_ALL);

    try {
        INIReader config(configFile);

        string entityName(config.Get("PowerFlowSim", "DeviceID", "PowerFlowSim"));

        auto&& sim = new PowerFlowSim(domainId, entityName, config);

        thread executeThread(&PowerFlowSim::ExecuteSim, sim);

        cout << "Power Flow Simulator Operating. Press Enter to quit." << endl;

        cin.get();

        sim->StopSim();
        executeThread.join();

        cout << "PowerFlowSim Shut Down." << endl;
    }
    catch (const std::exception& ex) {
        // This will catch DDS exceptions
        std::cerr << "Exception in publisher_main(): " << ex.what()
            << std::endl;
        return -1;
    }

    // RTI Connext provides a finalize_participant_factory() method
    // if you want to release memory used by the participant factory singleton.
    // Uncomment the following line to release the singleton:
    //
    // dds::domain::DomainParticipant::finalize_participant_factory();

    return 0;
}