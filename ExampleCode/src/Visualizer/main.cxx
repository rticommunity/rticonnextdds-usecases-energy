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

#include <gtkmm/application.h>  // GTK::Application
#include "Window.hpp"
#include <iostream>

int main(int argc, char* argv[])
{
    try {
        // Create the default application to run.
        auto app = Gtk::Application::create(
            argc,
            argv,
            "RTI Microgrid Controller");
        // Instantiate builder from glade file
        auto builder = Gtk::Builder::create_from_file("Visualizer.glade");

        MyWindow* window_ = nullptr;

        builder->get_widget_derived("WindowMain", window_, app);
        if (window_ == nullptr) {
            g_warning("unable to extract window");
            return -1;
        }


        app->run(*window_);
        delete window_;

    }
    catch (const Glib::FileError& ex) {
        std::cerr << "FileError: " << ex.what() << std::endl;
        return 1;
    }
    catch (const Glib::MarkupError& ex) {
        std::cerr << "MarkupError: " << ex.what() << std::endl;
        return 1;
    }
    catch (const Gtk::BuilderError& ex) {
        std::cerr << "BuilderError: " << ex.what() << std::endl;
        return 1;
    }  // catch

    return 0;
}  // main