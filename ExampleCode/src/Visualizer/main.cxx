#include <gtkmm/application.h> // GTK::Application
#include "window.hpp"
#include <iostream>
#include <dds/dds.hpp>

int main( int argc, char * argv[] ) {
    try {
        // Create the default application to run.
        auto app = Gtk::Application::create( argc, argv,
                                             "RTI Microgrid Controller" );
        // Instantiate builder from glade file
        auto builder = Gtk::Builder::create_from_file("Visualizer.glade");

        MyWindow *window_ = nullptr;

        builder->get_widget_derived( "WindowMain", window_, app );
        if ( window_ == nullptr ) {
            g_warning("unable to extract window");
            return -1;
        }



        app->run( *window_ );
        delete window_;

    }  catch( const Glib::FileError & ex ) {
        std::cerr << "FileError: " << ex.what() << std::endl;
        return 1;
    } catch( const Glib::MarkupError & ex ) {
        std::cerr << "MarkupError: " << ex.what() << std::endl;
        return 1;
    } catch( const Gtk::BuilderError & ex ) {
        std::cerr << "BuilderError: " << ex.what() << std::endl;
        return 1;
    } // catch

    return 0;
} // main
