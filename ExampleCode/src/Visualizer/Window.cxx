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

#include <iostream>
#include <thread>
#include <sstream>
#include <iomanip>
//#include <cmath>
#include <string>
//#include <map>
#include <rti/core/ListenerBinder.hpp>

#include "Window.hpp"
//#include "DataWorkers.hpp"
#include "../common/EnergyEnumHelper.hpp"
#include "../../../submodules/inih/INIReader.h"

using namespace std;

MyWindow::MyWindow(
    BaseObjectType* cobject, Glib::RefPtr<Gtk::Builder>& builder, Glib::RefPtr<Gtk::Application>& app)
    : Gtk::Window{ cobject },
    app_{ app },
    builder_{ builder },
    actionGroup_{ nullptr },
    button_main_control{ nullptr },
    button_sim_set_irradiance{ nullptr },
    button_sim_set_soc{ nullptr },
    button_sim_set_load{ nullptr },
    button_sim_set_curtailment{ nullptr },
    button_sim_trip{ nullptr },
    entry_sim_set_irradiance{ nullptr },
    entry_sim_set_soc{ nullptr },
    entry_sim_set_load{ nullptr },
    entry_sim_set_curtailment{ nullptr },
    textbuffer_pv_max_power{ nullptr },
    textbuffer_pv_meas{ nullptr },
    textbuffer_pv_status{ nullptr },
    textbuffer_gen_max_power{ nullptr },
    textbuffer_gen_meas{ nullptr },
    textbuffer_gen_status{ nullptr },
    textbuffer_load_max_power{ nullptr },
    textbuffer_load_meas{ nullptr },
    textbuffer_load_status{ nullptr },
    textbuffer_es_max_power{ nullptr },
    textbuffer_es_meas{ nullptr },
    textbuffer_es_status{ nullptr },
    textbuffer_es_capacity{ nullptr },
    textbuffer_es_soc{ nullptr },
    textbuffer_main_status{ nullptr },
    textbuffer_main_power{ nullptr }
{
    set_title("Microgrid Visualizer");  // Sets the window title.
    set_default_size(800, 600);  // Set default size, width and height, in
                                 // pixels.
    set_border_width(10);
    std::thread(&MyWindow::InitDDS, this).detach();
    buildUI();
    add_events(Gdk::KEY_PRESS_MASK);
    show_all_children();
    //InitDDS();
}

MyWindow::~MyWindow()
{
    StopViz();
}

void MyWindow::InitDDS()
{
    auto&& executeThread = std::thread(&Visualizer::ExecuteViz, this);

    while (RunProcesses()) {
        if (!executeThread.joinable())
            std::cerr << "Thread " << executeThread.get_id() << " not joinable!\n";
        std::this_thread::sleep_for(1s);
    }

    executeThread.join();
}

void MyWindow::buildUI()
{
    // Get textbuffers
    Gtk::TextView* TV_temp;
    builder_->get_widget("textview_es_capacity", TV_temp);
    textbuffer_es_capacity = TV_temp->get_buffer();
    builder_->get_widget("textview_es_max_power", TV_temp);
    textbuffer_es_max_power = TV_temp->get_buffer();
    builder_->get_widget("textview_gen_max_power", TV_temp);
    textbuffer_gen_max_power = TV_temp->get_buffer();
    builder_->get_widget("textview_pv_max_power", TV_temp);
    textbuffer_pv_max_power = TV_temp->get_buffer();
    builder_->get_widget("textview_load_max_power", TV_temp);
    textbuffer_load_max_power = TV_temp->get_buffer();
    builder_->get_widget("textview_load_meas", TV_temp);
    textbuffer_load_meas = TV_temp->get_buffer();
    builder_->get_widget("textview_es_meas", TV_temp);
    textbuffer_es_meas = TV_temp->get_buffer();
    builder_->get_widget("textview_pv_meas", TV_temp);
    textbuffer_pv_meas = TV_temp->get_buffer();
    builder_->get_widget("textview_gen_meas", TV_temp);
    textbuffer_gen_meas = TV_temp->get_buffer();
    builder_->get_widget("textview_main_power", TV_temp);
    textbuffer_main_power = TV_temp->get_buffer();
    builder_->get_widget("textview_pv_status", TV_temp);
    textbuffer_pv_status = TV_temp->get_buffer();
    builder_->get_widget("textview_es_status", TV_temp);
    textbuffer_es_status = TV_temp->get_buffer();
    builder_->get_widget("textview_gen_status", TV_temp);
    textbuffer_gen_status = TV_temp->get_buffer();
    builder_->get_widget("textview_load_status", TV_temp);
    textbuffer_load_status = TV_temp->get_buffer();
    builder_->get_widget("textview_main_status", TV_temp);
    textbuffer_main_status = TV_temp->get_buffer();
    builder_->get_widget("textview_es_soc", TV_temp);
    textbuffer_es_soc = TV_temp->get_buffer();

    // Populate all pointers
    builder_->get_widget("button_main_control", button_main_control);
    builder_->get_widget("button_sim_set_irradiance", button_sim_set_irradiance);
    builder_->get_widget("button_sim_set_soc", button_sim_set_soc);
    builder_->get_widget("button_sim_set_load", button_sim_set_load);
    builder_->get_widget("button_sim_set_curtailment", button_sim_set_curtailment);
    builder_->get_widget("button_sim_trip", button_sim_trip);
    builder_->get_widget("entry_sim_set_irradiance", entry_sim_set_irradiance);
    builder_->get_widget("entry_sim_set_soc", entry_sim_set_soc);
    builder_->get_widget("entry_sim_set_load", entry_sim_set_load);
    builder_->get_widget("entry_sim_set_curtailment", entry_sim_set_curtailment);

    if (button_main_control == nullptr || button_sim_set_irradiance == nullptr
        || button_sim_set_soc == nullptr || button_sim_set_load == nullptr
        || button_sim_set_curtailment == nullptr || button_sim_trip == nullptr
        || entry_sim_set_irradiance == nullptr || entry_sim_set_soc == nullptr
        || entry_sim_set_load == nullptr
        || entry_sim_set_curtailment == nullptr) {
        g_warning("unable to extract window sub-components");
        return;
    }

    // Create tool tips for entries
    entry_sim_set_irradiance->set_tooltip_text(
        Glib::ustring("0-1000 in W/m^2"));
    entry_sim_set_soc->set_tooltip_text(Glib::ustring("0-100 as a percentage"));
    entry_sim_set_load->set_tooltip_text(Glib::ustring("The load in kW"));
    entry_sim_set_curtailment->set_tooltip_text(
        Glib::ustring("A limit on the max generation in kW"));
    // Create tool tips for Set buttons
    auto set_button_text = Glib::ustring("Click to set value.");
    button_sim_set_load->set_tooltip_text(set_button_text);
    button_sim_set_curtailment->set_tooltip_text(set_button_text);
    button_sim_set_soc->set_tooltip_text(set_button_text);
    button_sim_set_irradiance->set_tooltip_text(set_button_text);

    // set button listeners
    button_main_control->signal_clicked().connect(sigc::mem_fun(*this, &MyWindow::button_main_control_clicked));
    button_sim_set_curtailment->signal_clicked().connect(sigc::mem_fun(*this, &MyWindow::button_sim_set_curtailment_clicked));
    button_sim_set_irradiance->signal_clicked().connect(sigc::mem_fun(*this, &MyWindow::button_sim_set_irradiance_clicked));
    button_sim_set_load->signal_clicked().connect(sigc::mem_fun(*this, &MyWindow::button_sim_set_load_clicked));
    button_sim_set_soc->signal_clicked().connect(sigc::mem_fun(*this, &MyWindow::button_sim_set_soc_clicked));
    button_sim_trip->signal_clicked().connect(sigc::mem_fun(*this, &MyWindow::button_sim_trip_clicked));

    // Create handlers for dispatchers
    dispatcher_Meas_NodePower_.connect(sigc::mem_fun(*this, &MyWindow::ThreadMeas_NodePower));
    dispatcher_Meas_SOC_.connect(sigc::mem_fun(*this, &MyWindow::ThreadMeas_SOC));
    dispatcher_Info_Resource_.connect(sigc::mem_fun(*this, &MyWindow::ThreadInfo_Resource));
    dispatcher_Info_Battery_.connect(sigc::mem_fun(*this, &MyWindow::ThreadInfo_Battery));
    dispatcher_Info_Generator_.connect(sigc::mem_fun(*this, &MyWindow::ThreadInfo_Generator));
    dispatcher_Status_Device_.connect(sigc::mem_fun(*this, &MyWindow::ThreadStatus_Device));
    dispatcher_Status_Microgrid_.connect(sigc::mem_fun(*this, &MyWindow::ThreadStatus_Microgrid));
    
}  // MyWindow::buildUI


void MyWindow::button_main_control_clicked()
{
    Energy::Ops::Status_Microgrid sample(
        OptimizerID(), Energy::Enums::MicrogridStatus::REQUEST_ISLAND);
    if (button_main_control->get_label() == Glib::ustring("Island"))
        WriterControl_Microgrid().write(sample);
    else if (button_main_control->get_label() == Glib::ustring("Resync")) {
        sample.MicrogridStatus(Energy::Enums::MicrogridStatus::REQUEST_RESYNC);
        WriterControl_Microgrid().write(sample);
    }
}

void MyWindow::button_sim_set_soc_clicked()
{
    Energy::Common::CNTL_Single_float32 sample(
        EnergyStorageID(), VizID(), stof(entry_sim_set_soc->get_buffer()->get_text().raw()));
    WriterControl_SOC().write(sample);
}

void MyWindow::button_sim_set_load_clicked()
{
    Energy::Common::CNTL_Single_float32 sample(
        LoadID(), VizID(), stof(entry_sim_set_load->get_buffer()->get_text().raw()));
    WriterControl_Power().write(sample);
}

void MyWindow::button_sim_set_irradiance_clicked()
{
    Energy::Common::CNTL_Single_float32 sample(
        SolarID(), VizID(), stof(entry_sim_set_irradiance->get_buffer()->get_text().raw()));
    WriterControl_Irradiance().write(sample);
}

void MyWindow::button_sim_set_curtailment_clicked()
{
    try {
        Energy::Common::CNTL_Single_float32 sample(
            SolarID(), VizID(), stof(entry_sim_set_curtailment->get_buffer()->get_text().raw()));
        WriterControl_Power().write(sample);
    }
    catch (...) {
    }
}

void MyWindow::button_sim_trip_clicked()
{
    Energy::Ops::Control_Device sample(
        MainInterconnectID(), VizID(), Energy::Enums::DeviceControl::DISCONNECT);
    WriterControl_Device().write(sample);
}

void MyWindow::Callback_ReaderInfo_Resource()
{
    for (auto sample : ReaderInfo_Resource().read()) 
        if (sample.info().valid()) 
            queue_Info_Resource_.push(sample.data());

    this->dispatcher_Info_Resource_.emit();
    std::this_thread::sleep_for(100ms); // Keeps the UI thread from overloading
}

void MyWindow::Callback_ReaderInfo_Battery()
{
    for (auto sample : this->ReaderInfo_Battery().read()) 
        if (sample.info().valid() && sample.data().Device() == EnergyStorageID()) 
            queue_Info_Battery_.push(sample.data());

    this->dispatcher_Info_Battery_.emit();
    std::this_thread::sleep_for(100ms); // Keeps the UI thread from overloading
}

void MyWindow::Callback_ReaderInfo_Generator()
{
    for (auto sample : ReaderInfo_Generator().read())
        if (sample.info().valid() && sample.data().Device() == GeneratorID())
            queue_Info_Generator_.push(sample.data());

    this->dispatcher_Info_Generator_.emit();
    std::this_thread::sleep_for(100ms); // Keeps the UI thread from overloading
}

void MyWindow::Callback_ReaderMeas_NodePower()
{
    for (auto sample : ReaderMeas_NodePower().read())
        if (sample.info().valid())
            queue_Meas_NodePower_.push(sample.data());

    this->dispatcher_Meas_NodePower_.emit();
    std::this_thread::sleep_for(100ms); // Keeps the UI thread from overloading
}

void MyWindow::Callback_ReaderMeas_SOC()
{
    for (auto sample : ReaderMeas_SOC().read()) 
        if (sample.info().valid() && sample.data().Device() == EnergyStorageID()) 
            queue_Meas_SOC_.push(sample.data());

    this->dispatcher_Meas_SOC_.emit();
    std::this_thread::sleep_for(100ms); // Keeps the UI thread from overloading
}

void MyWindow::Callback_ReaderStatus_Device()
{
    for (auto sample : ReaderStatus_Device().read()) 
        if (sample.info().valid()) 
            queue_Status_Device_.push(sample.data());

    this->dispatcher_Status_Device_.emit();
    std::this_thread::sleep_for(100ms); // Keeps the UI thread from overloading
}

void MyWindow::Callback_ReaderStatus_Microgrid()
{
    for (auto sample : ReaderStatus_Microgrid().read()) 
        if (sample.info().valid()) 
            queue_Status_Microgrid_.push(sample.data());

    this->dispatcher_Status_Microgrid_.emit();
    std::this_thread::sleep_for(100ms); // Keeps the UI thread from overloading
}

void MyWindow::ThreadMeas_NodePower()
{
    Energy::Ops::Meas_NodePower newVal;
    //gdk_threads_add_idle

    while (RunProcesses()) {
        switch (queue_Meas_NodePower_.pop(newVal, 1ms)) {
        case SafeQueue::QueueResult::OK:
        {
            std::ostringstream stream_power;
            stream_power << fixed << setprecision(2);
            stream_power << newVal.Value();
            if (newVal.Device() == LoadID())
                this->textbuffer_load_meas->set_text(stream_power.str() + " kW");
            else if (newVal.Device() == MainInterconnectID())
                this->textbuffer_main_power->set_text(stream_power.str() + " kW");
            else if (newVal.Device() == SolarID())
                this->textbuffer_pv_meas->set_text(stream_power.str() + " kW");
            else if (newVal.Device() == EnergyStorageID())
                this->textbuffer_es_meas->set_text(stream_power.str() + " kW");
            else if (newVal.Device() == GeneratorID())
                this->textbuffer_gen_meas->set_text(stream_power.str() + " kW");
        }
        break;
        case SafeQueue::QueueResult::TIMEOUT:
        case SafeQueue::QueueResult::CLOSED:
            return;
        }
    }
}

void MyWindow::ThreadInfo_Resource()
{
    Energy::Ops::Info_Resource newVal;

    while (RunProcesses()) {
        switch (queue_Info_Resource_.pop(newVal, 1ms)) {
        case SafeQueue::QueueResult::OK:
        {
            std::ostringstream stream_power;
            stream_power << fixed << setprecision(2);
            if (newVal.Device() == LoadID()) {
                stream_power << newVal.MaxLoad();
                this->textbuffer_load_max_power->set_text(stream_power.str() + " kW");
            }
            else if (newVal.Device() == SolarID()) {
                stream_power << newVal.MaxGeneration();
                this->textbuffer_pv_max_power->set_text(stream_power.str() + " kW");
            }
        }
        break;
        case SafeQueue::QueueResult::TIMEOUT:
        case SafeQueue::QueueResult::CLOSED:
            return;
        }
    }
}

void MyWindow::ThreadInfo_Battery()
{
    Energy::Ops::Info_Battery newVal;

    while (RunProcesses()) {
        switch (queue_Info_Battery_.pop(newVal, 1ms)) {
        case SafeQueue::QueueResult::OK:
        {
            ostringstream stream_power, stream_capacity;
            stream_capacity << fixed << setprecision(2);
            stream_capacity << newVal.Capacity();
            stream_power << fixed << setprecision(2);
            stream_power << newVal.MaxGeneration();
            this->textbuffer_es_max_power->set_text(stream_power.str() + " kW");
            this->textbuffer_es_capacity->set_text(stream_capacity.str() + " kWh");
        }
        break;
        case SafeQueue::QueueResult::TIMEOUT:
        case SafeQueue::QueueResult::CLOSED:
            return;
        }
    }
}

void MyWindow::ThreadInfo_Generator()
{
    Energy::Ops::Info_Generator newVal;

    while (RunProcesses()) {
        switch (queue_Info_Generator_.pop(newVal, 1ms)) {
        case SafeQueue::QueueResult::OK:
        {
            std::ostringstream stream_power;
            stream_power << fixed << setprecision(2);
            stream_power << newVal.MaxGeneration();
            this->textbuffer_gen_max_power->set_text(stream_power.str() + " kW");
        }
        break;
        case SafeQueue::QueueResult::TIMEOUT:
        case SafeQueue::QueueResult::CLOSED:
            return;
        }
    }
}

void MyWindow::ThreadMeas_SOC()
{
    Energy::Common::MMXU_Single_float32 newVal;

    while (RunProcesses()) {
        switch (queue_Meas_SOC_.pop(newVal, 1ms)) {
        case SafeQueue::QueueResult::OK:
        {
            std::ostringstream stream_power;
            stream_power << fixed << setprecision(4);
            stream_power << abs(newVal.Value());
            this->textbuffer_es_soc->set_text(stream_power.str() + "%");
        }
        break;
        case SafeQueue::QueueResult::TIMEOUT:
        case SafeQueue::QueueResult::CLOSED:
            return;
        }
    }
}

void MyWindow::ThreadStatus_Device()
{
    // This function converts the enum to a string
    using Energy::utils::EnumName;
    
    Energy::Ops::Status_Device newVal;

    while (RunProcesses()) {
        switch (queue_Status_Device_.pop(newVal, 1ms)) {
        case SafeQueue::QueueResult::OK:
        {
            std::ostringstream streamObj;
            streamObj << EnumName(newVal.OperationStatus());
            if (newVal.Device() == LoadID())
                this->textbuffer_load_status->set_text(streamObj.str());
            else if (newVal.Device() == SolarID())
                this->textbuffer_pv_status->set_text(streamObj.str());
            else if (newVal.Device() == GeneratorID())
                this->textbuffer_gen_status->set_text(streamObj.str());
            else if (newVal.Device() == EnergyStorageID())
                this->textbuffer_es_status->set_text(streamObj.str());
        }
        break;
        case SafeQueue::QueueResult::TIMEOUT:
        case SafeQueue::QueueResult::CLOSED:
            return;
        }
    }
}

void MyWindow::ThreadStatus_Microgrid()
{
    // This function converts the enum to a pretty string
    using Energy::utils::PrettyName;
    using Energy::Enums::MicrogridStatus;

    Energy::Ops::Status_Microgrid newVal;

    while (RunProcesses()) {
        switch (queue_Status_Microgrid_.pop(newVal, 1ms)) {
        case SafeQueue::QueueResult::OK:
        {
            std::ostringstream streamObj;
            streamObj << PrettyName(newVal.MicrogridStatus());
            this->textbuffer_main_status->set_text(streamObj.str());
            if (newVal.MicrogridStatus() == MicrogridStatus::CONNECTED)
                this->button_main_control->set_label(Glib::ustring("Island"));
            else if (newVal.MicrogridStatus() == MicrogridStatus::ISLANDED)
                this->button_main_control->set_label(Glib::ustring("Resync"));
        }
        break;
        case SafeQueue::QueueResult::TIMEOUT:
        case SafeQueue::QueueResult::CLOSED:
            return;
        }
    }
}
