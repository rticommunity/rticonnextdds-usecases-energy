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

#include <gtkmm.h>
#include <vector>
#include <string>
#include <dds/dds.hpp>

#include "../generated/EnergyComms.hpp"

using namespace dds::domain;
using namespace dds::topic;
using namespace dds::pub;

using namespace Energy::Ops;
using namespace Energy::Common;

class MyWindow : public Gtk::Window {
    Glib::RefPtr<Gtk::Application>& app_;
    Glib::RefPtr<Gtk::Builder>& builder_;
    Glib::RefPtr<Gio::SimpleActionGroup> actionGroup_;
    Gtk::Button *button_main_control, *button_sim_set_irradiance,
            *button_sim_set_soc, *button_sim_set_load,
            *button_sim_set_curtailment, *button_sim_trip;
    Gtk::Entry *entry_sim_set_irradiance, *entry_sim_set_soc,
            *entry_sim_set_load, *entry_sim_set_curtailment;
    Glib::RefPtr<Gtk::TextBuffer> textbuffer_pv_max_power, textbuffer_pv_meas,
            textbuffer_pv_status, textbuffer_gen_max_power, textbuffer_gen_meas,
            textbuffer_gen_status, textbuffer_load_max_power,
            textbuffer_load_meas, textbuffer_load_status,
            textbuffer_es_max_power, textbuffer_es_meas, textbuffer_es_status,
            textbuffer_es_capacity, textbuffer_es_soc, textbuffer_main_status,
            textbuffer_main_power;

    void buildUI();
    void initDds();
    static void DdsThread(MyWindow*);

private:
    DomainParticipant* participant;

    Topic<Status_Microgrid>* TopicControl_Microgrid;
    Topic<Control_Device>* TopicControl_Device;
    Topic<CNTL_Single_float32>* TopicControl_Irradiance;
    Topic<CNTL_Single_float32>* TopicControl_SOC;
    Topic<CNTL_Single_float32>* TopicControl_Power;

    Publisher* publisher;

    DataWriter<Status_Microgrid>* WriterControl_Microgrid;
    DataWriter<Control_Device>* WriterControl_Device;
    DataWriter<CNTL_Single_float32>* WriterControl_Irradiance;
    DataWriter<CNTL_Single_float32>* WriterControl_SOC;
    DataWriter<CNTL_Single_float32>* WriterControl_Power;

protected:
    // signal handlers
    void button_main_control_clicked();
    void button_sim_set_irradiance_clicked();
    void button_sim_set_soc_clicked();
    void button_sim_set_load_clicked();
    void button_sim_set_curtailment_clicked();
    void button_sim_trip_clicked();

public:
    MyWindow(
            BaseObjectType*,
            Glib::RefPtr<Gtk::Builder>&,
            Glib::RefPtr<Gtk::Application>&);
    ~MyWindow();
};
