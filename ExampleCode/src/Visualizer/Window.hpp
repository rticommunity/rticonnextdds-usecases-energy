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
#include <string>
#include <mutex>
#include <condition_variable>

#include "Visualizer.hpp"
#include "SafeQueue.hpp"

class MyWindow : public Gtk::Window, public Visualizer {
    Glib::RefPtr<Gtk::Application>& app_;
    Glib::RefPtr<Gtk::Builder>& builder_;
    Glib::RefPtr<Gio::SimpleActionGroup> actionGroup_;
    Gtk::Button* button_main_control, * button_sim_set_irradiance,
        * button_sim_set_soc, * button_sim_set_load,
        * button_sim_set_curtailment, * button_sim_trip;
    Gtk::Entry* entry_sim_set_irradiance, * entry_sim_set_soc,
        * entry_sim_set_load, * entry_sim_set_curtailment;
    Glib::RefPtr<Gtk::TextBuffer> textbuffer_pv_max_power, textbuffer_pv_meas,
        textbuffer_pv_status, textbuffer_gen_max_power, textbuffer_gen_meas,
        textbuffer_gen_status, textbuffer_load_max_power,
        textbuffer_load_meas, textbuffer_load_status,
        textbuffer_es_max_power, textbuffer_es_meas, textbuffer_es_status,
        textbuffer_es_capacity, textbuffer_es_soc, textbuffer_main_status,
        textbuffer_main_power;

    void buildUI();

private:
    void InitDDS();

    // Elements for processing DDS data to update UI
    void ThreadMeas_NodePower();
    SafeQueue::SafeQueue<Energy::Ops::Meas_NodePower> queue_Meas_NodePower_;
    void ThreadMeas_SOC();
    SafeQueue::SafeQueue<Energy::Common::MMXU_Single_float32> queue_Meas_SOC_;
    void ThreadInfo_Resource();
    SafeQueue::SafeQueue<Energy::Ops::Info_Resource> queue_Info_Resource_;
    void ThreadInfo_Battery();
    SafeQueue::SafeQueue<Energy::Ops::Info_Battery> queue_Info_Battery_;
    void ThreadInfo_Generator();
    SafeQueue::SafeQueue<Energy::Ops::Info_Generator> queue_Info_Generator_;
    void ThreadStatus_Device();
    SafeQueue::SafeQueue<Energy::Ops::Status_Device> queue_Status_Device_;
    void ThreadStatus_Microgrid();
    SafeQueue::SafeQueue<Energy::Ops::Status_Microgrid> queue_Status_Microgrid_;

    Glib::Dispatcher dispatcher_Meas_NodePower_;
    Glib::Dispatcher dispatcher_Meas_SOC_;
    Glib::Dispatcher dispatcher_Info_Resource_;
    Glib::Dispatcher dispatcher_Info_Battery_;
    Glib::Dispatcher dispatcher_Info_Generator_;
    Glib::Dispatcher dispatcher_Status_Device_;
    Glib::Dispatcher dispatcher_Status_Microgrid_;

protected:
    // signal handlers
    void button_main_control_clicked();
    void button_sim_set_irradiance_clicked();
    void button_sim_set_soc_clicked();
    void button_sim_set_load_clicked();
    void button_sim_set_curtailment_clicked();
    void button_sim_trip_clicked();

    // Overrides of Callbacks from Visualizer class
    void Callback_ReaderInfo_Resource() override;
    void Callback_ReaderInfo_Battery() override;
    void Callback_ReaderInfo_Generator() override;
    void Callback_ReaderMeas_NodePower() override;
    void Callback_ReaderMeas_SOC() override;
    void Callback_ReaderStatus_Device() override;
    void Callback_ReaderStatus_Microgrid() override;

    // Scheduling of buffer updates from subthreads to main thread
    struct DispatchData {
        GtkTextBuffer* buffer;
        char* output_str;
    };
    static gboolean display_textbuffer(DispatchData* data) {
        gtk_text_buffer_set_text(data->buffer, data->output_str, strlen(data->output_str));
        g_free(data);
        return G_SOURCE_REMOVE;
    }

public:
    MyWindow(
        BaseObjectType*, Glib::RefPtr<Gtk::Builder>&, Glib::RefPtr<Gtk::Application>&);
    ~MyWindow();
};