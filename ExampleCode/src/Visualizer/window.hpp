#include <gtkmm.h>
#include <vector>
#include <string>
#include <dds/dds.hpp>

#include "EnergyComms.hpp"
#include "listeners.hpp"

class MyWindow : public Gtk::Window {
    Glib::RefPtr<Gtk::Application> & app_;
    Glib::RefPtr<Gtk::Builder> & builder_;
    Glib::RefPtr<Gio::SimpleActionGroup> actionGroup_;
    Gtk::Button *button_main_control, *button_sim_set_irradiance,
        *button_sim_set_soc, *button_sim_set_load, *button_sim_set_curtailment,
        *button_sim_trip;
    Gtk::Entry *entry_sim_set_irradiance, *entry_sim_set_soc,
        *entry_sim_set_load, *entry_sim_set_curtailment;
    Glib::RefPtr<Gtk::TextBuffer> textbuffer_pv_max_power, textbuffer_pv_meas,
        textbuffer_pv_status, textbuffer_gen_max_power, textbuffer_gen_meas,
        textbuffer_gen_status, textbuffer_load_max_power, textbuffer_load_meas,
        textbuffer_load_status, textbuffer_es_max_power, textbuffer_es_meas,
        textbuffer_es_status, textbuffer_es_capacity, textbuffer_es_soc,
        textbuffer_main_status, textbuffer_main_power;

    void buildUI();
    static void DdsThread(MyWindow*);

protected:
    // signal handlers
    void button_main_control_clicked();
    void button_sim_set_irradiance_clicked();
    void button_sim_set_soc_clicked();
    void button_sim_set_load_clicked();
    void button_sim_set_curtailment_clicked();
    void button_sim_trip_clicked();

public:
    MyWindow( BaseObjectType *,
              Glib::RefPtr<Gtk::Builder> &,
              Glib::RefPtr<Gtk::Application> & );
    ~MyWindow();
};
