#include "window.hpp"
#include <iostream>
#include <thread>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <string>
#include <map>
#include <rti/core/ListenerBinder.hpp>
#include "EnergyComms.hpp"

using namespace std;

using namespace dds::core;
using namespace rti::core;
using namespace dds::core::status;
using namespace dds::domain;
using namespace dds::sub;
using namespace dds::pub;
using namespace dds::topic;

MyWindow::MyWindow(
    BaseObjectType * cobject,
    Glib::RefPtr<Gtk::Builder> & builder,
    Glib::RefPtr<Gtk::Application> & app) :
    Gtk::Window{cobject}, app_{app}, builder_{builder}, actionGroup_{nullptr},
    button_main_control{nullptr}, button_sim_set_irradiance{nullptr},
    button_sim_set_soc{nullptr}, button_sim_set_load{nullptr},
    button_sim_set_curtailment{nullptr}, button_sim_trip{nullptr},
    entry_sim_set_irradiance{nullptr}, entry_sim_set_soc{nullptr},
    entry_sim_set_load{nullptr}, entry_sim_set_curtailment{nullptr},
    textbuffer_pv_max_power{nullptr}, textbuffer_pv_meas{nullptr},
    textbuffer_pv_status{nullptr}, textbuffer_gen_max_power{nullptr},
    textbuffer_gen_meas{nullptr}, textbuffer_gen_status{nullptr},
    textbuffer_load_max_power{nullptr}, textbuffer_load_meas{nullptr},
    textbuffer_load_status{nullptr}, textbuffer_es_max_power{nullptr},
    textbuffer_es_meas{nullptr}, textbuffer_es_status{nullptr},
    textbuffer_es_capacity{nullptr}, textbuffer_es_soc{nullptr},
    textbuffer_main_status{nullptr}, textbuffer_main_power{nullptr}
{
    set_title( "Microgrid Visualizer" );      // Sets the window title.
    set_default_size( 800, 600 );  // Set default size, width and height, in pixels.
    set_border_width( 10 );
    buildUI();
    initDds();
    thread(DdsThread, this).detach();     // Create DDS Thread
    add_events (Gdk::KEY_PRESS_MASK);
    show_all_children();
}

MyWindow::~MyWindow() {}

void MyWindow::initDds() {
    // Create the Domain Participant QOS to set Entity Name
    auto qos_default = dds::core::QosProvider::Default();
    auto qos_participant = qos_default.participant_qos();
    rti::core::policy::EntityName entityName("Visualizer");
    qos_participant << entityName;
    // Set Ownershipstrength for writers
    auto qos_control = qos_default.datawriter_qos("EnergyCommsLibrary::Control");
    qos_control << dds::core::policy::OwnershipStrength(100000);

    // create DDS participant
    participant = new DomainParticipant(0, qos_participant);

    TopicControl_Microgrid = new Topic<Energy::Ops::Status_Microgrid>(
        *(participant), "Control_Microgrid");
    TopicControl_Device = new Topic<Energy::Ops::Control_Device>(
        *(participant), "Control_Device");
    TopicControl_Irradiance = new Topic<Energy::Common::CNTL_Single_float32>(
        *(participant), "Control_Irradiance");
    TopicControl_SOC = new Topic<Energy::Common::CNTL_Single_float32>(
        *(participant), "Control_SOC");
    TopicControl_Power = new Topic<Energy::Common::CNTL_Single_float32>(
        *(participant), "Control_Power");

    publisher = new Publisher(*(participant));

    WriterControl_Microgrid = new DataWriter<Energy::Ops::Status_Microgrid>(
        *(publisher), *(TopicControl_Microgrid), qos_control);
    WriterControl_Device = new DataWriter<Energy::Ops::Control_Device>(
        *(publisher), *(TopicControl_Device), qos_control);
    WriterControl_Irradiance = new DataWriter<Energy::Common::CNTL_Single_float32>(
        *(publisher), *(TopicControl_Irradiance), qos_control);
    WriterControl_SOC = new DataWriter<Energy::Common::CNTL_Single_float32>(
        *(publisher), *(TopicControl_SOC), qos_control);
    WriterControl_Power = new DataWriter<Energy::Common::CNTL_Single_float32>(
        *(publisher), *(TopicControl_Power), qos_control);
}

void MyWindow::DdsThread(MyWindow * main) {
    auto qos_default = dds::core::QosProvider::Default();

    // Create Topics -- and automatically register the types
    Topic<Energy::Ops::Info_Battery> TopicInfo_Battery(
        *(main->participant), "Info_Battery");
    Topic<Energy::Ops::Meas_NodePower> TopicMeas_NodePower(
        *(main->participant), "Meas_NodePower");
    Topic<Energy::Common::MMXU_Single_float32> TopicMeas_SOC(
        *(main->participant), "Meas_SOC");
    Topic<Energy::Ops::Info_Resource> TopicInfo_Resource(
        *(main->participant), "Info_Resource");
    Topic<Energy::Ops::Info_Generator> TopicInfo_Generator(
        *(main->participant), "Info_Generator");
    Topic<Energy::Ops::Status_Device> TopicStatus_Device(
        *(main->participant), "Status_Device");
    Topic<Energy::Ops::Status_Microgrid> TopicStatus_Microgrid(
        *(main->participant), "Status_Microgrid");
    /*Topic<Energy::Common::CNTL_Single_float32> TopicControl_Power(
     *(main->participant), "Control_Power");*/

    // Create Subscriber
    Subscriber subscriber(*(main->participant));

    // Get QOS for info, status, and measurement
    auto qos_info = qos_default.datareader_qos("EnergyCommsLibrary::Info");
    auto qos_meas = qos_default.datareader_qos("EnergyCommsLibrary::Measurement");
    auto qos_status = qos_default.datareader_qos("EnergyCommsLibrary::Status");

    // Create DataReaders
    DataReader<Energy::Ops::Info_Battery> ReaderInfo_Battery(
        subscriber, TopicInfo_Battery, qos_info);
    DataReader<Energy::Ops::Info_Generator> ReaderInfo_Generator(
        subscriber, TopicInfo_Generator, qos_info);
    DataReader<Energy::Ops::Info_Resource> ReaderInfo_Resource(
        subscriber, TopicInfo_Resource, qos_info);
    DataReader<Energy::Ops::Meas_NodePower> ReaderMeas_NodePower(
        subscriber, TopicMeas_NodePower, qos_meas);
    DataReader<Energy::Common::MMXU_Single_float32> ReaderMeas_SOC(
        subscriber, TopicMeas_SOC, qos_meas);
    DataReader<Energy::Ops::Status_Device> ReaderStatus_Device(
        subscriber, TopicStatus_Device, qos_status);
    DataReader<Energy::Ops::Status_Microgrid> ReaderStatus_Microgrid(
        subscriber, TopicStatus_Microgrid, qos_status);

    // This is used for all the Read Conditions
    dds::sub::status::DataState commonDataState = dds::sub::status::DataState(
        dds::sub::status::SampleState::not_read(),
        dds::sub::status::ViewState::any(),
        dds::sub::status::InstanceState::alive());

    dds::sub::cond::ReadCondition ReadConditionInfo_Battery(
        ReaderInfo_Battery, commonDataState,
        [&ReaderInfo_Battery, main]() {
            for (auto sample : ReaderInfo_Battery.take()) {
                if (sample.info().valid() &&
                    sample.data().Device() == "SampleES")
                {
                    ostringstream stream_power, stream_capacity;
                    stream_capacity << fixed << setprecision(2);
                    stream_power << fixed << setprecision(2);
                    stream_power << sample.data().MaxGeneration();
                    stream_capacity << sample.data().Capacity();
                    main->textbuffer_es_max_power->
                        set_text(stream_power.str() + " kW");
                    main->textbuffer_es_capacity->
                        set_text(stream_capacity.str() + " kWh");
                }
            }
        });

    dds::sub::cond::ReadCondition ReadConditionInfo_Generator(
        ReaderInfo_Generator, commonDataState,
        [&ReaderInfo_Generator, main]() {
            for (auto sample : ReaderInfo_Generator.take()) {
                if (sample.info().valid() &&
                    sample.data().Device() == "SampleGen")
                {
                    std::ostringstream stream_power;
                    stream_power << fixed << setprecision(2);
                    stream_power << sample.data().MaxGeneration();
                    main->textbuffer_gen_max_power->
                        set_text(stream_power.str() + " kW");
                }
            }
        });

    dds::sub::cond::ReadCondition ReadConditionInfo_Resource(
        ReaderInfo_Resource, commonDataState,
        [&ReaderInfo_Resource, main]() {
            for (auto sample : ReaderInfo_Resource.take()) {
                if (sample.info().valid()) {
                    std::ostringstream stream_power;
                    stream_power << fixed << setprecision(2);
                    if (sample.data().Device() ==  "SampleLoad") {
                        stream_power << sample.data().MaxLoad();
                        main->textbuffer_load_max_power->
                            set_text(stream_power.str() + " kW");
                    } else if (sample.data().Device() ==  "SamplePV") {
                        stream_power << sample.data().MaxGeneration();
                        main->textbuffer_pv_max_power->
                            set_text(stream_power.str() + " kW");
                    }
                }
            }
        });

    dds::sub::cond::ReadCondition ReadConditionMeas_NodePower(
        ReaderMeas_NodePower, commonDataState,
        [&ReaderMeas_NodePower, main]() {
            for (auto sample : ReaderMeas_NodePower.take()) {
                if (sample.info().valid()) {
                    std::ostringstream stream_power;
                    stream_power << fixed << setprecision(2);
                    stream_power << sample.data().Value();
                    if (sample.data().Device() ==  "SampleLoad")
                        main->textbuffer_load_meas->
                            set_text(stream_power.str() + " kW");
                    else if (sample.data().Device() ==  "SamplePV")
                        main->textbuffer_pv_meas->
                            set_text(stream_power.str() + " kW");
                    else if (sample.data().Device() ==  "SampleGen")
                        main->textbuffer_gen_meas->
                            set_text(stream_power.str() + " kW");
                    else if (sample.data().Device() ==  "SampleES")
                        main->textbuffer_es_meas->
                            set_text(stream_power.str() + " kW");
                    else if (sample.data().Device() ==  "SampleInterconnect")
                        main->textbuffer_main_power->
                            set_text(stream_power.str() + " kW");
                }
            }
        });

    dds::sub::cond::ReadCondition ReadConditionMeas_SOC(
        ReaderMeas_SOC, commonDataState,
        [&ReaderMeas_SOC, main]() {
            for (auto sample : ReaderMeas_SOC.take()) {
                if (sample.info().valid() && sample.data().Device() == "SampleES") {
                    std::ostringstream stream_power;
                    stream_power << fixed << setprecision(4);
                    stream_power << abs(sample.data().Value());
                    main->textbuffer_es_soc->set_text(stream_power.str() + "%");
                }
            }
        });

    dds::sub::cond::ReadCondition ReadConditionStatus_Device(
        ReaderStatus_Device, commonDataState,
        [&ReaderStatus_Device, main]() {
            // map for device status
            using namespace Energy::Enums;
            std::map<int, std::string> status_map = {
                {(int)OperationStatus::inner_enum::DISABLED_OFF, "DISABLED_OFF"},
                {(int)OperationStatus::inner_enum::DISABLED_ERROR, "DISABLED_ERROR"},
                {(int)OperationStatus::inner_enum::DISABLED_READY, "DISABLED_READY"},
                {(int)OperationStatus::inner_enum::ENABLED_STARTING, "ENABLED_STARTING"},
                {(int)OperationStatus::inner_enum::ENABLED_ON, "ENABLED_ON"},
                {(int)OperationStatus::inner_enum::ENABLED_VF_READY, "ENABLED_VF_READY"},
                {(int)OperationStatus::inner_enum::ENABLED_VF_ON, "ENABLED_VF_ON"}
            };
            for (auto sample : ReaderStatus_Device.take()) {
                if (sample.info().valid()) {
                    std::ostringstream streamObj;
                    streamObj << status_map[sample.data().OperationStatus().underlying()];
                    if (sample.data().Device() ==  "SampleLoad")
                        main->textbuffer_load_status->set_text(streamObj.str());
                    else if (sample.data().Device() ==  "SamplePV")
                        main->textbuffer_pv_status->set_text(streamObj.str());
                    else if (sample.data().Device() ==  "SampleGen")
                        main->textbuffer_gen_status->set_text(streamObj.str());
                    else if (sample.data().Device() ==  "SampleES")
                        main->textbuffer_es_status->set_text(streamObj.str());
                }
            }
        });

    dds::sub::cond::ReadCondition ReadConditionStatus_Microgrid(
        ReaderStatus_Microgrid, commonDataState,
        [&ReaderStatus_Microgrid, main]() {
            // map for microgrid status
            using namespace Energy::Enums;
            std::map<int, std::string> status_map = {
                {(int)MicrogridStatus::inner_enum::CONNECTED, "On Grid"},
                {(int)MicrogridStatus::inner_enum::REQUEST_ISLAND, "Preparing to Island"},
                {(int)MicrogridStatus::inner_enum::ISLANDED, "Islanded"},
                {(int)MicrogridStatus::inner_enum::REQUEST_RESYNC, "Preparing to Resynchonize"}
            };
            for (auto sample : ReaderStatus_Microgrid.take()) {
                if (sample.info().valid()) {
                    std::ostringstream streamObj;
                    streamObj << status_map[sample.data().MicrogridStatus().underlying()];
                    main->textbuffer_main_status->set_text(streamObj.str());
                    if (sample.data().MicrogridStatus().underlying() ==
                        MicrogridStatus::inner_enum::CONNECTED)
                        main->button_main_control->set_label(Glib::ustring("Island"));
                    else if (sample.data().MicrogridStatus().underlying() ==
                             MicrogridStatus::inner_enum::ISLANDED)
                        main->button_main_control->set_label(Glib::ustring("Resync"));
                }
            }
        });

    dds::core::cond::WaitSet waitset;
    waitset += ReadConditionInfo_Battery;
    waitset += ReadConditionInfo_Generator;
    waitset += ReadConditionInfo_Resource;
    waitset += ReadConditionMeas_NodePower;
    waitset += ReadConditionMeas_SOC;
    waitset += ReadConditionStatus_Device;
    waitset += ReadConditionStatus_Microgrid;

    // Here we are handling our waitset and reactions to inputs
    while (true) {
        // Dispatch will call the handlers associated to the WaitSet conditions
        // when they activate
        waitset.dispatch(dds::core::Duration(4));  // Wait up to 4s each time
    }
}

    void MyWindow::buildUI() {
    // Get textbuffers
    Gtk::TextView* TV_temp;
    builder_->get_widget ("textview_es_capacity", TV_temp);
    textbuffer_es_capacity = TV_temp->get_buffer();
    builder_->get_widget ("textview_es_max_power", TV_temp);
    textbuffer_es_max_power = TV_temp->get_buffer();
    builder_->get_widget ("textview_gen_max_power", TV_temp);
    textbuffer_gen_max_power = TV_temp->get_buffer();
    builder_->get_widget ("textview_pv_max_power", TV_temp);
    textbuffer_pv_max_power = TV_temp->get_buffer();
    builder_->get_widget ("textview_load_max_power", TV_temp);
    textbuffer_load_max_power = TV_temp->get_buffer();
    builder_->get_widget ("textview_load_meas", TV_temp);
    textbuffer_load_meas = TV_temp->get_buffer();
    builder_->get_widget ("textview_es_meas", TV_temp);
    textbuffer_es_meas = TV_temp->get_buffer();
    builder_->get_widget ("textview_pv_meas", TV_temp);
    textbuffer_pv_meas = TV_temp->get_buffer();
    builder_->get_widget ("textview_gen_meas", TV_temp);
    textbuffer_gen_meas = TV_temp->get_buffer();
    builder_->get_widget ("textview_main_power", TV_temp);
    textbuffer_main_power = TV_temp->get_buffer();
    builder_->get_widget ("textview_pv_status", TV_temp);
    textbuffer_pv_status = TV_temp->get_buffer();
    builder_->get_widget ("textview_es_status", TV_temp);
    textbuffer_es_status = TV_temp->get_buffer();
    builder_->get_widget ("textview_gen_status", TV_temp);
    textbuffer_gen_status = TV_temp->get_buffer();
    builder_->get_widget ("textview_load_status", TV_temp);
    textbuffer_load_status = TV_temp->get_buffer();
    builder_->get_widget ("textview_main_status", TV_temp);
    textbuffer_main_status = TV_temp->get_buffer();
    builder_->get_widget ("textview_es_soc", TV_temp);
    textbuffer_es_soc = TV_temp->get_buffer();

    //Populate all pointers
    builder_->get_widget( "button_main_control", button_main_control );
    builder_->get_widget( "button_sim_set_irradiance", button_sim_set_irradiance );
    builder_->get_widget( "button_sim_set_soc", button_sim_set_soc );
    builder_->get_widget( "button_sim_set_load", button_sim_set_load );
    builder_->get_widget( "button_sim_set_curtailment", button_sim_set_curtailment );
    builder_->get_widget( "button_sim_trip", button_sim_trip );
    builder_->get_widget( "entry_sim_set_irradiance", entry_sim_set_irradiance );
    builder_->get_widget( "entry_sim_set_soc", entry_sim_set_soc );
    builder_->get_widget( "entry_sim_set_load", entry_sim_set_load );
    builder_->get_widget( "entry_sim_set_curtailment", entry_sim_set_curtailment );

    if ( button_main_control == nullptr || button_sim_set_irradiance == nullptr ||
         button_sim_set_soc == nullptr || button_sim_set_load == nullptr ||
         button_sim_set_curtailment == nullptr || button_sim_trip == nullptr ||
         entry_sim_set_irradiance == nullptr || entry_sim_set_soc == nullptr ||
         entry_sim_set_load == nullptr || entry_sim_set_curtailment == nullptr) {
        g_warning("unable to extract window sub-components");
        return;
    }

    // set button listeners
    button_main_control->signal_clicked().connect(
        sigc::mem_fun(*this, &MyWindow::button_main_control_clicked) );
    button_sim_set_curtailment->signal_clicked().connect(
        sigc::mem_fun(*this, &MyWindow::button_sim_set_curtailment_clicked) );
    button_sim_set_irradiance->signal_clicked().connect(
        sigc::mem_fun(*this, &MyWindow::button_sim_set_irradiance_clicked) );
    button_sim_set_load->signal_clicked().connect(
        sigc::mem_fun(*this, &MyWindow::button_sim_set_load_clicked) );
    button_sim_set_soc->signal_clicked().connect(
        sigc::mem_fun(*this, &MyWindow::button_sim_set_soc_clicked) );
    button_sim_trip->signal_clicked().connect(
        sigc::mem_fun(*this, &MyWindow::button_sim_trip_clicked) );

} // MyWindow::buildUI


void MyWindow::button_main_control_clicked() {
    Energy::Ops::Status_Microgrid sample(
        "SampleOpt", Energy::Enums::MicrogridStatus::REQUEST_ISLAND);
    if (button_main_control->get_label() == Glib::ustring("Island"))
        WriterControl_Microgrid->write(sample);
    else if (button_main_control->get_label() == Glib::ustring("Resync")) {
        sample.MicrogridStatus(Energy::Enums::MicrogridStatus::REQUEST_RESYNC);
        WriterControl_Microgrid->write(sample);
    }
}

void MyWindow::button_sim_set_soc_clicked(){
    Energy::Common::CNTL_Single_float32 sample(
        "SampleES", "Visualizer",
        stof(entry_sim_set_soc->get_buffer()->get_text().raw()));
    WriterControl_SOC->write(sample);
}

void MyWindow::button_sim_set_load_clicked() {
    Energy::Common::CNTL_Single_float32 sample(
        "SampleLoad", "Visualizer",
        stof(entry_sim_set_load->get_buffer()->get_text().raw()));
    WriterControl_Power->write(sample);
}

void MyWindow::button_sim_set_irradiance_clicked() {
    Energy::Common::CNTL_Single_float32 sample(
        "SamplePV", "Visualizer",
        stof(entry_sim_set_irradiance->get_buffer()->get_text().raw()));
    WriterControl_Irradiance->write(sample);
}

void MyWindow::button_sim_set_curtailment_clicked() {
    Energy::Common::CNTL_Single_float32 sample(
        "SamplePV", "Visualizer",
        stof(entry_sim_set_curtailment->get_buffer()->get_text().raw()));
    WriterControl_Power->write(sample);
}

void MyWindow::button_sim_trip_clicked() {
    Energy::Ops::Control_Device sample(
        "SampleInterconnect", "Visualizer", Energy::Enums::DeviceControl::DISCONNECT);
    WriterControl_Device->write(sample);
}
