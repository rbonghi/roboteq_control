/**
 * Copyright (C) 2017, Raffaello Bonghi <raffaello@rnext.it>
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "roboteq/roboteq.h"
#include <boost/algorithm/string.hpp>

namespace roboteq
{

Roboteq::Roboteq(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, serial_controller *serial)
    : DiagnosticTask("Roboteq")
    , mNh(nh)
    , private_mNh(private_nh)
    , mSerial(serial)
{
    // First run dynamic reconfigurator
    setup_controller = false;
    // Initialize GPIO reading
    _isGPIOreading = false;
    // Load default configuration roboteq board
    getRoboteqInformation();

    //Services
    srv_board = private_mNh.advertiseService("system", &Roboteq::service_Callback, this);

    _first = false;
    std::vector<std::string> joint_list;
    if(private_nh.hasParam("joint"))
    {
        private_nh.getParam("joint", joint_list);
    }
    else
    {
        ROS_WARN("No joint list!");
        joint_list.push_back("joint_0");
        joint_list.push_back("joint_1");
        private_nh.setParam("joint", joint_list);
    }
    // Disable ECHO
    mSerial->echo(false);
    // Disable Script and wait to load all parameters
    mSerial->script(false);
    // Stop motors
    bool stop_motor = mSerial->command("EX");
    ROS_DEBUG_STREAM("Stop motor: " << (stop_motor ? "true" : "false"));

    // Initialize Joints
    for(unsigned i=0; i < joint_list.size(); ++i)
    {
        string motor_name = joint_list.at(i);
        int number = i + 1;

        if(!private_nh.hasParam(motor_name))
        {
            _first = true;
            ROS_WARN_STREAM("Load " << motor_name << " parameters from Roboteq board");
        }

        if(private_nh.hasParam(motor_name + "/number"))
        {
            private_nh.getParam(motor_name + "/number", number);
        }
        else
        {
            ROS_WARN_STREAM("Default number selected for Motor: " << motor_name << " is " << number);
            private_nh.setParam(motor_name + "/number", number);
        }

        ROS_INFO_STREAM("Motor[" << number << "] name: " << motor_name);
        //mMotor[motor_name] = new Motor(private_mNh, serial, motor_name, number);
        mMotor.push_back(new Motor(private_mNh, serial, motor_name, number));
    }

    // Launch initialization input/output
    for(int i = 0; i < 6; ++i)
    {
        _param_pulse.push_back(new GPIOPulseConfigurator(private_mNh, serial, mMotor, "/InOut", i+1));
    }
    for(int i = 0; i < 6; ++i)
    {
        _param_analog.push_back(new GPIOAnalogConfigurator(private_mNh, serial, mMotor, "/InOut", i+1));
    }
    for(int i = 0; i < 2; ++i)
    {
        _param_encoder.push_back(new GPIOEncoderConfigurator(private_mNh, serial, mMotor, "/InOut", i+1));
    }

    // Add subscriber stop
    sub_stop = private_mNh.subscribe("emergency_stop", 1, &Roboteq::stop_Callback, this);
    // Initialize the peripheral publisher
    pub_peripheral = private_mNh.advertise<roboteq_control::Peripheral>("peripheral", 10,
                boost::bind(&Roboteq::connectionCallback, this, _1), boost::bind(&Roboteq::connectionCallback, this, _1));

}

void Roboteq::connectionCallback(const ros::SingleSubscriberPublisher& pub) {
    // Information about the subscriber
    ROS_INFO_STREAM("Update: " << pub.getSubscriberName() << " - " << pub.getTopic());
    // Check if some subscriber is connected with peripheral publisher
    _isGPIOreading = (pub_peripheral.getNumSubscribers() >= 1);
}

void Roboteq::stop_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    bool status = (int)msg.get()->data;
    if(status)
    {
        // Send emergency stop
        mSerial->command("EX");
        ROS_WARN_STREAM("Emergency stop");
    } else
    {
        // Safety release
        mSerial->command("MG");
        ROS_WARN_STREAM("Safety release");
    }

}

void Roboteq::getRoboteqInformation()
{
    // Load model roboeq board
    string trn = mSerial->getQuery("TRN");
    std::vector<std::string> fields;
    boost::split(fields, trn, boost::algorithm::is_any_of(":"));
    _type = fields[0];
    _model = fields[1];
    // ROS_INFO_STREAM("Model " << _model);
    // Load firmware version
    _version = mSerial->getQuery("FID");
    // Load UID
    _uid = mSerial->getQuery("UID");
}

Roboteq::~Roboteq()
{
    // ROS_INFO_STREAM("Script: " << script(false));
}

void Roboteq::initialize()
{
    // Check if is required load paramers
    if(_first)
    {
        // Load parameters from roboteq
        getControllerFromRoboteq();
    }

    // Initialize parameter dynamic reconfigure
    mDynRecServer = boost::make_shared<dynamic_reconfigure::Server<roboteq_control::RoboteqControllerConfig>>(mDynServerMutex);
    dynamic_reconfigure::Server<roboteq_control::RoboteqControllerConfig>::CallbackType f;
    f = boost::bind(&Roboteq::reconfigureCBController, this, _1, _2);
    mDynRecServer->setCallback(f);
    // Update parameters
    roboteq_control::RoboteqControllerConfig config;
    mDynRecServer->getConfigDefault(config);
    mDynServerMutex.lock();
    mDynRecServer->updateConfig(config);
    mDynServerMutex.unlock();

    // Launch initialization GPIO
    for (vector<GPIOPulseConfigurator*>::iterator it = _param_pulse.begin() ; it != _param_pulse.end(); ++it)
    {
        ((GPIOPulseConfigurator*)(*it))->initConfigurator(true);
    }
    for (vector<GPIOAnalogConfigurator*>::iterator it = _param_analog.begin() ; it != _param_analog.end(); ++it)
    {
        ((GPIOAnalogConfigurator*)(*it))->initConfigurator(true);
    }
    for (vector<GPIOEncoderConfigurator*>::iterator it = _param_encoder.begin() ; it != _param_encoder.end(); ++it)
    {
        ((GPIOEncoderConfigurator*)(*it))->initConfigurator(true);
    }

    // Initialize all motors in list
    for (vector<Motor*>::iterator it = mMotor.begin() ; it != mMotor.end(); ++it)
    {
        Motor* motor = ((Motor*)(*it));
        // Launch initialization motors
        motor->initializeMotor(_first);
        ROS_DEBUG_STREAM("Motor [" << motor->getName() << "] Initialized");
    }
}

void Roboteq::initializeInterfaces()
{
    // Initialize the diagnostic from the primitive object
    initializeDiagnostic();

    if (!model.initParam("robot_description")){
        ROS_ERROR("Failed to parse urdf file");
    }
    else
    {
        ROS_INFO_STREAM("robot_description found! " << model.name_ << " parsed!");
    }

    for (vector<Motor*>::iterator it = mMotor.begin() ; it != mMotor.end(); ++it)
    {
        Motor* motor = ((Motor*)(*it));
        /// State interface
        joint_state_interface.registerHandle(motor->joint_state_handle);
        /// Velocity interface
        velocity_joint_interface.registerHandle(motor->joint_handle);

        // Setup limits
        motor->setupLimits(model);

        // reset position joint
        double position = 0;
        ROS_DEBUG_STREAM("Motor [" << motor->getName() << "] reset position to: " << position);
        motor->resetPosition(position);

        //Add motor in diagnostic updater
        diagnostic_updater.add(*(motor));
        ROS_DEBUG_STREAM("Motor [" << motor->getName() << "] Registered");
    }

    ROS_DEBUG_STREAM("Send all Constraint configuration");

    /// Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&velocity_joint_interface);
}

void Roboteq::initializeDiagnostic()
{
    ROS_INFO_STREAM("Roboteq " << _type << ":" << _model);
    ROS_INFO_STREAM(_version);

    diagnostic_updater.setHardwareID(_model);

    // Initialize this diagnostic interface
    diagnostic_updater.add(*this);
}

void Roboteq::updateDiagnostics()
{
    ROS_DEBUG_STREAM("Update diagnostic");

    // Scale factors as outlined in the relevant portions of the user manual, please
    // see mbs/script.mbs for URL and specific page references.
    try
    {
        // Fault flag [pag. 245]
        string fault_flag = mSerial->getQuery("FF");
        unsigned char fault = boost::lexical_cast<unsigned int>(fault_flag);
        memcpy(&_fault, &fault, sizeof(fault));
        // Status flag [pag. 247]
        string status_flag = mSerial->getQuery("FS");
        unsigned char status = boost::lexical_cast<unsigned int>(status_flag);
        memcpy(&_flag, &status, sizeof(status));
        // power supply voltage
        string supply_voltage_1 = mSerial->getQuery("V", "1");
        _volts_internal = boost::lexical_cast<double>(supply_voltage_1) / 10;
        string supply_voltage_3 = mSerial->getQuery("V", "3");
        _volts_five = boost::lexical_cast<double>(supply_voltage_3) / 1000;
        // temperature channels [pag. 259]
        string temperature_1 = mSerial->getQuery("T", "1");
        _temp_mcu = boost::lexical_cast<double>(temperature_1);
        string temperature_2 = mSerial->getQuery("T", "2");
        _temp_bridge = boost::lexical_cast<double>(temperature_2);
    }
    catch (std::bad_cast& e)
    {
      ROS_WARN("Failure parsing feedback data. Dropping message.");
      return;
    }

    // Force update all diagnostic parts
    diagnostic_updater.force_update();
}

void Roboteq::read(const ros::Time& time, const ros::Duration& period) {
    //ROS_DEBUG_STREAM("Get measure from Roboteq");

    std::vector<std::string> motors[mMotor.size()];
    std::vector<std::string> fields;

    // Read status motor
    // motor status flags [pag. 246]
    string str_status = mSerial->getQuery("FM");
    // ROS_INFO_STREAM("FM=" << str_status);
    boost::split(fields, str_status, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // motor command [pag. 250]
    string str_motor = mSerial->getQuery("M");
    // ROS_INFO_STREAM("M =" << str_motor);
    boost::split(fields, str_motor, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // motor feedback [pag. 244]
    string str_feedback = mSerial->getQuery("F");
    //ROS_INFO_STREAM("F =" << str_feedback);
    boost::split(fields, str_feedback, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // motor loop error [pag. 244]
    string str_loop_error = mSerial->getQuery("E");
    // ROS_INFO_STREAM("E =" << str_loop_error);
    boost::split(fields, str_loop_error, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // motor power [pag. 255]
    string str_motor_power = mSerial->getQuery("P");
    // ROS_INFO_STREAM("P =" << str_motor_power);
    boost::split(fields, str_motor_power, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // power supply voltage [pag. 262]
    string str_voltage_supply = mSerial->getQuery("V", "2");
    //ROS_INFO_STREAM("V2=" << str_voltage_supply);
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(str_voltage_supply);
    }
    // motor Amps [pag. 230]
    string str_motor_amps = mSerial->getQuery("A");
    //ROS_INFO_STREAM("A =" << str_motor_amps);
    boost::split(fields, str_motor_amps, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // motor battery amps [pag. 233]
    string str_motor_battery_amps = mSerial->getQuery("BA");
    //ROS_INFO_STREAM("BA=" << str_motor_battery_amps);
    boost::split(fields, str_motor_battery_amps, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // position encoder value [pag. 236]
    string str_position_encoder_absolute = mSerial->getQuery("C");
    //ROS_INFO_STREAM("C =" << str_position_encoder_absolute);
    boost::split(fields, str_position_encoder_absolute, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // motor track [pag. 260]
    string str_motor_track = mSerial->getQuery("TR");
    //ROS_INFO_STREAM("TR=" << str_motor_track);
    boost::split(fields, str_motor_track, boost::algorithm::is_any_of(":"));
    for(int i = 0; i < fields.size(); ++i) {
        motors[i].push_back(fields[i]);
    }
    // send list
    for(int i = 0; i < mMotor.size(); ++i) {
        //get number motor initialization
        unsigned int idx = mMotor[i]->mNumber-1;
        // Read and decode vector
        mMotor[i]->readVector(motors[idx]);
    }

    // Read data from GPIO
    if(_isGPIOreading)
    {
        msg_peripheral.header.stamp = ros::Time::now();
        std::vector<std::string> fields;
        // Get Pulse in status [pag. 256]
        string pulse_in = mSerial->getQuery("PI");
        boost::split(fields, pulse_in, boost::algorithm::is_any_of(":"));
        // Clear msg list
        msg_peripheral.pulse_in.clear();
        for(int i = 0; i < fields.size(); ++i)
        {
            try
            {
                msg_peripheral.pulse_in.push_back(boost::lexical_cast<unsigned int>(fields[i]));
            }
            catch (std::bad_cast& e)
            {
                msg_peripheral.pulse_in.push_back(0);
            }
        }
        // Get analog input values [pag. 231]
        string analog = mSerial->getQuery("AI");
        boost::split(fields, analog, boost::algorithm::is_any_of(":"));
        // Clear msg list
        msg_peripheral.analog.clear();
        for(int i = 0; i < fields.size(); ++i)
        {
            try
            {
                msg_peripheral.analog.push_back(boost::lexical_cast<double>(fields[i]) / 1000.0);
            }
            catch (std::bad_cast& e)
            {
                msg_peripheral.analog.push_back(0);
            }
        }

        // Get Digital input values [pag. 242]
        string digital_in = mSerial->getQuery("DI");
        boost::split(fields, digital_in, boost::algorithm::is_any_of(":"));
        // Clear msg list
        msg_peripheral.digital_in.clear();
        for(int i = 0; i < fields.size(); ++i)
        {
            try
            {
                msg_peripheral.digital_in.push_back(boost::lexical_cast<unsigned int>(fields[i]));
            }
            catch (std::bad_cast& e)
            {
                msg_peripheral.digital_in.push_back(0);
            }
        }

        string digital_out = mSerial->getQuery("DO");
        unsigned int num = 0;
        try
        {
            num = boost::lexical_cast<unsigned int>(digital_out);
        }
        catch (std::bad_cast& e)
        {
            num = 0;
        }
        int mask = 0x0;
        // Clear msg list
        msg_peripheral.digital_out.clear();
        for(int i = 0; i < 8; ++i)
        {
            msg_peripheral.digital_out.push_back((mask & num));
            mask <<= 1;
        }

        // Send GPIO status
        pub_peripheral.publish(msg_peripheral);
    }
}

void Roboteq::write(const ros::Time& time, const ros::Duration& period) {
    //ROS_DEBUG_STREAM("Write command to Roboteq");

    for (vector<Motor*>::iterator it = mMotor.begin() ; it != mMotor.end(); ++it)
    {
        Motor* motor = ((Motor*)(*it));
        // Launch initialization motors
        motor->writeCommandsToHardware(period);
        ROS_DEBUG_STREAM("Motor [" << motor->getName() << "] Send commands");
    }
}

bool Roboteq::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    ROS_INFO_STREAM("Prepare to switch!");
    return true;
}

void Roboteq::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    // Stop all controller in list
    for(std::list<hardware_interface::ControllerInfo>::const_iterator it = stop_list.begin(); it != stop_list.end(); ++it)
    {
        //ROS_INFO_STREAM("DO SWITCH STOP name: " << it->name << " - type: " << it->type);
        const hardware_interface::InterfaceResources& iface_res = it->claimed_resources.front();
        for (std::set<std::string>::const_iterator res_it = iface_res.resources.begin(); res_it != iface_res.resources.end(); ++res_it)
        {
            ROS_INFO_STREAM(it->name << "[" << *res_it << "] STOP");

            for (vector<Motor*>::iterator m_it = mMotor.begin() ; m_it != mMotor.end(); ++m_it)
            {
                Motor* motor = ((Motor*)(*m_it));
                if(motor->getName().compare(*res_it) == 0)
                {
                    // switch initialization motors
                    motor->switchController("disable");
                    break;
                }
            }
        }
    }
    // Stop motors
    mSerial->command("EX");
    // Run all new controllers
    for(std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
    {
        //ROS_INFO_STREAM("DO SWITCH START name: " << it->name << " - type: " << it->type);
        const hardware_interface::InterfaceResources& iface_res = it->claimed_resources.front();
        for (std::set<std::string>::const_iterator res_it = iface_res.resources.begin(); res_it != iface_res.resources.end(); ++res_it)
        {
            ROS_INFO_STREAM(it->name << "[" << *res_it << "] START");

            for (vector<Motor*>::iterator m_it = mMotor.begin() ; m_it != mMotor.end(); ++m_it)
            {
                Motor* motor = ((Motor*)(*m_it));
                if(motor->getName().compare(*res_it) == 0)
                {
                    // switch initialization motors
                    motor->switchController(it->type);
                    break;
                }
            }
        }
    }
    // Enable motor
    mSerial->command("MG");
}

void Roboteq::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    stat.add("Type board", _type);
    stat.add("Name board", _model);
    stat.add("Version", _version);
    stat.add("UID", _uid);

    stat.add("Temp MCU (deg)", _temp_mcu);
    stat.add("Temp Bridge (deg)", _temp_bridge);

    stat.add("Internal (V)", _volts_internal);
    stat.add("5v regulator (V)", _volts_five);

    string mode = "[ ";
    if(_flag.serial_mode)
        mode += "serial ";
    if(_flag.pulse_mode)
        mode += "pulse ";
    if(_flag.analog_mode)
        mode += "analog ";
    mode += "]";
    // Mode roboteq board
    stat.add("Mode", mode);
    // Spectrum
    stat.add("Spectrum", (bool)_flag.spectrum);
    // Microbasic
    stat.add("Micro basic running", (bool)_flag.microbasic_running);

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Board ready!");

    if(_flag.power_stage_off)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Power stage OFF");
    }

    if(_flag.stall_detect)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Stall detect");
    }

    if(_flag.at_limit)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "At limit");
    }

    if(_fault.brushless_sensor_fault)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Brushless sensor fault");
    }

    if(_fault.emergency_stop)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Emergency stop");
    }

    if(_fault.mosfet_failure)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "MOSFET failure");
    }

    if(_fault.overheat)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Over heat MCU=%f, Bridge=%f", _temp_mcu, _temp_bridge);
    }

    if(_fault.overvoltage)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Over voltage");
    }

    if(_fault.undervoltage)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Under voltage");
    }

    if(_fault.short_circuit)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Short circuit");
    }
}

void Roboteq::getControllerFromRoboteq()
{
    try
    {
        // Get PWM frequency PWMF [pag. 327]
        string str_pwm = mSerial->getParam("PWMF");
        unsigned int tmp_pwm = boost::lexical_cast<unsigned int>(str_pwm);
        double pwm = ((double) tmp_pwm) / 10.0;
        // Set params
        private_mNh.setParam("pwm_frequency", pwm);

        // Get over voltage limit OVL [pag. 326]
        string str_ovl = mSerial->getParam("OVL");
        unsigned int tmp_ovl = boost::lexical_cast<unsigned int>(str_ovl);
        double ovl = ((double) tmp_ovl) / 10.0;
        // Set params
        private_mNh.setParam("over_voltage_limit", ovl);

        // Get over voltage hystersis OVH [pag. 326]
        string str_ovh = mSerial->getParam("OVH");
        unsigned int tmp_ovh = boost::lexical_cast<unsigned int>(str_ovh);
        double ovh = ((double) tmp_ovh) / 10.0;
        // Set params
        private_mNh.setParam("over_voltage_hysteresis", ovh);

        // Get under voltage limit UVL [pag. 328]
        string str_uvl = mSerial->getParam("UVL");
        unsigned int tmp_uvl = boost::lexical_cast<unsigned int>(str_uvl);
        double uvl = ((double) tmp_uvl) / 10.0;
        // Set params
        private_mNh.setParam("under_voltage_limit", uvl);

        // Get brake activation delay BKD [pag. 309]
        string str_break = mSerial->getParam("BKD");
        int break_delay = boost::lexical_cast<unsigned int>(str_break);
        // Set params
        private_mNh.setParam("break_delay", break_delay);

        // Get Mixing mode MXMD [pag. 322]
        string str_mxd = mSerial->getParam("MXMD", "1");
        // ROS_INFO_STREAM("MXMD "<< str_mxd);
        int mixed = boost::lexical_cast<unsigned int>(str_mxd);
        // Set params
        private_mNh.setParam("mixing", mixed);

    } catch (std::bad_cast& e)
    {
        ROS_WARN_STREAM("Failure parsing feedback data. Dropping message." << e.what());
    }
}

void Roboteq::reconfigureCBController(roboteq_control::RoboteqControllerConfig &config, uint32_t level)
{
    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_controller)
    {
      _last_controller_config = config;
      default_controller_config = _last_controller_config;
      setup_controller = true;
      return;
    }

    if(config.restore_defaults)
    {
        //if someone sets restore defaults on the parameter server, prevent looping
        config.restore_defaults = false;
        // Overload config with default
        config = default_controller_config;
    }

    if(config.factory_reset)
    {
        //if someone sets restore defaults on the parameter server, prevent looping
        config.factory_reset = false;
        mSerial->factoryReset();
        // Enable load from roboteq board
        config.load_roboteq = true;
    }

    if(config.load_from_eeprom)
    {
        //if someone sets again the request on the parameter server, prevent looping
        config.load_from_eeprom = false;
        mSerial->loadFromEEPROM();
        // Enable load from roboteq board
        config.load_roboteq = true;
    }

    if(config.load_roboteq)
    {
        //if someone sets again the request on the parameter server, prevent looping
        config.load_roboteq = false;
        // Launch param load
        getControllerFromRoboteq();
        // Skip other read
        return;
    }

    if(config.store_in_eeprom)
    {
        //if someone sets again the request on the parameter server, prevent looping
        config.store_in_eeprom = false;
        // Save all data in eeprom
        mSerial->saveInEEPROM();
    }

    // Set PWM frequency PWMF [pag. 327]
    if(_last_controller_config.pwm_frequency != config.pwm_frequency)
    {
        // Update PWM
        int pwm = config.pwm_frequency * 10;
        mSerial->setParam("PWMF", std::to_string(pwm));
    }
    // Set over voltage limit OVL [pag. 326]
    if(_last_controller_config.over_voltage_limit != config.over_voltage_limit)
    {
        // Update over voltage limit
        int ovl = config.over_voltage_limit * 10;
        mSerial->setParam("OVL", std::to_string(ovl));
    }
    // Set over voltage hystersis OVH [pag. 326]
    if(_last_controller_config.over_voltage_hysteresis != config.over_voltage_hysteresis)
    {
        // Update over voltage hysteresis
        int ovh = config.over_voltage_hysteresis * 10;
        mSerial->setParam("OVH", std::to_string(ovh));
    }
    // Set under voltage limit UVL [pag. 328]
    if(_last_controller_config.under_voltage_limit != config.under_voltage_limit)
    {
        // Update under voltage limit
        int uvl = config.under_voltage_limit * 10;
        mSerial->setParam("UVL", std::to_string(uvl));
    }
    // Set brake activation delay BKD [pag. 309]
    if(_last_controller_config.break_delay != config.break_delay)
    {
        // Update brake activation delay
        mSerial->setParam("BKD", std::to_string(config.break_delay));
    }

    // Set Mixing mode MXMD [pag. 322]
    if(_last_controller_config.mixing != config.mixing)
    {
        // Update brake activation delay
        mSerial->setParam("MXMD", std::to_string(config.mixing) + ":0");
    }

    // Update last configuration
    _last_controller_config = config;
}

bool Roboteq::service_Callback(roboteq_control::Service::Request &req, roboteq_control::Service::Response &msg) {
    // Convert to lower case
    std::transform(req.service.begin(), req.service.end(), req.service.begin(), ::tolower);
    //ROS_INFO_STREAM("Name request: " << req.service);
    if(req.service.compare("info") == 0)
    {
        msg.information = "\nBoard type: " + _type + "\n"
                          "Name board: " + _model + "\n"
                          "Version: " + _version + "\n"
                          "UID: " + _uid + "\n";
    }
    else if(req.service.compare("reset") == 0)
    {
        // Launch reset command
        mSerial->reset();
        // return message
        msg.information = "System reset";
    }
    else if(req.service.compare("save") == 0)
    {
        // Launch reset command
        mSerial->saveInEEPROM();
        // return message
        msg.information = "Parameters saved";
    }
    else
    {
        msg.information = "\nList of commands availabes: \n"
                          "* info      - information about this board \n"
                          "* reset     - " + _model + " board software reset\n"
                          "* save      - Save all paramters in EEPROM \n"
                          "* help      - this help.";
    }
    return true;
}

}
