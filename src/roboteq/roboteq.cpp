

#include "roboteq/roboteq.h"

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
    // store the script version
    _script_ver = mSerial->getVersionScript();
    // Load default configuration roboteq board
    getRoboteqInformation();

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
    ROS_DEBUG_STREAM("Stop motor: " << (mSerial->command("EX") ? "true" : "false"));

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
        mMotor[motor_name] = new Motor(private_mNh, serial, motor_name, number);
    }

    // Add subscriber stop
    sub_stop = private_mNh.subscribe("emergency_stop", 1, &Roboteq::stop_Callback, this);

    // Add callback
    mSerial->addCallback(&Roboteq::status, this, "S");
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
    if(mSerial->query("TRN"))
    {
        string trn = mSerial->get();
        std::vector<std::string> fields;
        boost::split(fields, trn, boost::algorithm::is_any_of(":"));
        _type = fields[0];
        _model = fields[1];
        // ROS_INFO_STREAM("Model " << _model);
    }
    // Load firmware version
    if(mSerial->query("FID"))
    {
        _version = mSerial->get();
    }
    // Load UID
    if(mSerial->query("UID"))
    {
        _uid = mSerial->get();
    }
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
    ds_controller = new dynamic_reconfigure::Server<roboteq_control::RoboteqControllerConfig>(private_mNh);
    dynamic_reconfigure::Server<roboteq_control::RoboteqControllerConfig>::CallbackType cb_controller = boost::bind(&Roboteq::reconfigureCBController, this, _1, _2);
    ds_controller->setCallback(cb_controller);

    // Initialize all motors in list
    for( map<string, Motor*>::iterator ii=mMotor.begin(); ii!=mMotor.end(); ++ii)
    {
        // Launch initialization motors
        (*ii).second->initializeMotor(_first);
        ROS_DEBUG_STREAM("Motor [" << (*ii).first << "] Initialized");
    }
    // Enable script
    mSerial->script(true);
}

void Roboteq::initializeInterfaces()
{
    // Initialize the diagnostic from the primitive object
    initializeDiagnostic();

    if (!model.initParam("/robot_description")){
        ROS_ERROR("Failed to parse urdf file");
    }
    else
    {
        ROS_INFO_STREAM("/robot_description found! " << model.name_ << " parsed!");
    }

    for( map<string, Motor*>::iterator ii=mMotor.begin(); ii!=mMotor.end(); ++ii)
    {
        /// State interface
        joint_state_interface.registerHandle(((*ii).second)->joint_state_handle);
        /// Velocity interface
        velocity_joint_interface.registerHandle(((*ii).second)->joint_handle);

        // Setup limits
        ((*ii).second)->setupLimits(model);

        // reset position joint
        double position = 0;
        ROS_DEBUG_STREAM("Motor [" << (*ii).first << "] reset position to: " << position);
        ((*ii).second)->resetPosition(position);

        //Add motor in diagnostic updater
        diagnostic_updater.add(*((*ii).second));
        ROS_DEBUG_STREAM("Motor [" << (*ii).first << "] Registered");
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
    // Force update all diagnostic parts
    diagnostic_updater.force_update();
}

void Roboteq::read(const ros::Time& time, const ros::Duration& period) {
    //ROS_DEBUG_STREAM("Get measure from Roboteq");
    for( map<string, Motor*>::iterator ii=mMotor.begin(); ii!=mMotor.end(); ++ii)
    {
        // Not required now
        // TODO
        //(*ii).second->addRequestMeasure();
        ROS_DEBUG_STREAM("Motor [" << (*ii).first << "] Request measures");
    }
}

void Roboteq::write(const ros::Time& time, const ros::Duration& period) {
    //ROS_DEBUG_STREAM("Write command to Roboteq");
    for( map<string, Motor*>::iterator ii=mMotor.begin(); ii!=mMotor.end(); ++ii)
    {
        (*ii).second->writeCommandsToHardware(period);
        ROS_DEBUG_STREAM("Motor [" << (*ii).first << "] Send commands");
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
            mMotor[*res_it]->switchController("disable");
        }
    }
    // Stop script
    mSerial->script(false);
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
            mMotor[*res_it]->switchController(it->type);
        }
    }
    // Run script
    mSerial->script(true);
    // Enable motor
    mSerial->command("MG");
}

void Roboteq::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    stat.add("Type board", _type);
    stat.add("Name board", _model);
    stat.add("Version", _version);
    stat.add("UID", _uid);

    stat.add("Script", _script_ver);

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

void Roboteq::status(string data)
{
    // Temporary plot status
    // ROS_INFO_STREAM("STATUS=" << data);

    // Split data
    std::vector<std::string> fields;
    boost::split(fields, data, boost::algorithm::is_any_of(":"));

    // Scale factors as outlined in the relevant portions of the user manual, please
    // see mbs/script.mbs for URL and specific page references.
    try
    {
        // Status fault flags status_fault_t
        unsigned char fault = boost::lexical_cast<unsigned int>(fields[0]);
        memcpy(&_fault, &fault, sizeof(fault));
        // Status flags status_flag_t
        unsigned char status = boost::lexical_cast<unsigned int>(fields[1]);
        memcpy(&_flag, &status, sizeof(status));
        // Volt controller
        _volts_internal = boost::lexical_cast<double>(fields[2]) / 10;
        _volts_five = boost::lexical_cast<double>(fields[3]) / 1000;
        // Conversion temperature MCU and bridge
        _temp_mcu = boost::lexical_cast<double>(fields[4]);
        _temp_bridge = boost::lexical_cast<double>(fields[5]);
    }
    catch (std::bad_cast& e)
    {
      ROS_WARN("Failure parsing feedback data. Dropping message.");
      return;
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

    if(config.load_roboteq)
    {
        //if someone sets again the request on the parameter server, prevent looping
        config.load_roboteq = false;
        // Launch param load
        getControllerFromRoboteq();
        // Skip other read
        return;
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

}
