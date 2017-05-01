
#include "configurator/motor_pid.h"

MotorPIDConfigurator::MotorPIDConfigurator(const ros::NodeHandle& nh, roboteq::serial_controller *serial, string path, string name, unsigned int number)
    : nh_(nh)
    , mSerial(serial)
{
    // Find path param
    mName = nh_.getNamespace() + "/" + path + "/pid/" + name;
    ROS_DEBUG_STREAM("Param " << path + "/pid/" + name << " has " << mName << " N:" << number);
    // Roboteq motor number
    mNumber = number + 1;
    // Set false on first run
    setup_pid = false;

}

void MotorPIDConfigurator::initConfigurator(bool load_from_board)
{
    // Check if is required load paramers
    if(load_from_board)
    {
        // Load parameters from roboteq
        getPIDFromRoboteq();
    }

    // Initialize parameter dynamic reconfigure
    ds_pid = new dynamic_reconfigure::Server<roboteq_control::RoboteqPIDConfig>(ros::NodeHandle(mName));
    dynamic_reconfigure::Server<roboteq_control::RoboteqPIDConfig>::CallbackType cb_pid = boost::bind(&MotorPIDConfigurator::reconfigureCBPID, this, _1, _2);
    ds_pid->setCallback(cb_pid);
}

void MotorPIDConfigurator::getPIDFromRoboteq()
{
    try
    {
        // Get Position velocity [pag. 322]
        string str_pos_vel = mSerial->getParam("MVEL", std::to_string(mNumber));
        int pos_vel = boost::lexical_cast<unsigned int>(str_pos_vel);
        // Set params
        nh_.setParam(mName + "/position_mode_velocity", pos_vel);

        // Get number of turn between limits [pag. 325]
        string str_mxtrn = mSerial->getParam("MXTRN", std::to_string(mNumber));
        unsigned int tmp_mxtrn = boost::lexical_cast<unsigned int>(str_mxtrn);
        double mxtrn = ((double) tmp_mxtrn) / 100.0;
        // Set params
        nh_.setParam(mName + "/turn_min_to_max", mxtrn);

        // Get KP gain = kp / 10 [pag 319]
        string str_kp = mSerial->getParam("KP", std::to_string(mNumber));
        unsigned int tmp_kp = boost::lexical_cast<unsigned int>(str_kp);
        double kp = ((double) tmp_kp) / 10.0;
        // Set params
        nh_.setParam(mName + "/gain_proportional", kp);

        // Get KI gain = ki / 10 [pag 318]
        string str_ki = mSerial->getParam("KI", std::to_string(mNumber));
        unsigned int tmp_ki = boost::lexical_cast<unsigned int>(str_ki);
        double ki = ((double) tmp_ki) / 10.0;
        // Set params
        nh_.setParam(mName + "/gain_integral", ki);

        // Get KD gain = kd / 10 [pag 317]
        string str_kd = mSerial->getParam("KD", std::to_string(mNumber));
        unsigned int tmp_kd = boost::lexical_cast<unsigned int>(str_kd);
        double kd = ((double) tmp_kd) / 10.0;
        // Set params
        nh_.setParam(mName + "/gain_differential", kd);

        // Get Integral cap [pag. 317]
        string str_icap = mSerial->getParam("ICAP", std::to_string(mNumber));
        int icap = boost::lexical_cast<unsigned int>(str_icap);
        // Set params
        nh_.setParam(mName + "/integrator_limit", icap);

        // Get closed loop error detection [pag. 311]
        string str_clred = mSerial->getParam("CLERD", std::to_string(mNumber));
        int clerd = boost::lexical_cast<unsigned int>(str_clred);
        // Set params
        nh_.setParam(mName + "/loop_error_detection", clerd);

    } catch (std::bad_cast& e)
    {
        ROS_WARN_STREAM("Failure parsing feedback data. Dropping message." << e.what());
    }

}

void MotorPIDConfigurator::reconfigureCBPID(roboteq_control::RoboteqPIDConfig &config, uint32_t level)
{
    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_pid)
    {
      _last_pid_config = config;
      default_pid_config = _last_pid_config;
      setup_pid = true;
      return;
    }

    if(config.restore_defaults)
    {
        //if someone sets restore defaults on the parameter server, prevent looping
        config.restore_defaults = false;
        // Overload config with default
        config = default_pid_config;
    }

    if(config.load_roboteq)
    {
        //if someone sets again the request on the parameter server, prevent looping
        config.load_roboteq = false;
        // Launch param load
        getPIDFromRoboteq();
        // Skip other read
        return;
    }

    // Set Position velocity [pag. 322]
    if(_last_pid_config.position_mode_velocity != config.position_mode_velocity)
    {
        // Update position velocity
        mSerial->setParam("MVEL", std::to_string(mNumber) + " " + std::to_string(config.position_mode_velocity));
    }
    // Set number of turn between limits [pag. 325]
    if(_last_pid_config.turn_min_to_max != config.turn_min_to_max)
    {
        // Update position velocity
        int gain = config.turn_min_to_max * 100;
        mSerial->setParam("MXTRN", std::to_string(mNumber) + " " + std::to_string(gain));
    }
    // Set KP gain = kp * 10 [pag 319]
    if(_last_pid_config.gain_proportional != config.gain_proportional)
    {
        // Update gain
        int gain = config.gain_proportional * 10;
        mSerial->setParam("KP", std::to_string(mNumber) + " " + std::to_string(gain));
    }
    // Set KI gain = ki * 10 [pag 318]
    if(_last_pid_config.gain_integral != config.gain_integral)
    {
        // Update gain
        int gain = config.gain_integral * 10;
        mSerial->setParam("KI", std::to_string(mNumber) + " " + std::to_string(gain));
    }
    // Set KD gain = kd * 10 [pag 317]
    if(_last_pid_config.gain_differential != config.gain_differential)
    {
        // Update gain
        int gain = config.gain_differential * 10;
        mSerial->setParam("KD", std::to_string(mNumber) + " " + std::to_string(gain));
    }

    // Set Integral cap [pag. 317]
    if(_last_pid_config.integrator_limit != config.integrator_limit)
    {
        // Update integral cap
        mSerial->setParam("ICAP", std::to_string(mNumber) + " " + std::to_string(config.integrator_limit));
    }
    // Set closed loop error detection [pag. 311]
    if(_last_pid_config.loop_error_detection != config.loop_error_detection)
    {
        // Update integral cap
        mSerial->setParam("CLERD", std::to_string(mNumber) + " " + std::to_string(config.loop_error_detection));
    }

}
