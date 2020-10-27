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

#include "configurator/motor_param.h"

MotorParamConfigurator::MotorParamConfigurator(const ros::NodeHandle& nh, roboteq::serial_controller *serial, std::string name, unsigned int number)
    : nh_(nh)
    , mSerial(serial)
{
    // Find path param
    mName = nh_.getNamespace() + "/" + name;
    // Roboteq motor number
    mNumber = number;
    // Set false on first run
    setup_param = false;
}

void MotorParamConfigurator::initConfigurator(bool load_from_board)
{
    double ratio;
    // Check if exist ratio variable
    if(nh_.hasParam(mName + "/ratio"))
    {
        double temp_double;
        nh_.getParam(mName + "/ratio", temp_double);
        // Set Ratio
        ratio = temp_double;
    } else
    {
        nh_.setParam(mName + "/ratio", 1.0);
        // Set Ratio
        ratio = 1.0;
        // Send alter ratio value
        ROS_WARN_STREAM("Default Ratio is " << ratio);
    }

    // Check if is required load paramers
    if(load_from_board)
    {
        // Load parameters from roboteq
        getParamFromRoboteq();
    }

    // Initialize parameter dynamic reconfigure
    mDynRecServer_param = std::make_shared<ReconfigureServerParam>(mDynServerMutex_param, ros::NodeHandle(mName));
    // Load default configuration
    roboteq_control::RoboteqParameterConfig config_param;
    mDynRecServer_param->getConfigDefault(config_param);
    // Update parameters
    mDynServerMutex_param.lock();
    mDynRecServer_param->updateConfig(config_param);
    mDynServerMutex_param.unlock();
    // Set callback
    mDynRecServer_param->setCallback(boost::bind(&MotorParamConfigurator::reconfigureCBParam, this, _1, _2));

    // Initialize pid type dynamic reconfigure
    mDynRecServer_pid = std::make_shared<ReconfigureServerPID>(mDynServerMutex_pid, ros::NodeHandle(mName + "/pid"));
    // Load default configuration
    roboteq_control::RoboteqPIDtypeConfig config_pid;
    mDynRecServer_pid->getConfigDefault(config_pid);
    // Update parameters
    mDynServerMutex_pid.lock();
    mDynRecServer_pid->updateConfig(config_pid);
    mDynServerMutex_pid.unlock();
    // Set callback
    mDynRecServer_pid->setCallback(boost::bind(&MotorParamConfigurator::reconfigureCBPIDtype, this, _1, _2));
}

void MotorParamConfigurator::setOperativeMode(int type)
{
    // Update operative mode
    mSerial->setParam("MMOD", std::to_string(mNumber) + " " + std::to_string(type));
}

int MotorParamConfigurator::getOperativeMode()
{
    // Operative mode reference in [pag 321]
    string str_mode = mSerial->getParam("MMOD", std::to_string(mNumber));
    // Get sign from roboteq board
    int mode = boost::lexical_cast<int>(str_mode);
    // Set parameter
    // nh_.setParam(mName + "/operating_mode", mode);

    return mode;
}

void MotorParamConfigurator::reconfigureCBPIDtype(roboteq_control::RoboteqPIDtypeConfig &config, uint32_t level)
{
    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_pid_type)
    {
      _last_pid_type_config = config;
      default_pid_type_config = _last_pid_type_config;
      setup_pid_type = true;
      return;
    }

    if(config.restore_defaults)
    {
        //if someone sets restore defaults on the parameter server, prevent looping
        config.restore_defaults = false;
        // Overload config with default
        config = default_pid_type_config;
    }
}

void MotorParamConfigurator::getParamFromRoboteq()
{
    try
    {
        // Load Ratio
        double ratio;
        nh_.getParam(mName + "/ratio", ratio);

        // Motor direction {1 (Clockwise), -1 (Underclockwise)}
        string str_mdir = mSerial->getParam("MDIR", std::to_string(mNumber));
        // Get sign from roboteq board
        int sign = boost::lexical_cast<int>(str_mdir) ? -1 : 1;
        // Set parameter
        nh_.setParam(mName + "/rotation", sign);

        // Stall detection
        string str_stall = mSerial->getParam("BLSTD", std::to_string(mNumber));
        int stall = boost::lexical_cast<int>(str_stall);
        // Set params
        nh_.setParam(mName + "/stall_detection", stall);

        // Get Max Amper limit = alim / 10
        string str_alim = mSerial->getParam("ALIM", std::to_string(mNumber));
        unsigned int tmp = boost::lexical_cast<unsigned int>(str_alim);
        double alim = ((double) tmp) / 10.0;
        // Set params
        nh_.setParam(mName + "/amper_limit", alim);

        // Max power forward
        string str_max_fw = mSerial->getParam("MXPF", std::to_string(mNumber));
        // Get max forward
        int max_forward = boost::lexical_cast<unsigned int>(str_max_fw);
        // Set parameter
        nh_.setParam(mName + "/max_acceleration", max_forward);

        // Max power forward reverse
        string str_max_re = mSerial->getParam("MXPR", std::to_string(mNumber));
        // Get max reverse
        int max_reverse = boost::lexical_cast<unsigned int>(str_max_re);
        // Set parameter
        nh_.setParam(mName + "/max_deceleration", max_reverse);

        // Get Max RPM motor
        string str_rpm_motor = mSerial->getParam("MXRPM", std::to_string(mNumber));
        // Get RPM from board
        unsigned int rpm_motor = boost::lexical_cast<unsigned int>(str_rpm_motor);
        // Convert in max RPM
        double max_rpm = ((double) rpm_motor) / ratio;
        // Set parameter
        nh_.setParam(mName + "/max_speed", max_rpm);

        // Get Max RPM acceleration rate
        string str_rpm_acceleration_motor = mSerial->getParam("MAC", std::to_string(mNumber));
        // Get RPM from board
        unsigned int rpm_acceleration_motor = boost::lexical_cast<unsigned int>(str_rpm_acceleration_motor);
        // Convert in max RPM
        double rpm_acceleration = ((double) rpm_acceleration_motor) / ratio;
        // Set parameter
        nh_.setParam(mName + "/max_acceleration", rpm_acceleration);

        // Get Max RPM deceleration rate
        string str_rpm_deceleration_motor = mSerial->getParam("MDEC", std::to_string(mNumber));
        // Get RPM from board
        unsigned int rpm_deceleration_motor = boost::lexical_cast<unsigned int>(str_rpm_deceleration_motor);
        // Convert in max RPM
        double rpm_deceleration = ((double) rpm_deceleration_motor) / ratio;
        // Set parameter
        nh_.setParam(mName + "/max_deceleration", rpm_deceleration);

    } catch (std::bad_cast& e)
    {
        ROS_WARN_STREAM("Failure parsing feedback data. Dropping message." << e.what());
    }
}

void MotorParamConfigurator::reconfigureCBParam(roboteq_control::RoboteqParameterConfig &config, uint32_t level) {

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_param)
    {
      _last_param_config = config;
      default_param_config = _last_param_config;
      setup_param = true;
      return;
    }

    if(config.restore_defaults)
    {
        //if someone sets restore defaults on the parameter server, prevent looping
        config.restore_defaults = false;
        // Overload config with default
        config = default_param_config;
    }

    if(config.load_roboteq)
    {
        //if someone sets again the request on the parameter server, prevent looping
        config.load_roboteq = false;
        // Launch param load
        getParamFromRoboteq();
        // Skip other read
        return;
    }

    // Update ratio
    // Get old max speed, acceleration and deceleartion and evaluate new equivalent value
    if(_last_param_config.ratio != config.ratio)
    {
        // Get Max RPM motor
        string str_rpm_motor = mSerial->getParam("MXRPM", std::to_string(mNumber));
        // Get RPM from board
        unsigned int rpm_motor = boost::lexical_cast<unsigned int>(str_rpm_motor);
        // Update with new max speed
        config.max_speed = ((double) rpm_motor) / config.ratio;

        // Get Max RPM acceleration rate
        string str_rpm_acceleration_motor = mSerial->getParam("MAC", std::to_string(mNumber));
        // Get RPM from board
        unsigned int rpm_acceleration_motor = boost::lexical_cast<unsigned int>(str_rpm_acceleration_motor);
        // Convert in max RPM
        config.max_acceleration = ((double) rpm_acceleration_motor) / config.ratio;

        // Get Max RPM deceleration rate
        string str_rpm_deceleration_motor = mSerial->getParam("MDEC", std::to_string(mNumber));
        // Get RPM from board
        unsigned int rpm_deceleration_motor = boost::lexical_cast<unsigned int>(str_rpm_deceleration_motor);
        // Convert in max RPM
        config.max_deceleration = ((double) rpm_deceleration_motor) / config.ratio;
    }

    if(_last_param_config.rotation != config.rotation)
    {
        // Update direction
        int direction = (config.rotation == -1) ? 1 : 0;
        mSerial->setParam("MDIR", std::to_string(mNumber) + " " + std::to_string(direction));
    }
    // Stall detection [pag. 310]
    if(_last_param_config.stall_detection != config.stall_detection)
    {
        // Update stall detection value
        mSerial->setParam("BLSTD", std::to_string(mNumber) + " " + std::to_string(config.stall_detection));
    }
    // Get Max Amper limit = alim * 10 [pag 306]
    if(_last_param_config.amper_limit != config.amper_limit)
    {
        // Update stall detection value
        int alim = config.amper_limit * 10;
        mSerial->setParam("ALIM", std::to_string(mNumber) + " " + std::to_string(alim));
    }
    // Max power forward [pag. 323]
    if(_last_param_config.max_forward != config.max_forward)
    {
        // Update max forward
        mSerial->setParam("MXPF", std::to_string(mNumber) + " " + std::to_string(config.max_forward));
    }
    // Max power forward reverse [pag. 324]
    if(_last_param_config.max_forward != config.max_reverse)
    {
        // Update max forward reverse
        mSerial->setParam("MXPR", std::to_string(mNumber) + " " + std::to_string(config.max_reverse));
    }

    // Set Max RPM motor
    if(_last_param_config.max_speed != config.max_speed)
    {
        // Update max RPM motor
        long int max_speed_motor = config.ratio * config.max_speed;
        mSerial->setParam("MXRPM", std::to_string(mNumber) + " " + std::to_string(max_speed_motor));
    }

    // Set Max RPM acceleration rate
    if(_last_param_config.max_acceleration != config.max_acceleration)
    {
        // Update max acceleration RPM/s motor
        long int max_acceleration_motor = config.ratio * config.max_acceleration;
        mSerial->setParam("MAC", std::to_string(mNumber) + " " + std::to_string(max_acceleration_motor));
    }
    // Set Max RPM deceleration rate
    if(_last_param_config.max_deceleration != config.max_deceleration)
    {
        // Update max deceleration RPM/s motor
        long int max_deceleration_motor = config.ratio * config.max_deceleration;
        mSerial->setParam("MDEC", std::to_string(mNumber) + " " + std::to_string(max_deceleration_motor));
    }

    // Update last configuration
    _last_param_config = config;
}
