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

#include "configurator/motor_pid.h"

MotorPIDConfigurator::MotorPIDConfigurator(const ros::NodeHandle& nh, roboteq::serial_controller *serial, string path, string name, unsigned int number)
    : nh_(nh)
    , mSerial(serial)
{
    // Find path param
    mName = nh_.getNamespace() + "/" + path + "/pid/" + name;
    mType = name;
    ROS_DEBUG_STREAM("Param " << path + "/pid/" + name << " has " << mName << " N:" << number);
    // Roboteq motor number
    mNumber = number;
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
    mDynRecServer = std::make_shared<ReconfigureServer>(mDynServerMutex, ros::NodeHandle(mName));
    // Load default configuration
    roboteq_control::RoboteqPIDConfig config;
    mDynRecServer->getConfigDefault(config);
    // Update parameters
    mDynServerMutex.lock();
    mDynRecServer->updateConfig(config);
    mDynServerMutex.unlock();
    // Set callback
    mDynRecServer->setCallback(boost::bind(&MotorPIDConfigurator::reconfigureCBPID, this, _1, _2));
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
        nh_.setParam(mName + "/Kp", kp);

        // Get KI gain = ki / 10 [pag 318]
        string str_ki = mSerial->getParam("KI", std::to_string(mNumber));
        unsigned int tmp_ki = boost::lexical_cast<unsigned int>(str_ki);
        double ki = ((double) tmp_ki) / 10.0;
        // Set params
        nh_.setParam(mName + "/Ki", ki);

        // Get KD gain = kd / 10 [pag 317]
        string str_kd = mSerial->getParam("KD", std::to_string(mNumber));
        unsigned int tmp_kd = boost::lexical_cast<unsigned int>(str_kd);
        double kd = ((double) tmp_kd) / 10.0;
        // Set params
        nh_.setParam(mName + "/Kd", kd);

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

void MotorPIDConfigurator::setPIDconfiguration()
{
    // Set Position velocity
    int pos_vel;
    // Set params
    nh_.getParam(mName + "/position_mode_velocity", pos_vel);
    // Update position velocity
    mSerial->setParam("MVEL", std::to_string(mNumber) + " " + std::to_string(pos_vel));

    // Set number of turn between limits
    double mxtrn;
    // Set params
    nh_.getParam(mName + "/turn_min_to_max", mxtrn);
    // Update position velocity
    mSerial->setParam("MXTRN", std::to_string(mNumber) + " " + std::to_string(mxtrn * 100));

    // Set KP gain = kp * 10
    double kp;
    // Set params
    nh_.getParam(mName + "/Kp", kp);
    // Update gain position
    mSerial->setParam("KP", std::to_string(mNumber) + " " + std::to_string(kp * 10));

    // Set KI gain = ki * 10
    double ki;
    // Set params
    nh_.getParam(mName + "/Ki", ki);
    // Set KI parameter
    mSerial->setParam("KI", std::to_string(mNumber) + " " + std::to_string(ki * 10));

    // Set KD gain = kd * 10
    double kd;
    // Set params
    nh_.getParam(mName + "/Kd", kd);
    // Set KD parameter
    mSerial->setParam("KD", std::to_string(mNumber) + " " + std::to_string(kd * 10));

    // Set Integral cap
    int icap;
    // Set params
    nh_.getParam(mName + "/integrator_limit", icap);
    // Update integral cap
    mSerial->setParam("ICAP", std::to_string(mNumber) + " " + std::to_string(icap));

    // Set closed loop error detection
    int clerd;
    // Set params
    nh_.getParam(mName + "/loop_error_detection", clerd);
    // Update integral cap
    mSerial->setParam("CLERD", std::to_string(mNumber) + " " + std::to_string(clerd));
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

    // Check type if the PID selected is right
    // Operative mode reference in [pag 314]
    string str_mode = mSerial->getParam("MMOD", std::to_string(mNumber));
    // Get sign from roboteq board
    int mode = boost::lexical_cast<int>(str_mode);
    // Check if the roboteq board is set in right configuration for this PID controller
    bool check1 = (mType.compare("velocity") == 0 && ((mode == 1) || (mode == 6)));
    bool check2 = (mType.compare("position") == 0 && ((mode == 2) || (mode == 3) || (mode == 4)));
    bool check3 = (mType.compare("torque") == 0 && ((mode == 5)));
    bool status = check1 | check2 | check3;
    // ROS_INFO_STREAM("[" << mode << "]" << mType << " ck1:" << check1 << " ck2:" << check2 << " ck3:" << check3 << " status:" << (status ? "true" : "false"));
    if(!status)
    {
        // Restore old configuration
        config = _last_pid_config;
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

    // Set Position velocity
    if(_last_pid_config.position_mode_velocity != config.position_mode_velocity)
    {
        // Update position velocity
        mSerial->setParam("MVEL", std::to_string(mNumber) + " " + std::to_string(config.position_mode_velocity));
    }
    // Set number of turn between limits
    if(_last_pid_config.turn_min_to_max != config.turn_min_to_max)
    {
        // Update position velocity
        int gain = config.turn_min_to_max * 100;
        mSerial->setParam("MXTRN", std::to_string(mNumber) + " " + std::to_string(gain));
    }
    // Set KP gain = kp * 10
    if(_last_pid_config.Kp != config.Kp)
    {
        // Update gain
        int gain = config.Kp * 10;
        mSerial->setParam("KP", std::to_string(mNumber) + " " + std::to_string(gain));
    }
    // Set KI gain = ki * 10
    if(_last_pid_config.Ki != config.Ki)
    {
        // Update gain
        int gain = config.Ki * 10;
        mSerial->setParam("KI", std::to_string(mNumber) + " " + std::to_string(gain));
    }
    // Set KD gain = kd * 10
    if(_last_pid_config.Kd != config.Kd)
    {
        // Update gain
        int gain = config.Kd * 10;
        mSerial->setParam("KD", std::to_string(mNumber) + " " + std::to_string(gain));
    }

    // Set Integral cap
    if(_last_pid_config.integrator_limit != config.integrator_limit)
    {
        // Update integral cap
        mSerial->setParam("ICAP", std::to_string(mNumber) + " " + std::to_string(config.integrator_limit));
    }
    // Set closed loop error detection
    if(_last_pid_config.loop_error_detection != config.loop_error_detection)
    {
        // Update integral cap
        mSerial->setParam("CLERD", std::to_string(mNumber) + " " + std::to_string(config.loop_error_detection));
    }


    // Update last configuration
    _last_pid_config = config;
}
