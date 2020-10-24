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

#include "configurator/gpio_encoder.h"

#define PARAM_ENCODER_STRING "/encoder"

GPIOEncoderConfigurator::GPIOEncoderConfigurator(const ros::NodeHandle &nh, roboteq::serial_controller *serial, std::vector<roboteq::Motor *> motor, string name, unsigned int number)
    : nh_(nh)
    , mSerial(serial)
    ,_motor(motor)
{
    // Find path param
    mName = nh_.getNamespace() + name + PARAM_ENCODER_STRING + "/" + std::to_string(number);
    // Roboteq motor number
    mNumber = number;
    // Set false on first run
    setup_encoder = false;
}

void GPIOEncoderConfigurator::initConfigurator(bool load_from_board)
{
    // Get PPR Encoder parameter
    double ppr;
    nh_.getParam(mName + "/PPR", ppr);
    _reduction = ppr;
    // Multiply for quadrature
    _reduction *= 4;

    // Check if is required load paramers
    if(load_from_board)
    {
        // Load encoder properties from roboteq
        getEncoderFromRoboteq();
    }

    // Initialize encoder dynamic reconfigure
    mDynRecServer = std::make_shared<ReconfigureServer>(mDynServerMutex, ros::NodeHandle(mName));
    // Load default configuration
    roboteq_control::RoboteqEncoderConfig config;
    mDynRecServer->getConfigDefault(config);
    // Update parameters
    mDynServerMutex.lock();
    mDynRecServer->updateConfig(config);
    mDynServerMutex.unlock();
    // Set callback
    mDynRecServer->setCallback(boost::bind(&GPIOEncoderConfigurator::reconfigureCBEncoder, this, _1, _2));
}

double GPIOEncoderConfigurator::getConversion(double reduction) {
    // Check if exist ratio variable
    if(nh_.hasParam(mName + "/position"))
    {
        int position;
        nh_.getParam(mName + "/position", position);
        // Read position if before (1) multiply with ratio
        if(position) {
            return _reduction * reduction;
        }
    }
    return _reduction;
}

void GPIOEncoderConfigurator::getEncoderFromRoboteq() {
    try
    {
        // Get Encoder Usage - reference
        string str_emode = mSerial->getParam("EMOD", std::to_string(mNumber));
        // Get PPR from roboteq board
        int emod = boost::lexical_cast<unsigned int>(str_emode);
        // 3 modes:
        // 0 - Unsed
        // 1 - Command
        // 2 - Feedback
        int command = (emod & 0b11);
        int motors = (emod - command) >> 4;
        int tmp1 = ((motors & 0b1) > 0);
        int tmp2 = ((motors & 0b10) > 0);
        // Register reduction
        if(tmp1)
        {
            for (vector<roboteq::Motor*>::iterator it = _motor.begin() ; it != _motor.end(); ++it)
            {
                roboteq::Motor* motor = ((roboteq::Motor*)(*it));
                if(motor->getNumber() == 1)
                {
                    motor->registerSensor(this);
                    ROS_INFO_STREAM("Register encoder [" << mNumber << "] to: " << motor->getName());
                    break;
                }
            }
        }
        if(tmp2)
        {
            for (vector<roboteq::Motor*>::iterator it = _motor.begin() ; it != _motor.end(); ++it)
            {
                roboteq::Motor* motor = ((roboteq::Motor*)(*it));
                if(motor->getNumber() == 2)
                {
                    motor->registerSensor(this);
                    ROS_INFO_STREAM("Register encoder [" << mNumber << "] to: " << motor->getName());
                    break;
                }
            }
        }
        // Set parameter
        nh_.setParam(mName + "/configuration", command);
        nh_.setParam(mName + "/input_motor_one", tmp1);
        nh_.setParam(mName + "/input_motor_two", tmp2);

        // Get Encoder PPR (Pulse/rev)
        string str_ppr = mSerial->getParam("EPPR", std::to_string(mNumber));
        // Get PPR from roboteq board
        int ppr = boost::lexical_cast<unsigned int>(str_ppr);
        // Set parameter
        nh_.setParam(mName + "/PPR", ppr);

        // Get Encoder ELL - Min limit
        string str_ell = mSerial->getParam("ELL", std::to_string(mNumber));
        // Get PPR from roboteq board
        int ell = boost::lexical_cast<unsigned int>(str_ell);
        // Set parameter
        nh_.setParam(mName + "/encoder_low_count_limit", ell);

        // Get Encoder EHL - Max limit
        string str_ehl = mSerial->getParam("EHL", std::to_string(mNumber));
        // Get PPR from roboteq board
        int ehl = boost::lexical_cast<unsigned int>(str_ehl);
        // Set parameter
        nh_.setParam(mName + "/encoder_high_count_limit", ehl);

        // Get Encoder EHOME - Home count
        string str_home = mSerial->getParam("EHOME", std::to_string(mNumber));
        // Get PPR from roboteq board
        int home = boost::lexical_cast<unsigned int>(str_home);
        // Set parameter
        nh_.setParam(mName + "/encoder_home_count", home);


    } catch (std::bad_cast& e)
    {
        ROS_WARN_STREAM("Failure parsing feedback data. Dropping message." << e.what());
    }
}

void GPIOEncoderConfigurator::reconfigureCBEncoder(roboteq_control::RoboteqEncoderConfig &config, uint32_t level) {

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_encoder)
    {
      _last_encoder_config = config;
      default_encoder_config = _last_encoder_config;
      setup_encoder = true;
      return;
    }

    if(config.restore_defaults)
    {
        //if someone sets restore defaults on the parameter server, prevent looping
        config.restore_defaults = false;
        // Overload default configuration
        config = default_encoder_config;
    }

    if(config.load_roboteq)
    {
        ROS_INFO_STREAM("LOAD from Roboteq");
        //if someone sets again the request on the parameter server, prevent looping
        config.load_roboteq = false;
        // Launch encoder load
        getEncoderFromRoboteq();
        // Skip other read
        return;
    }

    // Set Encoder Usage - reference pag. 307
    if((_last_encoder_config.configuration != config.configuration) ||
            (_last_encoder_config.input_motor_one != config.input_motor_one) ||
            (_last_encoder_config.input_motor_two != config.input_motor_two))
    {
        int configuration = config.configuration + 16*config.input_motor_one + 32*config.input_motor_two;
        // Update operative mode
        mSerial->setParam("EMOD", std::to_string(mNumber) + " " + std::to_string(configuration));

        if(config.input_motor_one)
        {
            roboteq::Motor* motor = _motor.at(0);
            motor->registerSensor(this);
            ROS_INFO_STREAM("Register encoder [" << mNumber << "] to: " << motor->getName());
        }
        if(config.input_motor_two)
        {
            roboteq::Motor* motor = _motor.at(1);
            motor->registerSensor(this);
            ROS_INFO_STREAM("Register encoder [" << mNumber << "] to: " << motor->getName());
        }
    }
    // Set Encoder PPR [pag. 308]
    if(_last_encoder_config.PPR != config.PPR)
    {
        // Update reduction value
        _reduction = config.PPR;
        // Update operative mode
        mSerial->setParam("EPPR", std::to_string(mNumber) + " " + std::to_string(config.PPR));
    }
    // Set Encoder ELL - Min limit [pag. 306]
    if(_last_encoder_config.encoder_low_count_limit != config.encoder_low_count_limit)
    {
        // Update operative mode
        mSerial->setParam("ELL", std::to_string(mNumber) + " " + std::to_string(config.encoder_low_count_limit));
    }
    // Set Encoder EHL - Max limit [pag. 304]
    if(_last_encoder_config.encoder_high_count_limit != config.encoder_high_count_limit)
    {
        // Update operative mode
        mSerial->setParam("EHL", std::to_string(mNumber) + " " + std::to_string(config.encoder_high_count_limit));
    }
    // Set Encoder EHOME - Home count [pag. 306]
    if(_last_encoder_config.encoder_home_count != config.encoder_home_count)
    {
        // Update operative mode
        mSerial->setParam("EHOME", std::to_string(mNumber) + " " + std::to_string(config.encoder_home_count));
    }

    // Update last configuration
    _last_encoder_config = config;

}
