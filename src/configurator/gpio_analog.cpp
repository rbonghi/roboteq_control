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

#include "configurator/gpio_analog.h"

GPIOAnalogConfigurator::GPIOAnalogConfigurator(const ros::NodeHandle &nh, roboteq::serial_controller *serial, unsigned int number)
    : nh_(nh)
    , mSerial(serial)
{
    // Find path param
    mName = nh_.getNamespace() + "/analog/" + std::to_string(number);
    // Roboteq motor number
    mNumber = number;
    // Set false on first run
    setup_param = false;
}

void GPIOAnalogConfigurator::initConfigurator(bool load_from_board)
{
    // Check if is required load paramers
    if(load_from_board)
    {
        // Load parameters from roboteq
        getParamFromRoboteq();
    }
    // Initialize parameter dynamic reconfigure
    ds_param = new dynamic_reconfigure::Server<roboteq_control::RoboteqAnalogInputConfig>(ros::NodeHandle(mName));
    dynamic_reconfigure::Server<roboteq_control::RoboteqAnalogInputConfig>::CallbackType cb_param = boost::bind(&GPIOAnalogConfigurator::reconfigureCBParam, this, _1, _2);
    ds_param->setCallback(cb_param);
}

void GPIOAnalogConfigurator::getParamFromRoboteq()
{
    try
    {
    } catch (std::bad_cast& e)
    {
        ROS_WARN_STREAM("Failure parsing feedback data. Dropping message." << e.what());
    }
}

void GPIOAnalogConfigurator::reconfigureCBParam(roboteq_control::RoboteqAnalogInputConfig &config, uint32_t level)
{
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
}
