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

#ifndef GPIOPARAMCONFIGURATOR_H
#define GPIOPARAMCONFIGURATOR_H

#include <ros/ros.h>

#include <roboteq_control/RoboteqParameterConfig.h>
#include <roboteq_control/RoboteqEncoderConfig.h>
#include <roboteq_control/RoboteqPIDtypeConfig.h>
#include <dynamic_reconfigure/server.h>

#include "roboteq/serial_controller.h"

class MotorParamConfigurator
{
public:
    /**
     * @brief MotorParamConfigurator
     * @param nh
     * @param serial
     * @param name
     * @param number
     */
    MotorParamConfigurator(const ros::NodeHandle& nh, roboteq::serial_controller *serial, std::string name, unsigned int number);
    /**
     * @brief initConfigurator Initialize all parameter and syncronize parameters between ros and roboteq board
     * @param load_from_board If true load all paramter from roboteq board
     */
    void initConfigurator(bool load_from_board);

    int getOperativeMode();
    /**
     * @brief setOperativeMode Reference in page 321 - MMOD
     * @param type The operating_mode
     * 0 - open_loop
     * 1 - closed_loop_speed
     * 2 - closed_loop_position_relative
     * 3 - closed_loop_count_position
     * 4 - closed_loop_position_tracking
     * 5 - torque
     * 6 - closed_loop_speed_position
     */
    void setOperativeMode(int type);

private:
    /// Setup variable
    bool setup_param, setup_pid_type;

    /// Associate name space
    string mName;
    /// Number motor
    unsigned int mNumber;
    /// Private namespace
    ros::NodeHandle nh_;
    /// Serial port
    roboteq::serial_controller* mSerial;

    /// Dynamic reconfigure parameters
    typedef dynamic_reconfigure::Server<roboteq_control::RoboteqParameterConfig> ReconfigureServerParam;
    std::shared_ptr<ReconfigureServerParam> mDynRecServer_param;
    boost::recursive_mutex mDynServerMutex_param; // To avoid Dynamic Reconfigure Server warning
    /**
     * @brief reconfigureCBParam when the dynamic reconfigurator change some values start this method
     * @param config variable with all configuration from dynamic reconfigurator
     * @param level
     */
    void reconfigureCBParam(roboteq_control::RoboteqParameterConfig &config, uint32_t level);

    /// Dynamic reconfigure PID
    typedef dynamic_reconfigure::Server<roboteq_control::RoboteqPIDtypeConfig> ReconfigureServerPID;
    std::shared_ptr<ReconfigureServerPID> mDynRecServer_pid;
    boost::recursive_mutex mDynServerMutex_pid; // To avoid Dynamic Reconfigure Server warning
    /**
     * @brief reconfigureCBEncoder when the dynamic reconfigurator change some values start this method
     * @param config variable with all configuration from dynamic reconfigurator
     * @param level
     */
    void reconfigureCBPIDtype(roboteq_control::RoboteqPIDtypeConfig &config, uint32_t level);

    // Default parameter config
    roboteq_control::RoboteqParameterConfig default_param_config, _last_param_config;
    roboteq_control::RoboteqPIDtypeConfig default_pid_type_config, _last_pid_type_config;

    /**
     * @brief getParamFromRoboteq Load parameters from Roboteq board
     */
    void getParamFromRoboteq();

};

#endif // GPIOPARAMCONFIGURATOR_H
