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

#ifndef GPIOPIDCONFIGURATOR_H
#define GPIOPICCONFIGURATOR_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <roboteq_control/RoboteqPIDConfig.h>

#include "roboteq/serial_controller.h"

class MotorPIDConfigurator
{
public:
    /**
     * @brief MotorPIDConfigurator Initialize the dynamic reconfigurator
     * @param nh Nodehandle of the system
     * @param serial serial port
     * @param path original path to start to find all rosparam variable
     * @param name name of the PID configuration
     * @param number number of motor
     */
    MotorPIDConfigurator(const ros::NodeHandle& nh, roboteq::serial_controller *serial, string path, string name, unsigned int number);

    void initConfigurator(bool load_from_board);

    void setPIDconfiguration();

private:
    /// Setup variable
    bool setup_pid;

    /// Associate name space
    string mName;
    string mType;
    /// Number motor
    unsigned int mNumber;
    /// Private namespace
    ros::NodeHandle nh_;
    /// Serial port
    roboteq::serial_controller* mSerial;

    /// Dynamic reconfigure PID
    typedef dynamic_reconfigure::Server<roboteq_control::RoboteqPIDConfig> ReconfigureServer;
    std::shared_ptr<ReconfigureServer> mDynRecServer;
    boost::recursive_mutex mDynServerMutex; // To avoid Dynamic Reconfigure Server warning
    /**
     * @brief reconfigureCBEncoder when the dynamic reconfigurator change some values start this method
     * @param config variable with all configuration from dynamic reconfigurator
     * @param level
     */
    void reconfigureCBPID(roboteq_control::RoboteqPIDConfig &config, uint32_t level);

    // Default parameter config
    roboteq_control::RoboteqPIDConfig default_pid_config, _last_pid_config;

    /**
     * @brief getPIDFromRoboteq Load PID parameters from Roboteq board
     */
    void getPIDFromRoboteq();
};

#endif // GPIOPICCONFIGURATOR_H
