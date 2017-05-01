/**
*
*  \author     Raffaello Bonghi <raffaello@rnext.it>
*  \copyright  Copyright (c) 2017, Officine Robotiche, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Raffaello Bonghi nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL RAFFAELLO BONGHI BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to raffaello@rnext.it
*
*/

#include <ros/ros.h>

#include <roboteq_control/RoboteqParameterConfig.h>
#include <dynamic_reconfigure/server.h>

#include "roboteq/serial_controller.h"

typedef struct _motor_params {
    // Ratio gear
    double ratio;
    // Number of ppr configured [0, 5000]
    unsigned int ppr;
    // Motor direction {1 (Clockwise), -1 (Underclockwise)}
    int direction;
    // Max RPM motor axis (before reduction)
    unsigned int max_rpm;
} motor_params_t;

class MotorParamConfigurator
{
public:
    MotorParamConfigurator(const ros::NodeHandle& nh, roboteq::serial_controller *serial, std::string name, unsigned int number);

    motor_params_t initConfigurator();

    // void setParam(motor_parameter_t parameter);

    void getParam();

    motor_params_t params;

private:
    /// Setup variable
    bool setup_param;

    dynamic_reconfigure::Server<roboteq_control::RoboteqParameterConfig> *ds_param;
    void reconfigureCBParam(roboteq_control::RoboteqParameterConfig &config, uint32_t level);

    /// Associate name space
    string mName;
    /// Number motor
    unsigned int mNumber;
    /// Private namespace
    ros::NodeHandle nh_;
    /// Serial port
    roboteq::serial_controller* mSerial;
    /// Setup variable
    bool setup_;
};
