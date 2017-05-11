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

#ifndef MOTOR_H
#define MOTOR_H

#include <ros/ros.h>
#include <urdf/model.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <roboteq_control/MotorStatus.h>
#include <roboteq_control/ControlStatus.h>

#include "roboteq/serial_controller.h"

#include "configurator/motor_param.h"
#include "configurator/motor_pid.h"

namespace roboteq
{

typedef struct _motor_status {
    uint8_t amps_limit : 1;
    uint8_t motor_stalled : 1;
    uint8_t loop_error_detect : 1;
    uint8_t safety_stop_active : 1;
    uint8_t forward_limit_triggered : 1;
    uint8_t reverse_limit_triggered : 1;
    uint8_t amps_triggered_active : 1;
    uint8_t : 1;
} motor_status_t;

class Motor : public diagnostic_updater::DiagnosticTask
{
public:
    /**
     * @brief Motor The motor definition and all ros controller initializations are collected in this place
     * @param nh The ROS private node handle
     * @param serial The serial controller
     * @param name The name of the motor
     * @param number The number in Roboteq board
     */
    explicit Motor(const ros::NodeHandle &nh, serial_controller *serial, string name, unsigned int number);

    void initializeMotor(bool load_from_board);

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void setupLimits(urdf::Model model);

    void resetPosition(double position);

    void writeCommandsToHardware(ros::Duration period);

    void switchController(string type);
    /**
     * @brief stopMotor Stop the motor
     */
    void stopMotor();

    hardware_interface::JointStateHandle joint_state_handle;
    hardware_interface::JointHandle joint_handle;

protected:
  /**
   * @param x Angular velocity in radians/s.
   * @return Angular velocity in RPM.
   */
  static double to_rpm(double x)
  {
    return x * 60 / (2 * M_PI);
  }

  /**
   * @param x Angular velocity in RPM.
   * @return Angular velocity in rad/s.
   */
  static double from_rpm(double x)
  {
    return x * (2 * M_PI) / 60;
  }

  /**
   * Conversion of radians to encoder ticks.
   *
   * @param x Angular position in radians.
   * @return Angular position in encoder ticks.
   */
  double to_encoder_ticks(double x);

  /**
   * Conversion of encoder ticks to radians.
   *
   * @param x Angular position in encoder ticks.
   * @return Angular position in radians.
   */
  double from_encoder_ticks(double x);

private:
    //Initialization object
    //NameSpace for bridge controller
    ros::NodeHandle mNh;
    // Name of the motor
    string mMotorName;
    // Serial controller communication
    serial_controller *mSerial;
    // Number of motor
    unsigned int mNumber;
    // State of the motor
    double position, max_position;
    double velocity, max_velocity;
    double effort, max_effort;
    double command;

    int _control_mode;
    motor_status_t _status;

    /// ROS joint limits interface
    joint_limits_interface::VelocityJointSoftLimitsInterface vel_limits_interface;

    // Publisher diagnostic information
    ros::Publisher pub_status, pub_control;
    // Message
    roboteq_control::MotorStatus msg_status;
    roboteq_control::ControlStatus msg_control;

    MotorParamConfigurator* parameter;
    MotorPIDConfigurator* pid_velocity;
    MotorPIDConfigurator* pid_torque;
    MotorPIDConfigurator* pid_position;

    // Reader motor message
    void read(string data);

    void connectionCallback(const ros::SingleSubscriberPublisher& pub);
};

}

#endif // MOTOR_H
