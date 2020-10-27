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
#include "configurator/gpio_sensor.h"
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
    ~Motor();
    /**
     * @brief initializeMotor Initialization oh motor, this routine load parameter from ros server or load from roboteq board
     * @param load_from_board forse the load from roboteq board
     */
    void initializeMotor(bool load_from_board);
    /**
     * @brief run Run the diagnostic updater
     * @param stat the stat will be updated
     */
    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
    /**
     * @brief setupLimits setup the maximum velocity, positio and effort
     * @param model the robot model
     */
    void setupLimits(urdf::Model model);
    /**
     * @brief resetPosition Reset the motor in a new initial position
     * @param position the new position
     */
    void resetPosition(double position);
    /**
     * @brief writeCommandsToHardware Write a command to the hardware interface
     * @param period the period update
     */
    void writeCommandsToHardware(ros::Duration period);
    /**
     * @brief switchController Switch the controller from different type of ros controller
     * @param type the type of ros controller
     */
    void switchController(string type);
    /**
     * @brief stopMotor Stop the motor
     */
    void stopMotor();
    /**
     * @brief getNumber The roboteq number
     * @return the number associated in the roboteq board
     */
    int getNumber() {
        return mNumber;
    }

    /**
     * @brief getName the name of the motor
     * @return the string with the name of the motor
     */
    string getName() {
        return mMotorName;
    }
    /**
     * @brief registerSensor register the sensor
     * @param sensor the sensor interface
     */
    void registerSensor(GPIOSensor* sensor);
    /**
     * @brief readVector Decode vector data list
     * @param fields field of measures
     */
    void readVector(std::vector<std::string> fields);

    hardware_interface::JointStateHandle joint_state_handle;
    hardware_interface::JointHandle joint_handle;

    // Number of motor
    unsigned int mNumber;
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
    // State of the motor
    double position_, max_position_;
    double velocity_, max_velocity_;
    double effort_, max_effort_;
    double command_;
    // Motor reduction and ratio
    double reduction_;
    double ratio_;
    // max RPM
    double max_rpm_;

    int _control_mode;
    motor_status_t status_;

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

    GPIOSensor* _sensor;

    // Reader motor message
    void read(string data);

    void connectionCallback(const ros::SingleSubscriberPublisher& pub);
};

}

#endif // MOTOR_H
