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

#ifndef ROBOTEQ_H
#define ROBOTEQ_H

#include <ros/ros.h>
#include <serial/serial.h>

#include <std_msgs/Bool.h>
#include <roboteq_control/Service.h>
#include <roboteq_control/Peripheral.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <hardware_interface/robot_hw.h>

#include <roboteq_control/RoboteqControllerConfig.h>

#include "configurator/gpio_analog.h"
#include "configurator/gpio_pulse.h"
#include "configurator/gpio_encoder.h"
#include "roboteq/serial_controller.h"
#include "roboteq/motor.h"

using namespace std;

namespace roboteq
{

typedef struct joint
{
    Motor *motor;
    // State of the motor
    double position;
    double velocity;
    double effort;
    double velocity_command;
} joint_t;

typedef struct _status_flag {
    uint8_t serial_mode : 1;
    uint8_t pulse_mode : 1;
    uint8_t analog_mode : 1;
    uint8_t spectrum : 1;
    uint8_t power_stage_off : 1;
    uint8_t stall_detect : 1;
    uint8_t at_limit : 1;
    uint8_t microbasic_running : 1;
} status_flag_t;
// Reference in pag 245
typedef struct _status_fault {
    uint8_t overheat : 1;
    uint8_t overvoltage : 1;
    uint8_t undervoltage : 1;
    uint8_t short_circuit : 1;
    uint8_t emergency_stop : 1;
    uint8_t brushless_sensor_fault : 1;
    uint8_t mosfet_failure : 1;
} status_fault_t;

class Roboteq : public hardware_interface::RobotHW, public diagnostic_updater::DiagnosticTask
{
public:
    /**
     * @brief Roboteq The Roboteq board controller write and read messages about the motor state
     * @param nh The ROS public node handle
     * @param private_nh the ROS  private node handle
     * @param serial The serial controller
     */
    Roboteq(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, serial_controller *serial);
    /**
      * @brief The deconstructor
      */
    ~Roboteq();
    /**
     * @brief Switch off roboteq board
     */
    void switch_off();
    /**
     * @brief run Diagnostic thread called every request
     * @param stat the status of diagnostic updater
     */
    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
    /**
     * @brief initialize the roboteq controller
     */
    void initialize();
    /**
     * @brief initializeInterfaces Initialize all motors.
     * Add all Control Interface availbles and add in diagnostic task
     */
    void initializeInterfaces();
    /**
     * @brief updateDiagnostics
     */
    void updateDiagnostics();

    void initializeDiagnostic();

    void write(const ros::Time& time, const ros::Duration& period);

    void read(const ros::Time& time, const ros::Duration& period);

    bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);

    void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);

private:
    //Initialization object
    //NameSpace for bridge controller
    ros::NodeHandle mNh;
    ros::NodeHandle private_mNh;
    // Serial controller
    serial_controller *mSerial;
    // Diagnostic
    diagnostic_updater::Updater diagnostic_updater;
    // Publisher status periheral
    ros::Publisher pub_peripheral;
    // stop publisher
    ros::Subscriber sub_stop;
    // Service board
    ros::ServiceServer srv_board;

    /// URDF information about robot
    urdf::Model model;

    /// ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;

    bool motor_loop_;
    // Check if is the first run
    bool _first;
    // Motor definition
    //map<string, Motor*> mMotor;
    std::vector<Motor*> mMotor;

    string _type, _model;
    string _version;
    string _uid;
    // Status Roboteq board
    status_flag_t _flag;
    // Fault flags Roboteq board
    status_fault_t _fault;
    // Volts internal
    double _volts_internal, _volts_five;
    // Tempearture inside the Roboteq board
    double _temp_mcu, _temp_bridge;

    // GPIO enable read
    bool _isGPIOreading;
    roboteq_control::Peripheral msg_peripheral;
    std::vector<GPIOAnalogConfigurator*> _param_analog;
    std::vector<GPIOPulseConfigurator*> _param_pulse;
    // Encoder
    std::vector<GPIOEncoderConfigurator*> _param_encoder;


    // stop callback
    void stop_Callback(const std_msgs::Bool::ConstPtr& msg);
    /**
     * @brief getRoboteqInformation Load basic information from roboteq board
     */
    void getRoboteqInformation();

    /// Setup variable
    bool setup_controller;

    /// Dynamic reconfigure PID
    // Dynamic reconfigure
    boost::recursive_mutex mDynServerMutex; // To avoid Dynamic Reconfigure Server warning
    boost::shared_ptr<dynamic_reconfigure::Server<roboteq_control::RoboteqControllerConfig>> mDynRecServer;
    /**
     * @brief reconfigureCBEncoder when the dynamic reconfigurator change some values start this method
     * @param config variable with all configuration from dynamic reconfigurator
     * @param level
     */
    void reconfigureCBController(roboteq_control::RoboteqControllerConfig &config, uint32_t level);

    // Default parameter config
    roboteq_control::RoboteqControllerConfig default_controller_config, _last_controller_config;

    /**
     * @brief getPIDFromRoboteq Load PID parameters from Roboteq board
     */
    void getControllerFromRoboteq();

    /**
     * @brief service_Callback Internal service to require information from the board connected
     * @param req
     * @param msg
     * @return
     */
    bool service_Callback(roboteq_control::Service::Request &req, roboteq_control::Service::Response &msg_system);
    /**
     * @brief connectionCallback Check how many subscribers are connected
     * @param pub The information about the subscriber
     */
    void connectionCallback(const ros::SingleSubscriberPublisher& pub);

};

}


#endif // ROBOTEQ_H
