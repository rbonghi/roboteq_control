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

#include "roboteq/motor.h"
#include <boost/algorithm/algorithm.hpp>
#include <boost/algorithm/string.hpp>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

namespace roboteq {

Motor::Motor(const ros::NodeHandle& nh, serial_controller *serial, string name, unsigned int number)
    : DiagnosticTask(name + "_status")
    , joint_state_handle(name, &position_, &velocity_, &effort_)
    , joint_handle(joint_state_handle, &command_)
    , mNh(nh)
    , mSerial(serial)
{
    // Initialize all variables
    mNumber = number;
    mMotorName = name;
    command_ = 0;
    // Reset variables
    position_ = 0;
    velocity_ = 0;
    effort_ = 0;
    // Initialize control mode
    _control_mode = -1;
    // Initialize reduction and get ratio
    reduction_ = 0;
    // Get ratio
    mNh.getParam(mMotorName + "/ratio", ratio_);
    ROS_INFO_STREAM("Motor" << mNumber << " " << ratio_);
    // Get encoder max speed parameter
    max_rpm_ = 0;
    mNh.getParam(mMotorName + "/max_speed", max_rpm_);

    // Initialize Dynamic reconfigurator for generic parameters
    parameter = new MotorParamConfigurator(nh, serial, mMotorName, number);
    // Initialize Dynamic reconfigurator for generic parameters
    pid_velocity = new MotorPIDConfigurator(nh, serial, mMotorName, "velocity", number);
    pid_torque = new MotorPIDConfigurator(nh, serial, mMotorName, "torque", number);
    pid_position = new MotorPIDConfigurator(nh, serial, mMotorName, "position", number);

    // Add a status motor publisher
    pub_status = mNh.advertise<roboteq_control::MotorStatus>(mMotorName + "/status", 10);
    pub_control = mNh.advertise<roboteq_control::ControlStatus>(mMotorName + "/control", 10);

    // Add callback
    // mSerial->addCallback(&Motor::read, this, "F" + std::to_string(mNumber));
}

Motor::~Motor() {
    delete parameter;
    delete pid_velocity;
    delete pid_torque;
    delete pid_position;
}

void Motor::connectionCallback(const ros::SingleSubscriberPublisher& pub)
{
    ROS_DEBUG_STREAM("Update: " << pub.getSubscriberName() << " - " << pub.getTopic());
}

void Motor::initializeMotor(bool load_from_board)
{
    // Initialize parameters
    parameter->initConfigurator(load_from_board);
    // Load PID configuration from roboteq board
    // Get operative mode
    _control_mode = parameter->getOperativeMode();
    bool tmp_pos = load_from_board & ((_control_mode == 2) || (_control_mode == 3) || (_control_mode == 4));
    bool tmp_vel = load_from_board & ((_control_mode == 1) || (_control_mode == 6));
    bool tmp_tor = load_from_board & (_control_mode == 5);

    // ROS_INFO_STREAM("Type pos:" << tmp_pos << " vel:" << tmp_vel << " tor:" << tmp_tor);

    // Initialize pid loader
    pid_position->initConfigurator(tmp_pos);
    // Initialize pid loader
    pid_velocity->initConfigurator(tmp_vel);
    // Initialize pid loader
    pid_torque->initConfigurator(tmp_tor);

    // stop the motor
    stopMotor();
}

/**
 * @brief registerSensor register the sensor
 * 
 * @param sensor the sensor interface
 */
void Motor::registerSensor(GPIOSensor* sensor)
{
    _sensor = sensor;
    mNh.getParam(mMotorName + "/ratio", reduction_);
    reduction_ = _sensor->getConversion(reduction_);
}

/**
 * Conversion of radians to encoder ticks.
 *
 * @param x Angular position in radians.
 * @return Angular position in encoder ticks.
 */
double Motor::to_encoder_ticks(double x)
{
    // Return the value converted
    return x * (reduction_) / (2 * M_PI);
}

/**
 * Conversion of encoder ticks to radians.
 *
 * @param x Angular position in encoder ticks.
 * @return Angular position in radians.
 */
double Motor::from_encoder_ticks(double x)
{
    // Return the value converted
    return x * (2 * M_PI) / (reduction_);
}

void Motor::setupLimits(urdf::Model model)
{
    /// Add a velocity joint limits infomations
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::SoftJointLimits soft_limits;

    bool state = true;

    // Manual value setting
    limits.has_velocity_limits = true;
    limits.max_velocity = 5.0;
    limits.has_effort_limits = true;
    limits.max_effort = 2.0;

    // Populate (soft) joint limits from URDF
    // Limits specified in URDF overwrite existing values in 'limits' and 'soft_limits'
    // Limits not specified in URDF preserve their existing values

    urdf::JointConstSharedPtr urdf_joint = model.getJoint(mMotorName);
    const bool urdf_limits_ok = getJointLimits(urdf_joint, limits);
    const bool urdf_soft_limits_ok = getSoftJointLimits(urdf_joint, soft_limits);

    if(urdf_limits_ok) {
        ROS_INFO_STREAM("LOAD [" << mMotorName << "] limits from URDF: |" << limits.max_velocity << "| rad/s & |" << limits.max_effort << "| Nm");
        state = false;
    }

    if(urdf_soft_limits_ok) {
        ROS_INFO_STREAM("LOAD [" << mMotorName << "] soft limits from URDF: |" << limits.max_velocity << "| rad/s & |" << limits.max_effort << "| Nm");
        state = false;
    }

    // Populate (soft) joint limits from the ros parameter server
    // Limits specified in the parameter server overwrite existing values in 'limits' and 'soft_limits'
    // Limits not specified in the parameter server preserve their existing values
    const bool rosparam_limits_ok = getJointLimits(mMotorName, mNh, limits);
    if(rosparam_limits_ok) {
        ROS_WARN_STREAM("OVERLOAD [" << mMotorName << "] limits from ROSPARAM: |" << limits.max_velocity << "| rad/s & |" << limits.max_effort << "| Nm");
        state = false;
    }
    else
    {
        ROS_DEBUG("Setup limits, PARAM NOT available");
    }
    // If does not read any parameter from URDF or rosparm load default parameter
    if(state)
    {
        ROS_WARN_STREAM("LOAD [" << mMotorName << "] with DEFAULT limit = |" << limits.max_velocity << "| rad/s & |" << limits.max_effort << "| Nm");
    }

    // Set maximum limits if doesn't have limit
    if(limits.has_position_limits == false)
    {
        limits.max_position = 6.28;
    }
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    // TODO CHECK HOW TO INITIALIZE
    // Update limits
    // max_position = limits.max_position;
    // max_velocity = limits.max_velocity * params.ratio;
    // max_effort = limits.max_effort;
    // updateLimits(limits.max_position, limits.max_velocity, limits.max_effort);
    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    joint_limits_interface::VelocityJointSoftLimitsHandle handle(joint_handle, // We read the state and read/write the command
                                                                 limits,       // Limits spec
                                                                 soft_limits);  // Soft limits spec

    vel_limits_interface.registerHandle(handle);
}

void Motor::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    string control;
    switch(_control_mode)
    {
    case -1:
        control = "unset";
        break;
    case 0:
        control = "Open loop";
        break;
    case 1:
        control = "Closed loop speed";
        break;
    case 2:
        control = "Closed loop position relative";
        break;
    case 3:
        control = "Closed loop count position";
        break;
    case 4:
        control = "Closed loop position tracking";
        break;
    case 5:
        control = "Closed loop torque";
        break;
    case 6:
        control = "Closed loop speed position";
        break;
    default:
        control = "Error";
        break;
    }
    stat.add("Control", control);

    stat.add("Motor number", mNumber);
    stat.add("PWM rate (%)", msg_control.pwm);
    stat.add("Voltage (V)", msg_status.volts);
    stat.add("Battery (A)", msg_status.amps_batt);
    stat.add("Watt motor (W)", msg_status.volts * msg_status.amps_motor);
    stat.add("Watt batt (W)", msg_status.volts * msg_status.amps_batt);
    stat.add("Loop error", msg_control.loop_error);
    stat.add("Track", msg_status.track);
    stat.add("Position (deg)", position_);
    stat.add("Velociy (RPM)", to_rpm(velocity_));
    stat.add("Current (A)", msg_status.amps_motor);
    stat.add("Torque (Nm)", effort_);


    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motor Ready!");

    if(status_.amps_limit)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Amps limits motor=%.2f", msg_status.amps_motor);
    }

    if(status_.amps_triggered_active)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Amps trigger active");
    }

    if(status_.forward_limit_triggered)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Forward limit triggered");
    }

    if(status_.reverse_limit_triggered)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Reverse limit triggered");
    }

    if(status_.loop_error_detect)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Loop error detection");
    }

    if(status_.motor_stalled)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor stalled");
    }

    if(status_.safety_stop_active)
    {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Safety stop active");
    }
}

void Motor::stopMotor()
{
    // set to zero the reference
    mSerial->command("G ", std::to_string(mNumber) + " 0");
    // Stop motor [pag 222]
    mSerial->command("MS", std::to_string(mNumber));
}

void Motor::switchController(string type)
{
    if(type.compare("diff_drive_controller/DiffDriveController") == 0)
    {
        // Load type of PID velocity
        int pid_vel;
        mNh.getParam(mMotorName + "/pid/closed_loop_velocity", pid_vel);
        // ROS_INFO_STREAM("VEL mode:" << pid_vel);
        // Set in speed position mode
        parameter->setOperativeMode(pid_vel);
        _control_mode = pid_vel;
    }
    else
    {
        _control_mode = -1;
        // stop motor
        stopMotor();
    }
}

void Motor::resetPosition(double position)
{
    // Send reset position
    double enc_conv = to_encoder_ticks(position);
    mSerial->command("C ", std::to_string(mNumber) + " " + std::to_string(enc_conv));
}

void Motor::writeCommandsToHardware(ros::Duration period)
{
    // Enforce joint limits for all registered handles
    // Note: one can also enforce limits on a per-handle basis: handle.enforceLimits(period)
    vel_limits_interface.enforceLimits(period);
    // Get encoder max speed parameter
    double max_rpm;
    mNh.getParam(mMotorName + "/max_speed", max_rpm);
    // Build a command message
    long long int roboteq_velocity = static_cast<long long int>(to_rpm(command_) / max_rpm * 1000.0);

    // ROS_INFO_STREAM("Velocity" << mNumber << " val=" << command << " " << roboteq_velocity);

    mSerial->command("G ", std::to_string(mNumber) + " " + std::to_string(roboteq_velocity));
}

void Motor::read(string data) {
    std::vector<std::string> fields;
    boost::split(fields, data, boost::algorithm::is_any_of(":"));
    // Decode list
    readVector(fields);
}

void Motor::readVector(std::vector<std::string> fields) {
    double position, vel, volts, amps_motor;
    // Scale factors as outlined in the relevant portions of the user manual, please
    // see mbs/script.mbs for URL and specific page references.
    try
    {
        // reference command FM <-> _MOTFLAG [pag. 246]
        unsigned char status = boost::lexical_cast<unsigned int>(fields[0]);
        memcpy(&status_, &status, sizeof(status));
        // reference command M <-> _MOTCMD [pag. 250]
        double cmd = boost::lexical_cast<double>(fields[1]) * max_rpm_ / 1000.0;
        // reference command F <-> _FEEDBK [pag. 244]
        vel = boost::lexical_cast<double>(fields[2]) * max_rpm_ / 1000.0;
        // reference command E <-> _LPERR [pag. 243]
        double loop_error = boost::lexical_cast<double>(fields[3]) * max_rpm_ / 1000.0;
        // reference command P <-> _MOTPWR [pag. 255]
        double pwm = boost::lexical_cast<double>(fields[4]);
        // reference voltage V <-> _VOLTS [pag. ---]
        volts = boost::lexical_cast<double>(fields[5]) / 10;
        // reference command A <-> _MOTAMPS [pag. 230]
        amps_motor = boost::lexical_cast<double>(fields[6]) / 10;
        // reference command BA <-> _BATAMPS [pag. 233]
        double amps_batt = boost::lexical_cast<double>(fields[7]) / 10;
        // Reference command CR <-> _RELCNTR [pag. 241]
        // To check and substitute with C
        // Reference command C <-> _ABCNTR [pag. ---]
        position = boost::lexical_cast<double>(fields[8]);
        // reference command TR <-> _TR [pag. 260]
        double track = boost::lexical_cast<long>(fields[9]);

        // Build messages
        msg_control.header.stamp = ros::Time::now();
        // Fill fields
        msg_control.reference = (cmd / ratio_);
        msg_control.feedback = (vel / ratio_);
        msg_control.loop_error = (loop_error / ratio_);
        msg_control.pwm = pwm;
        // Publish status control motor
        pub_control.publish(msg_control);

        // Build control message
        msg_status.header.stamp = ros::Time::now();
        // Fill fields
        msg_status.volts = volts;
        msg_status.amps_motor = amps_motor;
        msg_status.amps_batt = amps_batt;
        msg_status.track = track;
        // Publish status motor
        pub_status.publish(msg_status);
    }
    catch (std::bad_cast& e)
    {
        ROS_WARN_STREAM(" [" << mNumber << "] " << mMotorName << ": Failure parsing feedback data. Dropping message. " << e.what());
        // Load same values
        position = to_encoder_ticks(position_);
        vel = ratio_ * velocity_;
        volts = 0;
        amps_motor = 0;
    }
    // Update position
    position_ = from_encoder_ticks(position);
    // Update velocity motor
    velocity_ = (vel / ratio_);
    // Evaluate effort
    if(velocity_ != 0)
    {
        effort_ = ((volts * amps_motor) / velocity_) * ratio_;
    }
    else
    {
        effort_ = 0;
    }
    //ROS_INFO_STREAM("[" << mNumber << "] track:" << msg_status.track);
    //ROS_INFO_STREAM("[" << mNumber << "] volts:" << msg_status.volts << " - amps:" << msg_status.amps_motor);
    //ROS_INFO_STREAM("[" << mNumber << "] status:" << status << " - pos:"<< position << " - vel:" << velocity << " - torque:");

}

}
