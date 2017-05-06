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
