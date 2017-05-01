#ifndef ROBOTEQ_H
#define ROBOTEQ_H

#include <ros/ros.h>
#include <serial/serial.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <hardware_interface/robot_hw.h>

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

class Roboteq : public hardware_interface::RobotHW, public diagnostic_updater::DiagnosticTask
{
public:
    /**
     * @brief serial_controller Open the serial controller
     * @param port set the port
     * @param set the baudrate
     */
    Roboteq(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, serial_controller *serial);

    ~Roboteq();

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

    /**
     * @brief initialize
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
    bool updateDiagnostics();

    void initializeDiagnostic();

    void write(const ros::Time& time, const ros::Duration& period);

    void read(const ros::Time& time, const ros::Duration& period);

private:
    //Initialization object
    //NameSpace for bridge controller
    ros::NodeHandle mNh;
    ros::NodeHandle private_mNh;
    // Serial controller
    serial_controller *mSerial;
    // Diagnostic
    diagnostic_updater::Updater diagnostic_updater;

    /// URDF information about robot
    urdf::Model model;

    /// ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;

    // Check if is the first run
    bool _first;
    // Motor definition
    map<string, Motor*> mMotor;
    map<int, string> mMotorName;

};

}


#endif // ROBOTEQ_H
