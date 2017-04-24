#ifndef MOTOR_H
#define MOTOR_H

#include <ros/ros.h>
#include <urdf/model.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include "roboteq/roboteq.h"

namespace roboteq
{

class Motor : public diagnostic_updater::DiagnosticTask
{
public:
    explicit Motor(const ros::NodeHandle &nh, Roboteq *roboteq, string name, unsigned int number);

    void initializeMotor();

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

    hardware_interface::JointStateHandle joint_state_handle;
    hardware_interface::JointHandle joint_handle;

private:
    //Initialization object
    //NameSpace for bridge controller
    ros::NodeHandle mNh;
    // Serial controller communication
    Roboteq *mRoboteq;

    double position, max_position;
    double velocity, max_velocity;
    double effort, max_effort;
    double command;
};

}

#endif // MOTOR_H
