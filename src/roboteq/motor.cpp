
#include "roboteq/motor.h"

namespace roboteq {

Motor::Motor(const ros::NodeHandle& nh, Roboteq *roboteq, string name, unsigned int number)
    : DiagnosticTask(name + "_status")
    , joint_state_handle(name, &position, &velocity, &effort)
    , joint_handle(joint_state_handle, &command)
    , mNh(nh)
    , mRoboteq(roboteq)
{

}
void Motor::initializeMotor()
{

}

void Motor::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{

}

}
