
#include "roboteq/motor.h"

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

namespace roboteq {

Motor::Motor(const ros::NodeHandle& nh, serial_controller *serial, string name, unsigned int number)
    : DiagnosticTask(name + "_status")
    , joint_state_handle(name, &position, &velocity, &effort)
    , joint_handle(joint_state_handle, &command)
    , mNh(nh)
    , mSerial(serial)
{
    // Initialize all variables
    mNumber = number+1;
    mMotorName = name;
    command = 0;

    parameter = new MotorParamConfigurator(nh, serial, mMotorName, number);

    // Add a status motor publisher
    pub_status = mNh.advertise<roboteq_control::MotorStatus>(mMotorName + "/status", 10);

    pub_reference = mNh.advertise<roboteq_control::ControlStatus>(mMotorName + "/reference", 10,
            boost::bind(&Motor::connectionCallback, this, _1), boost::bind(&Motor::connectionCallback, this, _1));
    pub_measure = mNh.advertise<roboteq_control::ControlStatus>(mMotorName + "/measure", 10);
    pub_control = mNh.advertise<roboteq_control::ControlStatus>(mMotorName + "/control", 10,
            boost::bind(&Motor::connectionCallback, this, _1), boost::bind(&Motor::connectionCallback, this, _1));


    // Add callback
    mSerial->addCallback(&Motor::read, this, "F" + std::to_string(mNumber));
}

void Motor::connectionCallback(const ros::SingleSubscriberPublisher& pub)
{
    ROS_DEBUG_STREAM("Update: " << pub.getSubscriberName() << " - " << pub.getTopic());
}

void Motor::initializeMotor()
{

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

    boost::shared_ptr<const urdf::Joint> urdf_joint = model.getJoint(mMotorName);
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
    // Update limits
    max_position = limits.max_position;
    max_velocity = limits.max_velocity * 16.0; // TODO Multiply with ration
    max_effort = limits.max_effort;


    //updateLimits(limits.max_position, limits.max_velocity, limits.max_effort);

    joint_limits_interface::VelocityJointSoftLimitsHandle handle(joint_handle, // We read the state and read/write the command
                                                                 limits,       // Limits spec
                                                                 soft_limits);  // Soft limits spec

    vel_limits_interface.registerHandle(handle);
}

void Motor::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{

}

void Motor::resetPosition(double position)
{
    // Send reset position
    // TODO Convert position in tick and send - now send only 0
    mSerial->command("C " + std::to_string(mNumber) + " " + std::to_string(0));
}

void Motor::writeCommandsToHardware(ros::Duration period)
{
    // Enforce joint limits for all registered handles
    // Note: one can also enforce limits on a per-handle basis: handle.enforceLimits(period)
    vel_limits_interface.enforceLimits(period);
    long long int roboteq_velocity = static_cast<long long int>(to_rpm(command*16.0) / 4096 *1000.0);

    // ROS_INFO_STREAM("Velocity" << mNumber << " val=" << command << " " << roboteq_velocity);

    mSerial->command("G " + std::to_string(mNumber) + " " + std::to_string(roboteq_velocity));
}

void Motor::read(string data) {
    // ROS_INFO_STREAM("Motor" << mNumber << " " << data);

    std::vector<std::string> fields;
    boost::split(fields, data, boost::algorithm::is_any_of(":"));

    msg_status.header.stamp = ros::Time::now();
    msg_measure.header.stamp = ros::Time::now();
    msg_control.header.stamp = ros::Time::now();
    msg_reference.header.stamp = ros::Time::now();

    // Scale factors as outlined in the relevant portions of the user manual, please
    // see mbs/script.mbs for URL and specific page references.
    try
    {
        msg_measure.current = boost::lexical_cast<float>(fields[0]) / 10;
        msg_reference.velocity = from_rpm(boost::lexical_cast<double>(fields[1]));
//      msg.motor_power = boost::lexical_cast<float>(fields[2]) / 1000.0;
        msg_measure.velocity = from_rpm(boost::lexical_cast<double>(fields[3]));
        msg_measure.position = from_encoder_ticks(boost::lexical_cast<double>(fields[4]));
//      msg.supply_voltage = boost::lexical_cast<float>(fields[5]) / 10.0;
//      msg.supply_current = boost::lexical_cast<float>(fields[6]) / 10.0;
//      msg.motor_temperature = boost::lexical_cast<int>(fields[7]) * 0.020153 - 4.1754;
//      msg.channel_temperature = boost::lexical_cast<int>(fields[8]);

        // Update joint status
        effort = msg_measure.effort;
        position = msg_measure.position;
        velocity = msg_measure.velocity;
    }
    catch (std::bad_cast& e)
    {
      ROS_WARN("Failure parsing feedback data. Dropping message.");
      return;
    }
    pub_status.publish(msg_status);
    pub_measure.publish(msg_measure);
    pub_control.publish(msg_control);
    pub_reference.publish(msg_reference);
}

}
