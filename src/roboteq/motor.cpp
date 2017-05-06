
#include "roboteq/motor.h"

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

namespace roboteq {

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
    // Reset variables
    position = 0;
    velocity = 0;
    effort = 0;

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
    mSerial->addCallback(&Motor::read, this, "F" + std::to_string(mNumber));
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
    int mode = parameter->getOperativeMode();
    bool tmp_pos = load_from_board & ((mode == 2) || (mode == 3) || (mode == 4));
    bool tmp_vel = load_from_board & ((mode == 1) || (mode == 6));
    bool tmp_tor = load_from_board & (mode == 5);

    // ROS_INFO_STREAM("Type pos:" << tmp_pos << " vel:" << tmp_vel << " tor:" << tmp_tor);

    // Initialize pid loader
    pid_position->initConfigurator(tmp_pos);
    // Initialize pid loader
    pid_velocity->initConfigurator(tmp_vel);
    // Initialize pid loader
    pid_torque->initConfigurator(tmp_tor);
}

/**
 * Conversion of radians to encoder ticks.
 *
 * @param x Angular position in radians.
 * @return Angular position in encoder ticks.
 */
double Motor::to_encoder_ticks(double x)
{
    double reduction = parameter->getReduction();
    return x * (reduction) / (2 * M_PI);
}

/**
 * Conversion of encoder ticks to radians.
 *
 * @param x Angular position in encoder ticks.
 * @return Angular position in radians.
 */
double Motor::from_encoder_ticks(double x)
{
    double reduction = parameter->getReduction();
    return x * (2 * M_PI) / (reduction);
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
    stat.add("State ", 11);

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motor Ready!");

//    stat.add("PWM rate (%)", msg_measure.pwm);
//    stat.add("Voltage (V)", msg_status.voltage);
//    stat.add("Watt (W)", msg_status.watt);
//    stat.add("Temperature (°C)", msg_status.temperature);
//    stat.add("Time execution (nS)", msg_status.time_execution);

//    stat.add("Position (deg)", ((double)msg_measure.position) * 180.0/M_PI);
//    stat.add("Velociy (RPM)", ((double)msg_measure.velocity) * (30.0 / M_PI));
//    stat.add("Current (A)", fabs(msg_measure.current));
//    stat.add("Torque (Nm)", msg_measure.effort);
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
    }
    else
    {
        // set to zero the reference
        mSerial->command("G ", std::to_string(mNumber) + " 0");
        // Stop motor [pag 222]
        mSerial->command("MS", std::to_string(mNumber));
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
    long long int roboteq_velocity = static_cast<long long int>(to_rpm(command) / max_rpm * 1000.0);

    // ROS_INFO_STREAM("Velocity" << mNumber << " val=" << command << " " << roboteq_velocity);

    mSerial->command("G ", std::to_string(mNumber) + " " + std::to_string(roboteq_velocity));
}

void Motor::read(string data) {
    double ratio, max_rpm;
    // ROS_INFO_STREAM("Motor" << mNumber << " " << data);

    std::vector<std::string> fields;
    boost::split(fields, data, boost::algorithm::is_any_of(":"));
    // Get ratio
    mNh.getParam(mMotorName + "/ratio", ratio);
    // Get encoder max speed parameter
    mNh.getParam(mMotorName + "/max_speed", max_rpm);
    // Build messages
    msg_status.header.stamp = ros::Time::now();
    msg_control.header.stamp = ros::Time::now();

    // Scale factors as outlined in the relevant portions of the user manual, please
    // see mbs/script.mbs for URL and specific page references.
    try
    {
        // reference command FM <-> _MOTFLAG [pag. 246]
        unsigned char status = boost::lexical_cast<uint8_t>(fields[0]);

        // reference command M <-> _MOTCMD [pag. 250]
        double cmd = boost::lexical_cast<double>(fields[1]) * max_rpm / 1000.0;
        msg_control.reference = (cmd / ratio);

        // reference command F <-> _FEEDBK [pag. 244]
        double vel = boost::lexical_cast<double>(fields[2]) * max_rpm / 1000.0;
        msg_control.feedback = (vel / ratio);
        // Update velocity motor
        velocity = (vel / ratio);

        // reference command E <-> _LPERR [pag. 243]
        double loop_error = boost::lexical_cast<double>(fields[3]) * max_rpm / 1000.0;
        msg_control.loop_error = (loop_error / ratio);

        // reference command P <-> _MOTPWR [pag. 255]
        msg_control.pwm = boost::lexical_cast<double>(fields[4]) / 10;

        // reference voltage V <-> _VOLTS [pag. ---]
        msg_status.volts = boost::lexical_cast<double>(fields[5]) / 10;

        // reference command A <-> _MOTAMPS [pag. 230]
        msg_status.amps_motor = boost::lexical_cast<double>(fields[6]) / 10;

        // Evaluate effort
        if(velocity != 0) effort = ((msg_status.volts * msg_status.amps_motor) / velocity) * ratio;
        else effort = 0;

        // reference command BA <-> _BATAMPS [pag. 233]
        msg_status.amps_motor = boost::lexical_cast<double>(fields[7]) / 10;

        // Reference command CR <-> _RELCNTR [pag. 241]
        // To check and substitute with C
        // Reference command C <-> _ABCNTR [pag. ---]
        position = from_encoder_ticks(boost::lexical_cast<double>(fields[8]));

        // reference command TR <-> _TR [pag. 260]
        msg_status.track = boost::lexical_cast<long>(fields[9]);

        // ROS_INFO_STREAM("[" << mNumber << "] track:" << msg_status.track);
        // ROS_INFO_STREAM("[" << mNumber << "] volts:" << msg_status.volts << " - amps:" << msg_status.amps_motor);
        // ROS_INFO_STREAM("[" << mNumber << "] status:" << status << " - pos:"<< position << " - vel:" << velocity << " - torque:");
    }
    catch (std::bad_cast& e)
    {
      ROS_WARN("Failure parsing feedback data. Dropping message.");
      return;
    }
    // Publish status motor
    pub_status.publish(msg_status);
    // Publish status control motor
    pub_control.publish(msg_control);
}

}
