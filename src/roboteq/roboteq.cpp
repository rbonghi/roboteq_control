

#include "roboteq/roboteq.h"

namespace roboteq
{

Roboteq::Roboteq(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, serial_controller *serial)
    : DiagnosticTask("Roboteq")
    , mNh(nh)
    , private_mNh(private_nh)
    , mSerial(serial)
{
    std::vector<std::string> joint_list;
    if(private_nh.hasParam("joint"))
    {
        private_nh.getParam("joint", joint_list);
    }
    else
    {
        ROS_WARN("No joint list!");
        joint_list.push_back("joint_0");
        joint_list.push_back("joint_1");
        private_nh.setParam("joint", joint_list);
    }

    // Disable ECHO
    mSerial->echo(false);
    // Disable Script and wait to load all parameters
    mSerial->script(false);

    // Initialize Joints
    for(unsigned i=0; i < joint_list.size(); ++i)
    {
        string motor_name = joint_list.at(i);
        int number = i;
        if(private_nh.hasParam(motor_name + "/number"))
        {
            private_nh.getParam(motor_name + "/number", number);
        }
        else
        {
            ROS_WARN_STREAM("Default number selected for Motor: " << motor_name << " is " << number);
            private_nh.setParam(motor_name + "/number", number);
        }

        ROS_INFO_STREAM("Motor[" << number << "] name: " << motor_name);
        mMotor[motor_name] = new Motor(private_mNh, serial, motor_name, number);
        mMotorName[number] = motor_name;
    }

    if(mSerial->query("FID"))
    {
        ROS_INFO_STREAM("Data=" << mSerial->get());
    }

    if(mSerial->query("TRN"))
    {
        ROS_INFO_STREAM("Data=" << mSerial->get());
    }

    mSerial->command("MG");

    // Enable script
    mSerial->script(true);
}

Roboteq::~Roboteq()
{
    // ROS_INFO_STREAM("Script: " << script(false));
}

void Roboteq::initialize()
{
    // Initialize all motors in list
    for( map<string, Motor*>::iterator ii=mMotor.begin(); ii!=mMotor.end(); ++ii)
    {
        // Launch initialization motors
        (*ii).second->initializeMotor();
        ROS_DEBUG_STREAM("Motor [" << (*ii).first << "] Initialized");
    }
}

void Roboteq::initializeInterfaces()
{
    // Initialize the diagnostic from the primitive object
    initializeDiagnostic();

    if (!model.initParam("/robot_description")){
        ROS_ERROR("Failed to parse urdf file");
    }
    else
    {
        ROS_INFO_STREAM("/robot_description found! " << model.name_ << " parsed!");
    }

    for( map<string, Motor*>::iterator ii=mMotor.begin(); ii!=mMotor.end(); ++ii)
    {
        /// State interface
        joint_state_interface.registerHandle(((*ii).second)->joint_state_handle);
        /// Velocity interface
        velocity_joint_interface.registerHandle(((*ii).second)->joint_handle);

        // Setup limits
        ((*ii).second)->setupLimits(model);

        // reset position joint
        double position = 0;
        ROS_DEBUG_STREAM("Motor [" << (*ii).first << "] reset position to: " << position);
        ((*ii).second)->resetPosition(position);

        //Add motor in diagnostic updater
        diagnostic_updater.add(*((*ii).second));
        ROS_DEBUG_STREAM("Motor [" << (*ii).first << "] Registered");
    }
    ROS_DEBUG_STREAM("Send all Constraint configuration");

    /// Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&velocity_joint_interface);
}

void Roboteq::initializeDiagnostic() {

}

void Roboteq::read(const ros::Time& time, const ros::Duration& period) {
    //ROS_DEBUG_STREAM("Get measure from uNav");
    for( map<string, Motor*>::iterator ii=mMotor.begin(); ii!=mMotor.end(); ++ii)
    {
        // Not required now
        // TODO
        //(*ii).second->addRequestMeasure();
        ROS_DEBUG_STREAM("Motor [" << (*ii).first << "] Request measures");
    }
}

void Roboteq::write(const ros::Time& time, const ros::Duration& period) {
    //ROS_DEBUG_STREAM("Write command to uNav");
    for( map<string, Motor*>::iterator ii=mMotor.begin(); ii!=mMotor.end(); ++ii)
    {
        (*ii).second->writeCommandsToHardware(period);
        ROS_DEBUG_STREAM("Motor [" << (*ii).first << "] Send commands");
    }
}

void Roboteq::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {

}

}
