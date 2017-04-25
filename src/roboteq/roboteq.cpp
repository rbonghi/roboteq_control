

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

    if(mSerial->query("FID"))
    {
        ROS_INFO_STREAM("Data=" << mSerial->get());
    }

    if(mSerial->query("TRN"))
    {
        ROS_INFO_STREAM("Data=" << mSerial->get());
    }

    ROS_INFO_STREAM("Script: " << script(true));

    // ROS_INFO_STREAM("Script: " << script(false));

}

Roboteq::~Roboteq()
{

}

void Roboteq::initializeDiagnostic() {

}

void Roboteq::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {

}

}
