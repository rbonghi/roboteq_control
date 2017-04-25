

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

    mSerial->send("FID");

}

Roboteq::~Roboteq()
{

}

void Roboteq::initializeDiagnostic() {

}

void Roboteq::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {

}

}
