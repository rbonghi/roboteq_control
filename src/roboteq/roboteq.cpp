

#include "roboteq/roboteq.h"

namespace roboteq
{

Roboteq::Roboteq(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, string port, unsigned long baudrate)
    : DiagnosticTask("Roboteq")
    , mNh(nh)
    , private_mNh(private_nh)
    , mSerialPort(port)
    , mBaudrate(baudrate)
{
    // Start status of the serial controller
    mStatus = SERIAL_OK;
    // Default timeout
    mTimeout = 500;

//    std::vector<std::string> joint_list;
//    if(private_nh.hasParam("joint"))
//    {
//        private_nh.getParam("joint", joint_list);
//    }
//    else
//    {
//        ROS_WARN("No joint list!");
//        joint_list.push_back("joint_0");
//        joint_list.push_back("joint_1");
//        private_nh.setParam("joint", joint_list);
//    }

}

Roboteq::~Roboteq()
{
    stop();
}

bool Roboteq::start()
{
    try
    {
        mSerial.setPort(mSerialPort);
        mSerial.open();
        mSerial.setBaudrate(mBaudrate);

        serial::Timeout to = serial::Timeout::simpleTimeout(mTimeout);
        mSerial.setTimeout(to);
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open serial port " << mSerialPort << " - Error: "  << e.what() );
        return false;
    }

    if(mSerial.isOpen()){
        ROS_DEBUG_STREAM("Serial Port correctly initialized: " << mSerialPort );
    }
    else
    {
        ROS_ERROR_STREAM( "Serial port not opened: " << mSerialPort );
        return false;
    }

    mStopping = false;

//    if(this->isAlive()){
//        ROS_DEBUG_STREAM("ORBUS Connection started: " << mSerialPort );
//    }
//    else
//    {
//        ROS_ERROR_STREAM("ORBUS does not found: " << mSerialPort );
//        return false;
//    }

    ROS_DEBUG_STREAM( "Serial port ready" );
    return true;
}

bool Roboteq::stop()
{
    // Stop the reader
    mStopping = true;
    // Close the serial port
    mSerial.close();
}

void Roboteq::initializeDiagnostic() {

}

void Roboteq::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {

}

}
