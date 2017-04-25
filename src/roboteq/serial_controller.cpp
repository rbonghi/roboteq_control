
#include "roboteq/serial_controller.h"

#include <regex>

namespace roboteq {

const std::string eol("\r");
const size_t max_line_length(128);

serial_controller::serial_controller(string port, unsigned long baudrate)
    : mSerialPort(port)
    , mBaudrate(baudrate)
{
    // Default timeout
    mTimeout = 500;
}

serial_controller::~serial_controller()
{
    stop();
}

bool serial_controller::start()
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
    // Initialize stop function
    mStopping = false;
    // Launch async reader thread
    first = std::thread(&serial_controller::async_reader, this);

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

bool serial_controller::stop()
{
    // Stop the reader
    mStopping = true;
    // Close the serial port
    mSerial.close();
    // Wait stop thread
    first.join();

}

bool serial_controller::send(string msg) {
    mMessage = msg;
    string msg2 = "?" + msg + eol;
    //ROS_INFO_STREAM_NAMED("serial", "TX: " << boost::algorithm::replace_all_copy(msg, eol, "\\r"));
    mSerial.write(msg2.c_str());

    return false;
}

void serial_controller::async_reader()
{
    while (!mStopping) {
        ROS_INFO_STREAM_NAMED("serial", "Bytes waiting: " << mSerial.available());
        std::string msg = mSerial.readline(max_line_length, eol);
        std::regex number("(.+)=(.+)\r");
        std::regex cmd("(\\+|-)\r");
        if (!msg.empty())
        {
          ROS_INFO_STREAM_NAMED("serial", "RX: " << msg);
          if (std::regex_match(msg, cmd))
          {
              ROS_INFO("Command message");
          }
          else if(std::regex_match(msg, number))
          {
              // Find matching message with request
              std::regex number(mMessage + "=(.+)\r");
              if(std::regex_match(msg, number))
              {
                  ROS_INFO("ECCOLA");
              }
          }
          else
          {
              ROS_INFO("Other message");
          }
        }
    }
    ROS_INFO("Exit");
}

}
