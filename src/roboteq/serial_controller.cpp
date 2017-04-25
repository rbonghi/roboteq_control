
#include "roboteq/serial_controller.h"

#include <regex>

namespace roboteq {

const std::string eol("\r");
const size_t max_line_length(128);
const std::regex rgx_query("(.+)=(.+)\r");
const std::regex rgx_cmd("(\\+|-)\r");

serial_controller::serial_controller(string port, unsigned long baudrate)
    : mSerialPort(port)
    , mBaudrate(baudrate)
{
    // Default timeout
    mTimeout = 500;


    unlock = true;
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

bool serial_controller::addCallback(const callback_data_t &callback, const string data)
{
    if (hashmap.find(data) != hashmap.end())
    {
        return false;
    } else
    {
        hashmap[data] = callback;
        return true;
    }
}

bool serial_controller::command(string msg) {
    return false;
}

bool serial_controller::query(string msg) {
    mMessage = msg;
    string msg2 = "?" + msg + eol;
    ROS_INFO_STREAM_NAMED("serial", "TX: " << msg);
    mSerial.write(msg2.c_str());
    while(unlock) {
        return true;
    }
    return false;
}

void serial_controller::async_reader()
{
    while (!mStopping) {
        // Read how many byte waiting to read
        ROS_INFO_STREAM_NAMED("serial", "Bytes waiting: " << mSerial.available());
        // Read line
        std::string msg = mSerial.readline(max_line_length, eol);
        // Decode message
        if (!msg.empty())
        {
          ROS_INFO_STREAM_NAMED("serial", "RX: " << msg);
          if (std::regex_match(msg, rgx_cmd))
          {
              ROS_INFO("Command message");
              // TODO
          }
          else if(std::regex_match(msg, rgx_query))
          {
              // Get command
              string sub_cmd = msg.substr(0, msg.find('='));
              // Get data
              sub_data = msg.substr(msg.find('=') + 1);
              ROS_INFO_STREAM("CMD=" << sub_cmd << " DATA=" << sub_data);
              // Check first of all a message sent require a data to return
              if(mMessage.compare("") != 0) {
                  if(mMessage.compare(sub_cmd) == 0) {
                      ROS_INFO("Return a data");
                      // Clear last query request
                      mMessage = "";
                      // Unlock query request
                      unlock = false;
                      // Skip other request
                      continue;
                  }
              }
              // Find in all callback a data to send
              if (hashmap.find(sub_cmd) != hashmap.end())
              {
                  ROS_INFO("Launch callback");
                  // Get callback from hashmap
                  callback_data_t callback = hashmap[sub_cmd];
                  // Launch callback with return query
                  callback(sub_data);
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
