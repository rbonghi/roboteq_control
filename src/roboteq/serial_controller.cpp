
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
    mWriteMutex.lock();
    data = false;
    string msg2 = "!" + msg + eol;

    unsigned int counter = 0;
    while (counter < 5)
    {
        mSerial.write(msg2.c_str());
        data = false;
        // Set lock variable and wait a data to return
        std::unique_lock<std::mutex> lck(mReaderMutex);
        cv.wait_for(lck, std::chrono::seconds(1));
        if(data)
        {
            ROS_DEBUG_STREAM("N:" << (counter+1) << " CMD:" << msg << " DATA:" << sub_data_cmd);
            break;
        } else
        {
            // Increase counter
            counter++;
        }
    }
    // Unlock mutex
    mWriteMutex.unlock();
    return sub_data_cmd;
}

bool serial_controller::query(string msg, string type) {
    mWriteMutex.lock();
    mMessage = msg;
    string msg2 = type + msg + eol;

    unsigned int counter = 0;
    while (counter < 5)
    {
        ROS_DEBUG_STREAM("N:" << (counter+1) << " TX: " << msg);
        mSerial.write(msg2.c_str());
        data = false;
        // Set lock variable and wait a data to return
        std::unique_lock<std::mutex> lck(mReaderMutex);
        cv.wait_for(lck, std::chrono::seconds(1));
        if(data)
        {
            ROS_DEBUG_STREAM("N:" << (counter+1) << " CMD:" << msg << " DATA:" << sub_data);
            break;
        } else
        {
            // Increase counter
            counter++;
        }
    }
    // Clear last query request
    mMessage = "";
    // Unlock mutex
    mWriteMutex.unlock();
    return data;
}

void serial_controller::async_reader()
{
    while (!mStopping) {
        // Read how many byte waiting to read
        ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << mSerial.available());
        // Read line
        std::string msg = mSerial.readline(max_line_length, eol);
        // Decode message
        if (!msg.empty())
        {
          ROS_DEBUG_STREAM_NAMED("serial", "RX: " << msg);
          if (std::regex_match(msg, rgx_cmd))
          {
              // Decode if command return true
              if(msg[0] == '+') sub_data_cmd = true;
              else sub_data_cmd = false;
              // Unlock command
              data = true;
              // Unlock query request
              cv.notify_one();
          }
          else if(std::regex_match(msg, rgx_query))
          {
              // Get command
              string sub_cmd = msg.substr(0, msg.find('='));
              // Get data
              sub_data = msg.substr(msg.find('=') + 1);
              // ROS_INFO_STREAM("CMD=" << sub_cmd << " DATA=" << sub_data);
              // Check first of all a message sent require a data to return
              if(mMessage.compare("") != 0) {
                  if(mMessage.compare(sub_cmd) == 0) {
                      data = true;
                      // Unlock query request
                      cv.notify_one();
                      // Skip other request
                      continue;
                  }
              }
              // Find in all callback a data to send
              if (hashmap.find(sub_cmd) != hashmap.end())
              {
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
    ROS_INFO("Async serial reader closed");
}

}
