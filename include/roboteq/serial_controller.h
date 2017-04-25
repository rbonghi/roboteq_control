#ifndef SERIAL_CONTROLLER_H
#define SERIAL_CONTROLLER_H

#include <ros/ros.h>
#include <serial/serial.h>

#include <mutex>
#include <thread>

using namespace std;

namespace roboteq {

/// Read complete callback - Array of callback
typedef function<void (string data) > callback_data_t;

class serial_controller
{
public:
    /**
     * @brief serial_controller Open the serial controller
     * @param port set the port
     * @param set the baudrate
     */
    serial_controller(string port, unsigned long baudrate);

    ~serial_controller();
    /**
     * @brief start Initialize the serial communcation
     * @return if open the connection return true
     */
    bool start();
    /**
     * @brief stop
     * @return
     */
    bool stop();

    bool command(string msg);

    bool query(string msg);

    // Async reader from serial
    void async_reader();
    /**
     * @brief addCallback
     * @param callback
     * @param type
     */
    bool addCallback(const callback_data_t &callback, const string data);

protected:

private:
    // Serial port object
    serial::Serial mSerial;
    // Serial port name
    string mSerialPort;
    // Serial port baudrate
    uint32_t mBaudrate;
    // Timeout open serial port
    uint32_t mTimeout;
    // Used to stop the serial processing
    bool mStopping;
    // Last message sent
    string mMessage;
    string sub_data;
    bool unlock;
    // Async reader controller
    std::thread first;

    // Hashmap with all type of message
    map<string, callback_data_t> hashmap;

};

}

#endif // SERIAL_CONTROLLER_H
