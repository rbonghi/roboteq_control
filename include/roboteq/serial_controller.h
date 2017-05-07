#ifndef SERIAL_CONTROLLER_H
#define SERIAL_CONTROLLER_H

#include <ros/ros.h>
#include <serial/serial.h>

#include <mutex>
#include <condition_variable>  // std::condition_variable

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

    bool command(string msg, string params="", string type="!");

    bool query(string msg, string params="", string type="?");

    bool setParam(string msg, string params="") {
        return command(msg, params, "^");
    }

    string getParam(string msg, string params="") {
        if(query(msg, params, "~"))
        {
            return get();
        }
        else
        {
            return "";
        }
    }

    /**
     * @brief get Get the message parsed
     * @return Return the string received
     */
    string get()
    {
        return sub_data;
    }
    /**
     * @brief getVersionScript The version of the script loaded
     * @return The string of roboteq control version
     */
    string getVersionScript()
    {
        return "V" + _script_ver;
    }

    /**
     * @brief script Run and stop the script inside the Roboteq
     * @param status The status of the script
     * @return the status of command
     */
    bool script(bool status) {
        if(status)
        {
            return command("R");
        } else
        {
            return command("R", "0");
        }
    }
    /**
     * @brief echo Enable or disable the echo message
     * @param type status of echo
     * @return The status of command
     */
    bool echo(bool type) {
        if(type) {
            return setParam("ECHOF", "0");
        } else
        {
            return setParam("ECHOF", "1");
        }
    }
    /**
     * @brief downloadScript
     * @return
     */
    bool downloadScript();
    /**
     * @brief addCallback Add callback message
     * @param callback The callback function
     * @param type The type of message to check
     */
    bool addCallback(const callback_data_t &callback, const string data);
    /**
     * Template to connect a method in callback
     */
    template <class T> bool addCallback(void(T::*fp)(const string), T* obj, const string data) {
        return addCallback(bind(fp, obj, _1), data);
    }
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
    bool sub_data_cmd;
    bool data;
    // Async reader controller
    std::thread first;
    // Mutex to sto concurent sending
    mutex mWriteMutex;
    mutex mReaderMutex;
    std::condition_variable cv;
    // Hashmap with all type of message
    map<string, callback_data_t> hashmap;
    // HLD mode - To download script - reference [pag. 183]
    bool isHLD;
    // Version script
    string _script_ver;
    /**
     * @brief async_reader Thread to read realtime all charachters sent from roboteq board
     */
    void async_reader();
    /**
     * @brief enableDownload Enable writing script
     * @return Status of HLD reference [pag. 183]
     */
    bool enableDownload();
};

}

#endif // SERIAL_CONTROLLER_H
