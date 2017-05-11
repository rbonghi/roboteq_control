/**
 * Copyright (C) 2017, Raffaello Bonghi <raffaello@rnext.it>
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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

    string getQuery(string msg, string params="")
    {
        if(query(msg, params))
        {
            return get();
        }
        else
        {
            return "";
        }
    }

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

    bool maintenance(string msg, string params="")
    {
        return command(msg, params, "%");
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
     * @brief reset Reset the Roboteq board
     */
    void reset()
    {
        // Send reset command
        mSerial.write("%RESET 321654987");
        // Wait one second after reset
        ros::Duration(1).sleep();
    }

    /**
     * @brief factoryReset Factory reset of Roboteq board
     * @return the status of write
     */
    bool factoryReset()
    {
        return maintenance("EERST");
    }
    /**
     * @brief loadFromEEPROM
     * @return the status of write
     */
    bool loadFromEEPROM()
    {
        return maintenance("EELD");
    }

    /**
     * @brief saveInEEPROM The %EESAV it's a real-time Command must be used to copy the RAM array to Flash. The Flash is copied to RAM every time the device powers up.
     * @return the status of write
     */
    bool saveInEEPROM() {
        return maintenance("EESAV");
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
     * @brief downloadScript Launch the script update for the Roboteq board
     * @return Return true if the script is fully updated
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
