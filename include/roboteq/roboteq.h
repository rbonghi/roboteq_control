#ifndef ROBOTEQ_H
#define ROBOTEQ_H

#include <ros/ros.h>
#include <serial/serial.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <mutex>

using namespace std;

namespace roboteq
{

typedef enum serial_status
{
    SERIAL_OK,
    SERIAL_TIMEOUT,
    SERIAL_EMPTY,
    SERIAL_BUFFER_FULL,
    SERIAL_IOEXCEPTION,
    SERIAL_EXCEPTION

} serial_status_t;

class Roboteq : public diagnostic_updater::DiagnosticTask
{
public:
    /**
     * @brief serial_controller Open the serial controller
     * @param port set the port
     * @param set the baudrate
     */
    Roboteq(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, string port, unsigned long baudrate);

    ~Roboteq();
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

    void initializeDiagnostic();

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

    // Serial port object
    serial::Serial mSerial;

private:
    //Initialization object
    //NameSpace for bridge controller
    ros::NodeHandle mNh;
    ros::NodeHandle private_mNh;
    // Serial port name
    string mSerialPort;
    // Serial port baudrate
    uint32_t mBaudrate;
    // Timeout open serial port
    uint32_t mTimeout;
    // Used to stop the serial processing
    bool mStopping;
    // Status of the serial communication
    serial_status_t mStatus;
    // Diagnostic
    diagnostic_updater::Updater diagnostic_updater;

};

}


#endif // ROBOTEQ_H
