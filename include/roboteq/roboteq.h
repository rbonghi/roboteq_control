#ifndef ROBOTEQ_H
#define ROBOTEQ_H

#include <ros/ros.h>
#include <serial/serial.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "roboteq/serial_controller.h"

using namespace std;

namespace roboteq
{

class Roboteq : public diagnostic_updater::DiagnosticTask
{
public:
    /**
     * @brief serial_controller Open the serial controller
     * @param port set the port
     * @param set the baudrate
     */
    Roboteq(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, serial_controller *serial);

    ~Roboteq();

    void initializeDiagnostic();

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

private:
    //Initialization object
    //NameSpace for bridge controller
    ros::NodeHandle mNh;
    ros::NodeHandle private_mNh;
    // Serial controller
    serial_controller *mSerial;
    // Diagnostic
    diagnostic_updater::Updater diagnostic_updater;

};

}


#endif // ROBOTEQ_H
