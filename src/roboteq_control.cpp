/*
 * Copyright (C) 2016 Officine Robotiche
 * Author: Raffaello Bonghi
 * email:  raffaello@rnext.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>
#include <signal.h>

#include "roboteq/serial_controller.h"
#include "roboteq/roboteq.h"

#include <boost/chrono.hpp>

using namespace std;

typedef boost::chrono::steady_clock time_source;

ros::Timer control_loop;
ros::Timer diagnostic_loop;

roboteq::serial_controller *rSerial;

bool status = true;

// >>>>> Ctrl+C handler
void siginthandler(int param)
{
    ROS_INFO("User pressed Ctrl+C Shutting down...");
    control_loop.stop();
    diagnostic_loop.stop();
    rSerial->stop();
    ROS_INFO("Control and diagnostic loop stopped");
    ROS_INFO_STREAM("--------- ROBOTEQ_NODE STOPPED ---------");
    ros::shutdown();

}
// <<<<< Ctrl+C handler
/**
* Control loop not realtime safe
*/
void controlLoop(roboteq::Roboteq &orb,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{

    // Calculate monotonic time difference
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    //ROS_INFO_STREAM("CONTROL - running");
    // Process control loop
    orb.read(ros::Time::now(), elapsed);
    cm.update(ros::Time::now(), elapsed);
    orb.write(ros::Time::now(), elapsed);
}

/**
* Diagnostics loop for ORB boards, not realtime safe
*/
void diagnosticLoop(roboteq::Roboteq &orb)
{
    //ROS_INFO_STREAM("DIAGNOSTIC - running");
//    bool diagnostic = orb.updateDiagnostics();
//    // Set true if the diagnostic change with the before status
//    //ROS_INFO_STREAM("Status:" << status << "- Diagnostic:" << diagnostic);
//    if(status != diagnostic)
//    {
//        if(diagnostic)
//        {
//            ROS_INFO_STREAM("DIAGNOSTIC - Initialize again the unav and restart control loop");
//            orb.initialize();
//            control_loop.start();
//        }
//        else
//        {
//            // Stopping control node
//            ROS_ERROR_STREAM("DIAGNOSTIC - Stop control loop");
//            control_loop.stop();
//        }
//    }
//    status = diagnostic;
//    //ROS_INFO_STREAM("New status:" << status);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "roboteq_control");
    ros::NodeHandle nh, private_nh("~");

    signal(SIGINT, siginthandler);
    ROS_INFO_STREAM("----------------------------------------");
    ROS_INFO_STREAM("------------- ROBOTEQ_NODE -------------");
    //Hardware information
    double control_frequency, diagnostic_frequency;
    private_nh.param<double>("control_frequency", control_frequency, 1.0);
    private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);
    ROS_INFO_STREAM("Control:" << control_frequency << "Hz - Diagnostic:" << diagnostic_frequency << "Hz");

    string serial_port_string;
    int32_t baud_rate;

    private_nh.param<string>("serial_port", serial_port_string, "/dev/ttyACM0");
    private_nh.param<int32_t>("serial_rate", baud_rate, 115200);
    ROS_INFO_STREAM("Open Serial " << serial_port_string << ":" << baud_rate);

    rSerial = new roboteq::serial_controller(serial_port_string, baud_rate);
    // Run the serial controller
    bool start = rSerial->start();
    // Check connection started
    if(start) {
        // Initialize roboteq controller
        roboteq::Roboteq interface(nh, private_nh, rSerial);
        // Initialize the motor parameters
        interface.initialize();
        //Initialize all interfaces and setup diagnostic messages
        interface.initializeInterfaces();

        controller_manager::ControllerManager cm(&interface, nh);

        // Setup separate queue and single-threaded spinner to process timer callbacks
        // that interface with uNav hardware.
        // This avoids having to lock around hardware access, but precludes realtime safety
        // in the control loop.
        ros::CallbackQueue unav_queue;
        ros::AsyncSpinner unav_spinner(1, &unav_queue);

        time_source::time_point last_time = time_source::now();
        ros::TimerOptions control_timer(
                    ros::Duration(1 / control_frequency),
                    boost::bind(controlLoop, boost::ref(interface), boost::ref(cm), boost::ref(last_time)),
                    &unav_queue);
        // Global variable
        control_loop = nh.createTimer(control_timer);

        ros::TimerOptions diagnostic_timer(
                    ros::Duration(1 / diagnostic_frequency),
                    boost::bind(diagnosticLoop, boost::ref(interface)),
                    &unav_queue);
        diagnostic_loop = nh.createTimer(diagnostic_timer);

        unav_spinner.start();

        std::string name_node = ros::this_node::getName();
        ROS_INFO("Started %s", name_node.c_str());

        // Process remainder of ROS callbacks separately, mainly ControlManager related
        ros::spin();
    } else {

        ROS_ERROR_STREAM("Error connection, shutting down");
    }
    return 0;

}
