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

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>
#include <signal.h>

#include "roboteq/serial_controller.h"
#include "roboteq/roboteq.h"

#include <boost/chrono.hpp>
#include <boost/algorithm/algorithm.hpp>

using namespace std;

typedef boost::chrono::steady_clock time_source;

ros::Timer control_loop;
ros::Timer diagnostic_loop;

roboteq::serial_controller *rSerial;

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
void controlLoop(roboteq::Roboteq &roboteq,
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
    roboteq.read(ros::Time::now(), elapsed);
    cm.update(ros::Time::now(), elapsed);
    roboteq.write(ros::Time::now(), elapsed);
}

/**
* Diagnostics loop for ORB boards, not realtime safe
*/
void diagnosticLoop(roboteq::Roboteq &roboteq)
{
    //ROS_INFO_STREAM("DIAGNOSTIC - running");
    roboteq.updateDiagnostics();
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
        // that interface with RoboTeq hardware.
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
