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


ros::Timer control_loop;
ros::Timer diagnostic_loop;

roboteq::serial_controller *rSerial;

bool status = true;

// >>>>> Ctrl+C handler
void siginthandler(int param)
{
    ROS_INFO("User pressed Ctrl+C Shutting down...");
    rSerial->stop();
    ROS_INFO("Control and diagnostic loop stopped");
    ROS_INFO_STREAM("--------- ROBOTEQ_NODE STOPPED ---------");
    ros::shutdown();

}
// <<<<< Ctrl+C handler

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
        roboteq::Roboteq roboteq(nh, private_nh, rSerial);

//        ROS_INFO_STREAM_NAMED("serial", "Bytes waiting: " << roboteq.mSerial.available());
//        std::string msg = roboteq.mSerial.readline(max_line_length, eol);
//        if (!msg.empty()) {
//          ROS_INFO_STREAM_NAMED("serial", "RX: " << msg);
//        }

        std::string name_node = ros::this_node::getName();
        ROS_INFO("Started %s", name_node.c_str());

        // Process remainder of ROS callbacks separately, mainly ControlManager related
        ros::spin();
    } else {

        ROS_ERROR_STREAM("Error connection, shutting down");
    }
    return 0;

}
