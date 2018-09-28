/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   inekf_node.cpp
 *  @author Ross Hartley
 *  @brief  Creates ROS node that runs the invariant-ekf
 *  @date   September 27, 2018
 **/

#include "ros/ros.h"
#include "InEKF_ROS.h"
#include <chrono>

using namespace std;
using namespace inekf;

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "inekf_node");
    ros::NodeHandle n;    

    // Initialize InEKF ROS wrapper
    InEKF_ROS inekf_wrapper(n);
    inekf_wrapper.init();
    inekf_wrapper.run();

    return 0;
}
