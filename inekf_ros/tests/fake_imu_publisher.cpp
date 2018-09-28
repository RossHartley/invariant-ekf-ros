/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   fake_imu_publisher.cpp
 *  @author Ross Hartley
 *  @brief  Publishes fake imu data over the /imu topic
 *  @date   September 27, 2018
 **/

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <random>

using namespace std;

/**
 * Publishes fake IMU data over the /imu topic
 */
int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "fake_imu_publisher");
    ros::NodeHandle n;
    // Create Fake IMU publisher
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 1000);

    // Specify publishing frequency
    ros::NodeHandle nh("~");
    float rate = 100;
    nh.getParam("rate", rate);
    cout << "Publishing IMU at " << rate << " Hz." << endl;
    ros::Rate loop_rate(rate);

    // Initialize random number generator
    default_random_engine generator;
    double gyro_std = 0.01;
    nh.getParam("noise/gyroscope_std", gyro_std);
    cout << "Gyroscope noise std: " << gyro_std << endl;
    normal_distribution<double> gyro_noise(0,gyro_std);
    double accel_std = 0.1;
    nh.getParam("noise/accelerometer_std", accel_std);
    cout << "Accelerometer noise std: " << accel_std << endl;
    normal_distribution<double> accel_noise(0,accel_std);

    uint32_t seq = 0;
    while (ros::ok()) {
        // Construct IMU message
        sensor_msgs::Imu msg;

        msg.header.seq = seq;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/imu"; 

        msg.orientation.w = 1;
        msg.orientation.x = 0;
        msg.orientation.y = 0;
        msg.orientation.z = 0;

        msg.angular_velocity.x = 0 + gyro_noise(generator);
        msg.angular_velocity.y = 0 + gyro_noise(generator);
        msg.angular_velocity.z = 0 + gyro_noise(generator);

        msg.linear_acceleration.x = 0 + accel_noise(generator);
        msg.linear_acceleration.y = 0 + accel_noise(generator);    
        msg.linear_acceleration.z = 9.81 + accel_noise(generator);

        // Send message
        imu_pub.publish(msg);

        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
        seq++;
    }


    return 0;
}