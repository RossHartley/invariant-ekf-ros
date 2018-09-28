/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   fake_contact_kinematics_publisher.cpp
 *  @author Ross Hartley
 *  @brief  Publishes fake contact and kinematic data over the /contacts and /kinematics topic
 *  @date   September 27, 2018
 **/

#include "ros/ros.h"
#include "inekf_msgs/ContactArray.h"
#include "inekf_msgs/KinematicsArray.h"
#include <random>

using namespace std;

/**
 * Publishes fake contact and kinematics data over the /landmarks topic
 */
int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "fake_contact_kinematics_publisher");
    ros::NodeHandle n;
    // Create Fake ContactArray publisher
    ros::Publisher contact_pub = n.advertise<inekf_msgs::ContactArray>("/contacts", 1000);
    // Create Fake KinematicsArray publisher
    ros::Publisher kinematics_pub = n.advertise<inekf_msgs::KinematicsArray>("/kinematics", 1000);

    // Specify publishing frequency
    ros::NodeHandle nh("~");
    float rate = 1;
    nh.getParam("rate", rate);
    cout << "Publishing landmarks at " << rate << " Hz." << endl;
    ros::Rate loop_rate(rate);

    // Initialize random number generator
    // default_random_engine generator;
    // double landmark_std = 0.1;
    // nh.getParam("noise/landmark_std", landmark_std);
    // cout << "Landmark noise std: " << landmark_std << endl;
    // normal_distribution<double> landmark_noise(0,landmark_std);

    uint32_t seq = 0;
    while (ros::ok()) {

        // Construct and send ContactArray message
        inekf_msgs::ContactArray contact_msg;

        contact_msg.header.seq = seq;
        contact_msg.header.stamp = ros::Time::now();
        contact_msg.header.frame_id = ""; 

        inekf_msgs::Contact contact;
        contact.id = 0;
        if (seq < 1000)
            contact.indicator = false;
        else if (seq > 2500) 
            contact.indicator = false;
        else 
            contact.indicator = true;
        contact_msg.contacts.push_back(contact);

        contact.id = 1;
        if (seq < 2000)
            contact.indicator = false;
        else if (seq > 5000) 
            contact.indicator = false;
        else 
            contact.indicator = true;
        contact_msg.contacts.push_back(contact);
        contact_pub.publish(contact_msg);

        // Construct and send KinematicsArray message
        inekf_msgs::KinematicsArray kinematics_msg;

        kinematics_msg.header.seq = seq;
        kinematics_msg.header.stamp = ros::Time::now();
        kinematics_msg.header.frame_id = "/imu"; 

        inekf_msgs::Kinematics frame;
        frame.id = 0;
        frame.pose.pose.position.x = 0; 
        frame.pose.pose.position.y = -0.1;
        frame.pose.pose.position.z = -1;
        frame.pose.pose.orientation.w = 1;
        frame.pose.pose.orientation.x = 0;
        frame.pose.pose.orientation.y = 0;
        frame.pose.pose.orientation.z = 0;
        boost::array<double, 36> cov = { {0.001,0,0,0,0,0, 0,0.001,0,0,0,0, 0,0,0.001,0,0,0, 0,0,0,0.001,0,0, 0,0,0,0,0.001,0, 0,0,0,0,0,0.001} };
        frame.pose.covariance = cov;
        kinematics_msg.frames.push_back(frame);

        frame.id = 1;
        frame.pose.pose.position.x = 0;
        frame.pose.pose.position.y = 0.1;
        frame.pose.pose.position.z = -1;
        frame.pose.pose.orientation.w = 1;
        frame.pose.pose.orientation.x = 0;
        frame.pose.pose.orientation.y = 0;
        frame.pose.pose.orientation.z = 0;
        frame.pose.covariance = cov;
        kinematics_msg.frames.push_back(frame);
 
        kinematics_pub.publish(kinematics_msg);

        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
        seq++;
    }


    return 0;
}