/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   data_file_publisher.cpp
 *  @author Ross Hartley
 *  @brief  Publishes data from a text file
 *  @date   September 27, 2018
 **/

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "inekf_msgs/LandmarkArray.h"
#include "inekf_msgs/ContactArray.h"
#include "inekf_msgs/KinematicsArray.h"
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <string>

using namespace std;

/**
 * Publishes IMU, landmark, contact, and kinematic data from text file
 */
int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "data_file_publisher");
    ros::NodeHandle n;
    // Create Fake publishers
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 1000);
    ros::Publisher landmark_pub = n.advertise<inekf_msgs::LandmarkArray>("/landmarks", 1000);
    ros::Publisher contact_pub = n.advertise<inekf_msgs::ContactArray>("/contacts", 1000);
    ros::Publisher kinematics_pub = n.advertise<inekf_msgs::KinematicsArray>("/kinematics", 1000);

    // Open Data file
    ros::NodeHandle nh("~");
    string filename;
    if (nh.getParam("data_file", filename)) {
        ROS_INFO("Opening data file: %s", filename.c_str());
    } else {
        ROS_ERROR("No data_file parameter set.");
        return 0;
    }

    ifstream infile(filename);
    string line;
    double t, t_last;
    uint32_t seq = 0;
    t_last = 0;

    // int frame_count = 0;
    // const int num_frames_kept = 10; 
    // int id_offset = 0;
    // const int max_landmarks = 10;

    // Loop through data file and read in measurements line by line
    while (getline(infile, line)){
        vector<string> measurement;
        boost::split(measurement,line,boost::is_any_of(" "));

        // Handle measurement
        if (measurement[0].compare("IMU")==0){
            ROS_INFO("Received IMU Measurement");
            assert((measurement.size()-2) == 6);
            t = stod(measurement[1]);
            sensor_msgs::Imu msg;
            msg.header.seq = seq;
            msg.header.stamp = ros::Time(t);
            msg.header.frame_id = "/imu"; 
            msg.orientation.w = 0;
            msg.orientation.x = 1;
            msg.orientation.y = 0;
            msg.orientation.z = 0;
            msg.angular_velocity.x = stod(measurement[2]);
            msg.angular_velocity.y = stod(measurement[3]);
            msg.angular_velocity.z = stod(measurement[4]);
            msg.linear_acceleration.x = stod(measurement[5]);
            msg.linear_acceleration.y = stod(measurement[6]);
            msg.linear_acceleration.z = stod(measurement[7]);
            ros::Duration(t-t_last).sleep();
            imu_pub.publish(msg);
        }
        else if (measurement[0].compare("LANDMARK")==0){
            ROS_INFO("Received LANDMARK Measurement");
            assert((measurement.size()-2)%4 == 0);
            inekf_msgs::LandmarkArray msg;
            t = stod(measurement[1]);
            msg.header.seq = seq;
            msg.header.stamp = ros::Time(t);
            msg.header.frame_id = "/imu"; 
            for (int i=2; i<measurement.size(); i+=4) {
                inekf_msgs::VectorWithId landmark;
                landmark.id = stoi(measurement[i]); //+ id_offset;
                landmark.position.x = stod(measurement[i+1]);
                landmark.position.y = stod(measurement[i+2]);
                landmark.position.z = stod(measurement[i+3]);
                msg.landmarks.push_back(landmark);
            }
            ros::Duration(t-t_last).sleep();
            landmark_pub.publish(msg);
            // frame_count++;
            // if (frame_count%num_frames_kept==0) {
            //     id_offset+=max_landmarks;
            // }
        }
        else if (measurement[0].compare("CONTACT")==0){
            ROS_INFO("Received CONTACT Measurement");
            assert((measurement.size()-2)%2 == 0);
            inekf_msgs::ContactArray msg;
            t = stod(measurement[1]); 
            msg.header.seq = seq;
            msg.header.stamp = ros::Time(t);
            msg.header.frame_id = ""; 
            for (int i=2; i<measurement.size(); i+=2) {
                inekf_msgs::Contact contact;
                contact.id = stoi(measurement[i]);
                contact.indicator = bool(stod(measurement[i+1]));
                msg.contacts.push_back(contact);
            }
            ros::Duration(t-t_last).sleep();
            contact_pub.publish(msg);
        }
        else if (measurement[0].compare("KINEMATIC")==0){
            ROS_INFO("Received KINEMATIC Measurement");               
            assert((measurement.size()-2)%44 == 0);
            inekf_msgs::KinematicsArray msg;
            t = stod(measurement[1]); 
            msg.header.seq = seq;
            msg.header.stamp = ros::Time(t);
            msg.header.frame_id = "/imu"; 
            for (int i=2; i<measurement.size(); i+=44) {
                inekf_msgs::Kinematics frame;
                frame.id = stoi(measurement[i]);
                frame.pose.pose.orientation.w = stod(measurement[i+1]);    
                frame.pose.pose.orientation.x = stod(measurement[i+2]);    
                frame.pose.pose.orientation.y = stod(measurement[i+3]);    
                frame.pose.pose.orientation.z = stod(measurement[i+4]);    
                frame.pose.pose.position.x = stod(measurement[i+5]);
                frame.pose.pose.position.y = stod(measurement[i+6]);
                frame.pose.pose.position.z = stod(measurement[i+7]);
                boost::array<double, 36> cov;
                for (int j=0; j<6; ++j) {
                    for (int k=0; k<6; ++k) {
                        cov[j*6+k] = stod(measurement[i+8 + j*6+k]);
                    }
                }
                frame.pose.covariance = cov;
                msg.frames.push_back(frame);
            }
            ros::Duration(t-t_last).sleep();
            kinematics_pub.publish(msg);
        }


        // Spin and sleep
        ros::spinOnce();
        seq++;
        t_last = t;
        ROS_INFO("Time: %f", t);
    }

    ROS_INFO("End of data stream");
    return 0;
}