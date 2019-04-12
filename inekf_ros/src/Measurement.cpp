/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   Measurement.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Measurement class
 *  @date   September 27, 2018
 **/

#include "Measurement.h"
using namespace std;
using namespace inekf;

// Construct Empty Measurement
Measurement::Measurement() {
    t_ = 0;
    type_ = EMPTY;
}
// Getters
double Measurement::getTime() { return t_; }
MeasurementType Measurement::getType() { return type_; }

// Construct IMU measurement
ImuMeasurement::ImuMeasurement(const sensor_msgs::Imu::ConstPtr& msg) {
    t_ = msg->header.stamp.toSec();
    data_ << msg->angular_velocity.x, 
             msg->angular_velocity.y, 
             msg->angular_velocity.z,
             msg->linear_acceleration.x, 
             msg->linear_acceleration.y, 
             msg->linear_acceleration.z;
    Eigen::Quaternion<double> q(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);
    R_ = q.toRotationMatrix();
    type_ = IMU;
}
Eigen::VectorXd ImuMeasurement::getData() { return data_; }
Eigen::Matrix3d ImuMeasurement::getRotation() { return R_; }

// Construct Landmark measurement
LandmarkMeasurement::LandmarkMeasurement(const inekf_msgs::LandmarkArray::ConstPtr& msg, const tf::StampedTransform& transform, const Eigen::Matrix3d& covariance){
    t_ = msg->header.stamp.toSec();
    type_ = LANDMARK;
    for (auto it=msg->landmarks.begin(); it!=msg->landmarks.end(); ++it) {
        tf::Vector3 p_cl(it->position.x, it->position.y, it->position.z);
        tf::Vector3 p_bl = transform*p_cl; // Transform measurement from camera frame to imu frame
        Eigen::Vector3d position;
        position << p_bl.getX(), p_bl.getY(), p_bl.getZ();
        data_.push_back(Landmark(it->id, position, covariance)); 
    }
}
vectorLandmarks LandmarkMeasurement::getData() { return data_; }; 


// Construct Contact measurement
ContactMeasurement::ContactMeasurement(const inekf_msgs::ContactArray::ConstPtr& msg){
    t_ = msg->header.stamp.toSec();
    type_ = CONTACT;
    for (auto it=msg->contacts.begin(); it!=msg->contacts.end(); ++it) {
        data_.push_back(pair<int,bool> (it->id, it->indicator)); 
    }
}
vector<pair<int,bool>> ContactMeasurement::getData() { return data_; }; 


// Construct Kinematic measurement
KinematicMeasurement::KinematicMeasurement(const inekf_msgs::KinematicsArray::ConstPtr& msg){
    t_ = msg->header.stamp.toSec();
    type_ = KINEMATIC;
    for (auto it=msg->frames.begin(); it!=msg->frames.end(); ++it) {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        Eigen::Quaternion<double> quat(it->pose.pose.orientation.w, it->pose.pose.orientation.x, it->pose.pose.orientation.y, it->pose.pose.orientation.z);
        pose.block<3,3>(0,0) = quat.toRotationMatrix();
        pose(0,3) = it->pose.pose.position.x;
        pose(1,3) = it->pose.pose.position.y;
        pose(2,3) = it->pose.pose.position.z;
        Eigen::Matrix<double,6,6> covariance; // Assumed orientation first, the position
        for (int i=0; i<6; ++i) {
            for (int j=0; j<6; ++j) {
                covariance(i,j) = it->pose.covariance[6*i+j]; // Assume row-major
            }
        }
        data_.push_back(Kinematics(it->id, pose, covariance));
    }
}
vectorKinematics KinematicMeasurement::getData() { return data_; }; 

// Print measurement
ostream& operator<<(ostream& os, const Measurement& m) {
    string type_str;
    switch (m.type_) {
        case IMU :
            type_str = "IMU";
            break;
        default:
            type_str = "Unknown";
    }
    os << "Measurement type: " << type_str << endl;
    return os;
}
