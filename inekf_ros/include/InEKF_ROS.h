/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF_ROS.h
 *  @author Ross Hartley
 *  @brief  Header file for a ROS wrapper of the Invariant EKF 
 *  @date   September 27, 2018
 **/

#ifndef INEKF_ROS_H
#define INEKF_ROS_H 
#include <Eigen/Dense>
#include <memory>
#include <chrono>
#include <thread>
#include <string>
#include <boost/lockfree/queue.hpp>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "InEKF.h"
#include "Measurement.h"
#include "Queue.h"
#include "inekf_msgs/State.h"
#include "visualization_msgs/MarkerArray.h"
#include <mutex>
#include <random>

#define QUEUE_BUFFER_SIZE 50
#define MAX_QUEUE_SIZE 200


class InEKF_ROS {
    public:
        InEKF_ROS(ros::NodeHandle n);
        void init();
        void run();

    private: 
        ros::NodeHandle n_;
        inekf::InEKF filter_;
        ros::Subscriber imu_sub_;
        ros::Subscriber landmarks_sub_;
        ros::Subscriber kinematics_sub_;
        ros::Subscriber contact_sub_;

        uint32_t seq_ = 0;
        ros::Publisher pose_pub_;
        ros::Publisher state_pub_;
        ros::Publisher visualization_pub_;
        tf::TransformBroadcaster tf_broadcaster_;

        std::thread filtering_thread_;
        Queue<std::shared_ptr<Measurement>, std::vector<std::shared_ptr<Measurement>>, MeasurementCompare> m_queue_;

        std::string base_frame_id_;
        std::string imu_frame_id_;
        std::string map_frame_id_;
        
        bool enable_landmarks_;
        bool enable_kinematics_;
        bool publish_visualization_markers_;
        bool enabled_;
        bool bias_initialized_;
        bool initialize_state_from_first_observation_;
        bool static_bias_initialization_;
        bool flat_ground_;

        geometry_msgs::Point point_prev_;    
        tf::StampedTransform imu_to_base_transform_;
        tf::StampedTransform imu_to_camera_transform_;

        double t_;
        double dt_;
        double t_prev_;
        std::shared_ptr<ImuMeasurement> imu_prev_;
        Eigen::Vector3d bg0_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d ba0_ = Eigen::Vector3d::Zero();

        Eigen::Matrix3d observation_covariance_landmark_ = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d observation_covariance_magnetometer_ = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d observation_covariance_position_ = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d observation_covariance_contact_position_ = Eigen::Matrix3d::Identity();

        bool enabled();
        bool biasInitialized();
        void subscribe();
        void initBias();
        void initState();
        void update();
        void mainFilteringThread();

        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg); 
        void landmarkCallback(const inekf_msgs::LandmarkArray::ConstPtr& msg);
        void kinematicsCallback(const inekf_msgs::KinematicsArray::ConstPtr& msg);
        void contactCallback(const inekf_msgs::ContactArray::ConstPtr& msg);

        void publish();
        void publishLandmarkMeasurementMarkers(std::shared_ptr<LandmarkMeasurement> ptr);
        void publishKinematicMeasurementMarkers(std::shared_ptr<KinematicMeasurement> ptr);

};

#endif 
