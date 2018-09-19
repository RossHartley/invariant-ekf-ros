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

#define MAX_QUEUE_SIZE 100

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
        std::thread filtering_thread_;
        std::thread output_thread_;
        Queue<std::shared_ptr<Measurement>> m_queue_;
        tf::StampedTransform camera_to_imu_transform_;
        std::string imu_frame_id_;
        std::string map_frame_id_;
        bool publish_landmark_measurement_markers_;
        ros::Publisher landmark_measurement_vis_pub_;
        bool publish_landmark_position_markers_;
        bool publish_trajectory_markers_;


        void subscribe();
        void mainFilteringThread();
        void outputPublishingThread();
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg); 
        void landmarkCallback(const inekf_msgs::LandmarkArray::ConstPtr& msg);
        void publishLandmarkMeasurementMarkers(std::shared_ptr<LandmarkMeasurement> ptr);
};

#endif 
