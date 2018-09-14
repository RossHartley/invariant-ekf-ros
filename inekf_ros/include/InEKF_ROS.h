#ifndef INEKF_ROS_H
#define INEKF_ROS_H 
#include <Eigen/Dense>
#include <memory>
#include <chrono>
#include <thread>
#include <boost/lockfree/queue.hpp>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include "InEKF.h"
#include "Measurement.h"
#include "Queue.h"

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
        std::thread filtering_thread_;
        std::thread output_thread_;
        Queue<std::shared_ptr<Measurement>> m_queue_;

        //std::mutex imu_mutex_;

        void subscribe();
        void mainFilteringThread();
        void outputPublishingThread();
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg); 
};

#endif 
