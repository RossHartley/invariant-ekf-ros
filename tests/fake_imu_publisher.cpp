#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <sstream>

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
    ros::Rate loop_rate(1);

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

        msg.angular_velocity.x = 0;
        msg.angular_velocity.y = 0;
        msg.angular_velocity.z = 0;

        msg.linear_acceleration.x = 0;
        msg.linear_acceleration.y = 0;
        msg.linear_acceleration.z = 9.81;

        //stringstream ss;
        //ss << "hello world!";
        //ROS_INFO("%s", ss.str().c_str());

        // Send message
        imu_pub.publish(msg);

        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
        seq++;
    }


    return 0;
}