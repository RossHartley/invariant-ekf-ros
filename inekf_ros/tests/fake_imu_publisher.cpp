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
    float rate;
    if (nh.hasParam("rate")) {
        nh.getParam("rate", rate);
    } else {
        rate = 100;
    }
    cout << "Publishing IMU at " << rate << " Hz." << endl;
    ros::Rate loop_rate(rate);

    // Initialize random number generator
    default_random_engine generator;
    normal_distribution<double> distribution(0,0.1);

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

        msg.angular_velocity.x = 0 + distribution(generator);
        msg.angular_velocity.y = 0 + distribution(generator);
        msg.angular_velocity.z = 0 + distribution(generator);

        msg.linear_acceleration.x = 0 + distribution(generator);
        msg.linear_acceleration.y = 0 + distribution(generator);    
        msg.linear_acceleration.z = 9.81 + distribution(generator);

        // Send message
        imu_pub.publish(msg);

        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
        seq++;
    }


    return 0;
}