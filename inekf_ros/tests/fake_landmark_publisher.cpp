#include "ros/ros.h"
#include "inekf_msgs/Landmark.h"
#include "inekf_msgs/LandmarkArray.h"

using namespace std;

/**
 * Publishes fake landmark data over the /landmarks topic
 */
int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "fake_landmark_publisher");
    ros::NodeHandle n;
    // Create Fake landmark publisher
    ros::Publisher imu_pub = n.advertise<inekf_msgs::Landmark>("/landmarks", 1000);

    // Specify publishing frequency
    ros::NodeHandle nh("~");
    float rate;
    if (nh.hasParam("rate")) {
        nh.getParam("rate", rate);
    } else {
        rate = 1;
    }
    cout << "Publishing landmarks at " << rate << " Hz." << endl;
    ros::Rate loop_rate(rate);

    uint32_t seq = 0;
    while (ros::ok()) {
        // Construct landmark message
        inekf_msgs::Landmark msg;

        msg.header.seq = seq;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/imu"; 

        msg.id = 0;
        msg.position.x = 0;
        msg.position.y = 0;
        msg.position.z = 0;

        // Send message
        imu_pub.publish(msg);

        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
        seq++;
    }


    return 0;
}