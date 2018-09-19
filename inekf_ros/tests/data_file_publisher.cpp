#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "inekf_msgs/LandmarkArray.h"
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <string>

using namespace std;

/**
 * Publishes fake IMU and landmark data over the /imu and /landmarks topics
 */
int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "data_file_publisher");
    ros::NodeHandle n;
    // Create Fake publishers
    ros::Publisher landmark_pub = n.advertise<inekf_msgs::LandmarkArray>("/landmarks", 1000);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 1000);

    // Open Data file
    ros::NodeHandle nh("~");
    string filename;
    if (nh.getParam("data_file", filename)) {
        cout << "Opening data file:  " << filename << endl;
    } else {
        ROS_ERROR("No data_file parameter set.");
        return 0;
    }

    ifstream infile(filename);
    string line;
    double t, t_last;
    uint32_t seq = 0;
    t_last = 0;

    // Loop through data file and read in measurements line by line
    while (getline(infile, line)){
        vector<string> measurement;
        boost::split(measurement,line,boost::is_any_of(" "));
        if (measurement[0].compare("IMU")==0){
            t = stod(measurement[1]);
            sensor_msgs::Imu msg;
            msg.header.seq = seq;
            msg.header.stamp = ros::Time(stod(measurement[1]));
            msg.header.frame_id = "/imu"; 
            msg.orientation.w = 1;
            msg.orientation.x = 0;
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
            inekf_msgs::LandmarkArray msg;
            t = stod(measurement[1]);
            msg.header.seq = seq;
            msg.header.stamp = ros::Time(stod(measurement[1]));
            msg.header.frame_id = "/imu"; 
            for (int i=2; i<measurement.size(); i+=4) {
                inekf_msgs::Landmark landmark;
                landmark.id = stoi(measurement[i]);
                landmark.position.x = stod(measurement[i+1]);
                landmark.position.y = stod(measurement[i+2]);
                landmark.position.z = stod(measurement[i+3]);
                msg.landmarks.push_back(landmark);
            }
            ros::Duration(t-t_last).sleep();
            landmark_pub.publish(msg);
        }
        // Spin and sleep
        ros::spinOnce();
        seq++;
        t_last = t;
        ROS_INFO("Time: %f", t);
    }


    return 0;
}