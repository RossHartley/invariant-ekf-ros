#include "ros/ros.h"
#include "InEKF_ROS.h"

using namespace std;
using namespace inekf;

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "inekf_node");
    ros::NodeHandle n;    

    // Initialize InEKF ROS wrapper
    InEKF_ROS inekf_wrapper(n);
    inekf_wrapper.init();
    inekf_wrapper.run();

    return 0;
}
