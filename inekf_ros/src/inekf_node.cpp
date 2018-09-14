#include "ros/ros.h"
#include "InEKF_ROS.h"
#include <chrono>

using namespace std;
using namespace inekf;

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "inekf_node");
    ros::NodeHandle n;    

    // // InEKF filter; 
    // int N = 10000;
    // InEKF filter;
    // cout << filter.getNoiseParams() << endl;
    // cout << filter.getState() << endl;
    // Eigen::Matrix<double,6,1> m; m << 0,0,0, 0,0,9.81;
    // double dt = 0.01;
    // cout << "Propagating " << N << " IMU measurements...\n";
    // int64_t max_duration = 0;
    // int64_t sum_duration = 0;
    // for(int i=0; i<N; ++i){
    //     chrono::high_resolution_clock::time_point start_time = chrono::high_resolution_clock::now();
    //     filter.Propagate(m, dt);
    //     chrono::high_resolution_clock::time_point end_time = chrono::high_resolution_clock::now();
    //     auto duration = chrono::duration_cast<chrono::microseconds>( end_time - start_time ).count();
    //     sum_duration += duration;
    //     if (duration > max_duration)
    //         max_duration = duration;
    //     //cout << filter.getState() << endl;
    // }
    // cout << "max duration: " <<  max_duration << endl;
    // cout << "average duration: " <<  double(sum_duration)/double(N) << endl;

    // Initialize InEKF ROS wrapper
    InEKF_ROS inekf_wrapper(n);
    inekf_wrapper.init();
    inekf_wrapper.run();

    return 0;
}
