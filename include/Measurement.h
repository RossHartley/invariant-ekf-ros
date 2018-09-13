#ifndef MEASUREMENT_H
#define MEASUREMENT_H 
#include <Eigen/Dense>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

enum MeasurementType {EMPTY, IMU, LANDMARK};

class Measurement {

    public:
        Measurement();
        Measurement(const sensor_msgs::Imu::ConstPtr& msg);
        double getTime();
        Eigen::VectorXd getData();
        MeasurementType getType();
    
        friend std::ostream& operator<<(std::ostream& os, const Measurement& m);  

    private:
        double t_;
        Eigen::VectorXd data_;
        MeasurementType type_;
};

#endif 
