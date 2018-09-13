#include "Measurement.h"
using namespace std;

// Construct Empty Measurement
Measurement::Measurement() {
    t_ = 0;
    data_.resize(1);
    data_ << 0;
    type_ = EMPTY;
}

// Construct IMU measurement
Measurement::Measurement(const sensor_msgs::Imu::ConstPtr& msg) {
    t_ = msg->header.stamp.toSec();
    data_.resize(6);
    data_ << msg->angular_velocity.x, 
             msg->angular_velocity.y, 
             msg->angular_velocity.z,
             msg->linear_acceleration.x, 
             msg->linear_acceleration.y, 
             msg->linear_acceleration.z;
    type_ = IMU;
}

// Getters
double Measurement::getTime() { return t_; }
Eigen::VectorXd Measurement::getData() { return data_; }
MeasurementType Measurement::getType() { return type_; }


// Print measurement
ostream& operator<<(ostream& os, const Measurement& m) {
    string type_str;
    switch (m.type_) {
        case IMU :
            type_str = "IMU";
            break;
        default:
            type_str = "Unknown";
    }
    os << "Measurement type: " << type_str << endl;
    //os << "Measurement time: " << m.t_ << endl;
    //os << "Measurement data: \n" << m.data_ << endl;
}
