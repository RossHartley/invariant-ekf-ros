#include "Measurement.h"
using namespace std;
using namespace inekf;

// Construct Empty Measurement
Measurement::Measurement() {
    t_ = 0;
    type_ = EMPTY;
}
// Getters
double Measurement::getTime() { return t_; }
MeasurementType Measurement::getType() { return type_; }

// Construct IMU measurement
ImuMeasurement::ImuMeasurement(const sensor_msgs::Imu::ConstPtr& msg) {
    t_ = msg->header.stamp.toSec();
    data_ << msg->angular_velocity.x, 
             msg->angular_velocity.y, 
             msg->angular_velocity.z,
             msg->linear_acceleration.x, 
             msg->linear_acceleration.y, 
             msg->linear_acceleration.z;
    type_ = IMU;
}
Eigen::VectorXd ImuMeasurement::getData() { return data_; }

// Construct Landmark measurement
LandmarkMeasurement::LandmarkMeasurement(const inekf_msgs::LandmarkArray::ConstPtr& msg){
    t_ = msg->header.stamp.toSec();
    type_ = LANDMARK;
    for (int i=0; i<msg->landmarks.size(); ++i) {
        Eigen::Vector3d position;
        position << msg->landmarks[i].position.x, 
                    msg->landmarks[i].position.y, 
                    msg->landmarks[i].position.z,
        data_.push_back(pair<int,Eigen::Vector3d> (msg->landmarks[i].id, position)); 
    }
}
vectorPairIntVector3d LandmarkMeasurement::getData() { return data_; }; 


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
