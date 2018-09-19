#ifndef MEASUREMENT_H
#define MEASUREMENT_H 
#include <Eigen/Dense>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "inekf_msgs/LandmarkStamped.h"
#include "inekf_msgs/LandmarkArray.h"
#include "InEKF.h"

enum MeasurementType {EMPTY, IMU, LANDMARK};

class Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Measurement();
        virtual ~Measurement() = default;

        double getTime();
        MeasurementType getType();

        friend std::ostream& operator<<(std::ostream& os, const Measurement& m);  

    protected:
        double t_;
        MeasurementType type_;
};

class ImuMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ImuMeasurement(const sensor_msgs::Imu::ConstPtr& msg);
        Eigen::VectorXd getData();

    private: 
        Eigen::Matrix<double,6,1> data_;

        //friend std::ostream& operator<<(std::ostream& os, const ImuMeasurement& m);  
};


class LandmarkMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        LandmarkMeasurement(const inekf_msgs::LandmarkArray::ConstPtr& msg);
        inekf::vectorPairIntVector3d getData();
        //friend std::ostream& operator<<(std::ostream& os, const Measurement& m);  

    private:
        inekf::vectorPairIntVector3d data_;
};

#endif 
