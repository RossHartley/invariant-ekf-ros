#ifndef MEASUREMENT_H
#define MEASUREMENT_H 
#include <Eigen/Dense>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "inekf_msgs/ContactArray.h"
#include "inekf_msgs/KinematicsArray.h"
#include "inekf_msgs/LandmarkArray.h"
#include "InEKF.h"
#include "tf/transform_listener.h"

enum MeasurementType {EMPTY, IMU, LANDMARK, KINEMATIC, CONTACT};

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
        LandmarkMeasurement(const inekf_msgs::LandmarkArray::ConstPtr& msg, const tf::StampedTransform& transform);
        inekf::vectorPairIntVector3d getData();
        //friend std::ostream& operator<<(std::ostream& os, const Measurement& m);  

    private:
        inekf::vectorPairIntVector3d data_;
};

class ContactMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ContactMeasurement(const inekf_msgs::ContactArray::ConstPtr& msg);
        std::vector<std::pair<int,bool>> getData();
        //friend std::ostream& operator<<(std::ostream& os, const Measurement& m);  

    private:
        std::vector<std::pair<int,bool>> data_;
};


class KinematicMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        KinematicMeasurement(const inekf_msgs::KinematicsArray::ConstPtr& msg);
        inekf::vectorTupleIntMatrix4dMatrix6d getData();
        //friend std::ostream& operator<<(std::ostream& os, const Measurement& m);  

    private:
        inekf::vectorTupleIntMatrix4dMatrix6d data_;
};


#endif 
