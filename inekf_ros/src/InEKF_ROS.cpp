#include "InEKF_ROS.h"
using namespace std;
using namespace inekf;

// Constructor
InEKF_ROS::InEKF_ROS(ros::NodeHandle n) : n_(n) {}

// Initialize ROS node and filter
void InEKF_ROS::init() {
    // Create private node handle
    ros::NodeHandle nh("~");
    // Set noise parameters
    NoiseParams params;
    double std, cov;
    if (nh.getParam("noise/gyroscope_std", std)) { 
        params.setGyroscopeNoise(std);
    }
    if (nh.getParam("noise/accelerometer_std", std)) { 
        params.setAccelerometerNoise(std);
    }
    if (nh.getParam("noise/gyroscope_bias_std", std)) { 
        params.setGyroscopeBiasNoise(std);
    }
    if (nh.getParam("noise/accelerometer_bias_std", std)) { 
        params.setAccelerometerBiasNoise(std);
    }
    if (nh.getParam("noise/landmark_std", std)) { 
        params.setLandmarkNoise(std);
    }
    filter_.setNoiseParams(params);

    // Set initial state and covariance
    RobotState state;
    Eigen::Matrix3d R_init = Eigen::Matrix3d::Identity();
    Eigen::Vector3d v_init = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_init = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg_init = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba_init = Eigen::Vector3d::Zero();
    Eigen::Matrix<double,15,15> P_init = Eigen::Matrix<double,15,15>::Zero();

    vector<double> param_vec;
    if (nh.getParam("prior/base_orientation", param_vec)) { 
        ROS_ASSERT(param_vec.size() == 4);
        Eigen::Quaternion<double> q(param_vec[0], param_vec[1], param_vec[2], param_vec[3]);
        q.normalize();
        R_init = q.toRotationMatrix();
    }
    if (nh.getParam("prior/base_velocity", param_vec)) { 
        ROS_ASSERT(param_vec.size() == 3);
        v_init << param_vec[0], param_vec[1], param_vec[2];
    }
    if (nh.getParam("prior/base_position", param_vec)) { 
        ROS_ASSERT(param_vec.size() == 3);
        p_init << param_vec[0], param_vec[1], param_vec[2];
    }
    if (nh.getParam("prior/gyroscope_bias", param_vec)) { 
        ROS_ASSERT(param_vec.size() == 3);
        bg_init << param_vec[0], param_vec[1], param_vec[2];
    }
    if (nh.getParam("prior/accelerometer_bias", param_vec)) { 
        ROS_ASSERT(param_vec.size() == 3);
        ba_init << param_vec[0], param_vec[1], param_vec[2];
    }
    if (nh.getParam("prior/base_orientation_std", std)) { 
        cov = std*std;
        P_init(0,0) = cov; P_init(1,1) = cov; P_init(2,2) = cov;
    }
    if (nh.getParam("prior/base_velocity_std", std)) { 
        cov = std*std;
        P_init(3,3) = cov; P_init(4,4) = cov; P_init(5,5) = cov;
    }
    if (nh.getParam("prior/base_position_std", std)) { 
        cov = std*std;
        P_init(6,6) = cov; P_init(7,7) = cov; P_init(8,8) = cov;
    }
    if (nh.getParam("prior/gyroscope_bias_std", std)) { 
        cov = std*std;
        P_init(9,9) = cov; P_init(10,10) = cov; P_init(11,11) = cov;
    }
    if (nh.getParam("prior/accelerometer_bias_std", std)) { 
        cov = std*std;
        P_init(12,12) = cov; P_init(13,13) = cov; P_init(14,14) = cov;
    }
    state.setRotation(R_init);
    state.setVelocity(v_init);
    state.setPosition(p_init);
    state.setAngularVelocityBias(bg_init);
    state.setLinearAccelerationBias(ba_init);
    state.setP(P_init);
    filter_.setState(state);

    // Print out initialization
    cout << "Robot's state is initialized to: \n";
    cout << filter_.getState() << endl;
    cout << filter_.getNoiseParams() << endl;

    // Set prior landmarks if given
    mapIntVector3d prior_landmarks;
    XmlRpc::XmlRpcValue list;
    if (nh.getParam("prior/landmarks", list)) { 
        for (int i = 0; i < list.size(); ++i) {
            XmlRpc::XmlRpcValue landmark = list[i];
            ROS_ASSERT_MSG(landmark["id"].getType() == XmlRpc::XmlRpcValue::TypeInt, "Prior landmark ID is not an int.");
            int id = static_cast<int>(landmark["id"]);
            XmlRpc::XmlRpcValue position = landmark["position"];
            ROS_ASSERT_MSG(position.size()==3, "Prior landmark position does not have 3 elements.");
            Eigen::Vector3d p_wl;
            for (int j=0; j<3; ++j){
                ROS_ASSERT_MSG(position[i].getType() == XmlRpc::XmlRpcValue::TypeDouble, "Prior landmark position is not a double.");
                p_wl(j) = static_cast<double>(position[j]);
            }
            ROS_INFO("Adding prior landmark (ID, position): %d, [%f, %f, %f]", id, p_wl[0], p_wl[1], p_wl[2]);
            prior_landmarks.insert(pair<int,Eigen::Vector3d> (landmark["id"], p_wl)); 
        }
    }
    filter_.setPriorLandmarks(prior_landmarks);
}

// Process Data
void InEKF_ROS::run() {
    // Subscribe to all publishers
    this->subscribe();

    // Start main processing thread
    filtering_thread_ = std::thread([this]{this->mainFilteringThread();});
    output_thread_ = std::thread([this]{this->outputPublishingThread();});
    ros::spin();
}    

// Subscribe to all publishers
void InEKF_ROS::subscribe() {
    // Subscribe to IMU publisher
    imu_sub_ = n_.subscribe("/imu", 1000, &InEKF_ROS::imuCallback, this);
    // Subscribe to Landmark publisher
    landmarks_sub_ = n_.subscribe("/landmarks", 1000, &InEKF_ROS::landmarkCallback, this);
}

// IMU Callback function
void InEKF_ROS::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    shared_ptr<Measurement> ptr(new ImuMeasurement(msg));
    m_queue_.push(ptr);
}

// Landmark Callback function
void InEKF_ROS::landmarkCallback(const inekf_msgs::LandmarkArray::ConstPtr& msg) {
    shared_ptr<Measurement> ptr(new LandmarkMeasurement(msg));
    m_queue_.push(ptr);
}

void InEKF_ROS::mainFilteringThread() {
    cout << "Inside Main Filtering Thread\n";
    shared_ptr<Measurement> m_ptr;
    shared_ptr<ImuMeasurement> imu_ptr_last;
    double t, t_last;

    // Block until first IMU measurement is received
    while (true) {
        // Try next item (Blocking)
        m_queue_.pop(m_ptr);
        if (m_ptr->getType() == IMU) {
            ROS_INFO("First IMU measurement received. Starting Filter.");
            t_last = m_ptr->getTime();
            imu_ptr_last = dynamic_pointer_cast<ImuMeasurement>(m_ptr);
            break;
        }
        this_thread::sleep_for(chrono::milliseconds(10));
    }
    
    // Main loop
    while (true) {
        // Throw warning if measurement queue is getting too large
        if (m_queue_.size() > MAX_QUEUE_SIZE) {
            ROS_WARN("Measurement queue size (%d) is greater than MAX_QUEUE_SIZE. Filter is not realtime!", m_queue_.size());
        }   
        // Retrieve next measurement (Blocking)
        m_queue_.pop(m_ptr);
        // Handle measurement
        switch (m_ptr->getType()) {
            case IMU: {
                //ROS_INFO("Propagating state with IMU measurements.");
                auto imu_ptr = dynamic_pointer_cast<ImuMeasurement>(m_ptr);
                t = imu_ptr->getTime();
                filter_.Propagate(imu_ptr_last->getData(), t - t_last);
                t_last = t;
                imu_ptr_last = imu_ptr;
                break;
            }
            case LANDMARK: {
                //ROS_INFO("Correcting state with LANDMARK measurements.");
                auto landmark_ptr = dynamic_pointer_cast<LandmarkMeasurement>(m_ptr);
                filter_.CorrectLandmarks(landmark_ptr->getData());
                break;
            }
            default:
                cout << "Unknown measurement, skipping...\n";
        }
    }
}


// Thread for publishing the output of the filter
void InEKF_ROS::outputPublishingThread() {
    // Create pose publisher
    ros::Publisher pose_pub = n_.advertise<geometry_msgs::PoseStamped>("/pose", 1000);
    ros::Publisher state_pub = n_.advertise<inekf_msgs::State>("/state", 1000);
    static tf::TransformBroadcaster tf_broadcaster;
    uint32_t seq = 0;

    ros::Rate loop_rate(100);
    // Main loop
    while(true) {
        // Create and send pose message
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.seq = seq;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "/map"; 
        RobotState state = filter_.getState();
        Eigen::Vector3d position = state.getPosition();
        Eigen::Quaternion<double> orientation(state.getRotation());
        orientation.normalize();
        pose_msg.pose.position.x = position(0); 
        pose_msg.pose.position.y = position(1); 
        pose_msg.pose.position.z = position(2); 
        pose_msg.pose.orientation.w = orientation.w(); 
        pose_msg.pose.orientation.x = orientation.x(); 
        pose_msg.pose.orientation.y = orientation.y(); 
        pose_msg.pose.orientation.z = orientation.z();
        pose_pub.publish(pose_msg);

        // Create and send tf message
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(position(0),position(1),position(2)) );
        transform.setRotation( tf::Quaternion(orientation.x(),orientation.y(),orientation.z(),orientation.w()) );
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/imu"));

        // Create and send State message
        inekf_msgs::State state_msg;
        state_msg.header.seq = seq;
        state_msg.header.stamp = ros::Time::now();
        state_msg.header.frame_id = "/map"; 
        state_msg.pose = pose_msg.pose;
        Eigen::Vector3d velocity = state.getVelocity();
        state_msg.velocity.x = velocity(0); 
        state_msg.velocity.y = velocity(1); 
        state_msg.velocity.z = velocity(2); 
        Eigen::MatrixXd X = state.getX();
        for (int i=5; i<X.cols(); ++i) {
            geometry_msgs::Point landmark;
            landmark.x = X(0,i);
            landmark.y = X(1,i);
            landmark.z = X(2,i);
            state_msg.landmarks.push_back(landmark);
        }
        Eigen::Vector3d bg = state.getAngularVelocityBias();
        state_msg.gyroscope_bias.x = bg(0); 
        state_msg.gyroscope_bias.y = bg(1); 
        state_msg.gyroscope_bias.z = bg(2); 
        Eigen::Vector3d ba = state.getLinearAccelerationBias();
        state_msg.accelerometer_bias.x = ba(0); 
        state_msg.accelerometer_bias.y = ba(1); 
        state_msg.accelerometer_bias.z = ba(2); 
        state_pub.publish(state_msg);

        seq++;
        loop_rate.sleep();
    }

}
