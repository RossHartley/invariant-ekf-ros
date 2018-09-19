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


    // ----- Settings --------
    string landmark_measurement_markers_topic;
    nh.param<string>("settings/map_frame_id", map_frame_id_, "/map");

    // Create publisher if landmark_measurement_markers are to be published
    nh.param<bool>("settings/publish_landmark_measurement_markers", publish_landmark_measurement_markers_, false);
    if (publish_landmark_measurement_markers_) {
        nh.param<string>("settings/landmark_measurement_markers_topic", landmark_measurement_markers_topic, "/landmark_measurement_markers");
        landmark_measurement_vis_pub_ = n_.advertise<visualization_msgs::Marker>(landmark_measurement_markers_topic, 1000);
    }

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
    // Create private node handle to get topic names
    ros::NodeHandle nh("~");
    string imu_topic, landmarks_topic;
    nh.param<string>("settings/imu_topic", imu_topic, "/imu");
    nh.param<string>("settings/landmarks_topic", landmarks_topic, "/landmarks");

    ROS_INFO("Waiting for IMU message...");
    sensor_msgs::Imu::ConstPtr imu_msg = ros::topic::waitForMessage<sensor_msgs::Imu>(imu_topic);
    imu_frame_id_ = imu_msg->header.frame_id;
    ROS_INFO("IMU message received. IMU frame is set to %s.", imu_frame_id_.c_str());

    ROS_INFO("Waiting for Landmark message...");
    inekf_msgs::LandmarkArray::ConstPtr landmark_msg = ros::topic::waitForMessage<inekf_msgs::LandmarkArray>(landmarks_topic);
    string camera_frame_id = landmark_msg->header.frame_id;
    ROS_INFO("Landmark message received. Camera frame is set to %s.", camera_frame_id.c_str());

    ROS_INFO("Waiting for tf lookup between frames %s and %s...", camera_frame_id.c_str(), imu_frame_id_.c_str());
    tf::TransformListener listener;
    try {
        listener.waitForTransform(imu_frame_id_, camera_frame_id, ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform(imu_frame_id_, camera_frame_id, ros::Time(0), camera_to_imu_transform_);
        ROS_INFO("Tranform between frames %s and %s was found.", camera_frame_id.c_str(), imu_frame_id_.c_str());
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s. Using identity transform.",ex.what());
        camera_to_imu_transform_ = tf::StampedTransform( tf::Transform::getIdentity(), ros::Time::now(), camera_frame_id, imu_frame_id_);
    }   

    // Subscribe to IMU publisher
    ROS_INFO("Subscribing to %s.", imu_topic.c_str());
    imu_sub_ = n_.subscribe(imu_topic, 1000, &InEKF_ROS::imuCallback, this);
    // Subscribe to Landmark publisher
    ROS_INFO("Subscribing to %s.", landmarks_topic.c_str());
    landmarks_sub_ = n_.subscribe(landmarks_topic, 1000, &InEKF_ROS::landmarkCallback, this);
}

// IMU Callback function
void InEKF_ROS::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    shared_ptr<Measurement> ptr(new ImuMeasurement(msg));
    m_queue_.push(ptr);
}

// Landmark Callback function
void InEKF_ROS::landmarkCallback(const inekf_msgs::LandmarkArray::ConstPtr& msg) {
    shared_ptr<Measurement> ptr(new LandmarkMeasurement(msg, camera_to_imu_transform_));
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
                if (publish_landmark_measurement_markers_) {
                    this->publishLandmarkMeasurementMarkers(landmark_ptr);
                }
                break;
            }
            default:
                cout << "Unknown measurement, skipping...\n";
        }
    }
}

// Publish line markers between IMU and detected landmarks
void InEKF_ROS::publishLandmarkMeasurementMarkers(shared_ptr<LandmarkMeasurement> ptr){
    // Create and send marker for estimated trajectory visualization
    visualization_msgs::Marker landmark_measurement_msg;
    landmark_measurement_msg.header.frame_id = map_frame_id_;
    landmark_measurement_msg.header.stamp = ros::Time(ptr->getTime());
    landmark_measurement_msg.header.seq = 0;
    landmark_measurement_msg.ns = "trajectory";
    landmark_measurement_msg.type = visualization_msgs::Marker::LINE_LIST;
    landmark_measurement_msg.action = visualization_msgs::Marker::ADD;
    landmark_measurement_msg.id = 0;
    landmark_measurement_msg.scale.x = 0.01;
    landmark_measurement_msg.scale.y = 0.01;
    landmark_measurement_msg.scale.z = 0.01;
    landmark_measurement_msg.color.a = 1.0; // Don't forget to set the alpha!
    landmark_measurement_msg.color.r = 0.0;
    landmark_measurement_msg.color.g = 0.0;
    landmark_measurement_msg.color.b = 1.0;
    landmark_measurement_msg.lifetime = ros::Duration(100.0);

    geometry_msgs::Point base_point, landmark_point;
    RobotState state = filter_.getState();
    Eigen::MatrixXd X = state.getX();
    Eigen::Vector3d position = state.getPosition();
    base_point.x = position(0);
    base_point.y = position(1);
    base_point.z = position(2);
    vectorPairIntVector3d measured_landmarks = ptr->getData();
    map<int,int> estimated_landmarks = filter_.getEstimatedLandmarks();
    for (auto it=measured_landmarks.begin(); it!=measured_landmarks.end(); ++it) {
        auto search = estimated_landmarks.find(it->first);
        if (search == estimated_landmarks.end()) { continue; }
        landmark_point.x = X(0,search->second);
        landmark_point.y = X(1,search->second);
        landmark_point.z = X(2,search->second);
        landmark_measurement_msg.points.push_back(base_point);
        landmark_measurement_msg.points.push_back(landmark_point);
    }
    landmark_measurement_vis_pub_.publish(landmark_measurement_msg);
}

// Thread for publishing the output of the filter
void InEKF_ROS::outputPublishingThread() {
    // Create private node handle to get topic names
    ros::NodeHandle nh("~");
    double publish_rate;
    string base_frame_id, pose_topic, state_topic;

    nh.param<double>("settings/publish_rate", publish_rate, 10);
    nh.param<string>("settings/base_frame_id", base_frame_id, "/imu");
    nh.param<string>("settings/pose_topic", pose_topic, "/pose");
    nh.param<string>("settings/state_topic", state_topic, "/state");

    ROS_INFO("Map frame id set to %s.", map_frame_id_.c_str());
    ROS_INFO("Base frame id set to %s.", base_frame_id.c_str());
    ROS_INFO("Pose topic publishing under %s.", pose_topic.c_str());
    ROS_INFO("State topic publishing under %s.", state_topic.c_str());

    // TODO: Convert output from IMU frame to base frame 
    ROS_INFO("Waiting for tf lookup between frames %s and %s...", imu_frame_id_.c_str(), base_frame_id.c_str());
    tf::TransformListener listener;
    tf::StampedTransform imu_to_base_transform;
    try {
        listener.waitForTransform(base_frame_id, imu_frame_id_, ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform(base_frame_id, imu_frame_id_, ros::Time(0), imu_to_base_transform);
        ROS_INFO("Tranform between frames %s and %s was found.", imu_frame_id_.c_str(), base_frame_id.c_str());
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s. Using identity transform.",ex.what());
        imu_to_base_transform = tf::StampedTransform( tf::Transform::getIdentity(), ros::Time::now(), imu_frame_id_, base_frame_id);
    } 

    // Create publishers for pose and state messages
    ros::Publisher pose_pub = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1000);
    ros::Publisher state_pub = n_.advertise<inekf_msgs::State>(state_topic, 1000);
    static tf::TransformBroadcaster tf_broadcaster;

    // Create publishers for landmark position and trajectory markers if requested
    ros::Publisher landmark_vis_pub, traj_vis_pub;
    nh.param<bool>("settings/publish_landmark_position_markers", publish_landmark_position_markers_, false);
    if (publish_landmark_position_markers_) {
        string landmark_position_markers_topic;
        nh.param<string>("settings/landmark_position_markers_topic", landmark_position_markers_topic, "/landmark_position_markers");
        landmark_vis_pub = n_.advertise<visualization_msgs::MarkerArray>(landmark_position_markers_topic, 1000);
        ROS_INFO("Landmark visualization topic publishing under %s.", landmark_position_markers_topic.c_str());
    }
    nh.param<bool>("settings/publish_trajectory_markers", publish_trajectory_markers_, false);
    if (publish_trajectory_markers_) {
        string trajectory_markers_topic;
        nh.param<string>("settings/trajectory_markers_topic", trajectory_markers_topic, "/trajectory_markers");
        traj_vis_pub = n_.advertise<visualization_msgs::Marker>(trajectory_markers_topic, 1000);
        ROS_INFO("Trajectory visualization topic publishing under %s.", trajectory_markers_topic.c_str());
    }
    
    // Init loop params
    uint32_t seq = 0;
    geometry_msgs::Point point_last;
    ros::Rate loop_rate(publish_rate);

    // Main loop
    while(true) {
        RobotState state = filter_.getState();
        Eigen::MatrixXd X = state.getX();
        Eigen::MatrixXd P = state.getP();

        // Create and send pose message
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.seq = seq;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = map_frame_id_; 
        Eigen::Vector3d position = state.getPosition();
        Eigen::Quaternion<double> orientation(state.getRotation());
        orientation.normalize();
        // Transform from imu frame to base frame
        tf::Transform imu_pose;
        imu_pose.setRotation( tf::Quaternion(orientation.x(),orientation.y(),orientation.z(),orientation.w()) );
        imu_pose.setOrigin( tf::Vector3(position(0),position(1),position(2)) );
        tf::Transform base_pose = imu_to_base_transform*imu_pose;
        tf::Quaternion base_orientation = base_pose.getRotation().normalize();
        tf::Vector3 base_position = base_pose.getOrigin();  
        // Construct message
        pose_msg.pose.pose.position.x = base_position.getX(); 
        pose_msg.pose.pose.position.y = base_position.getY(); 
        pose_msg.pose.pose.position.z = base_position.getZ(); 
        pose_msg.pose.pose.orientation.w = base_orientation.getW();
        pose_msg.pose.pose.orientation.x = base_orientation.getX();
        pose_msg.pose.pose.orientation.y = base_orientation.getY();
        pose_msg.pose.pose.orientation.z = base_orientation.getZ();
        Eigen::Matrix<double,6,6> P_pose; // TODO: convert covariance from imu to body frame (adjoint?)
        P_pose.block<3,3>(0,0) = P.block<3,3>(0,0);
        P_pose.block<3,3>(0,3) = P.block<3,3>(0,6);
        P_pose.block<3,3>(3,0) = P.block<3,3>(6,0);
        P_pose.block<3,3>(3,3) = P.block<3,3>(6,6);
        for (int i=0; i<36; ++i) {
            pose_msg.pose.covariance[i] = P_pose(i);
        }
        pose_pub.publish(pose_msg);

        // Create and send tf message
        tf_broadcaster.sendTransform(tf::StampedTransform(base_pose, ros::Time::now(), map_frame_id_, base_frame_id));

        // Create and send State message
        inekf_msgs::State state_msg;
        state_msg.header.seq = seq;
        state_msg.header.stamp = ros::Time::now();
        state_msg.header.frame_id = map_frame_id_; 
        state_msg.pose = pose_msg.pose.pose;
        Eigen::Vector3d velocity = state.getVelocity();
        state_msg.velocity.x = velocity(0); 
        state_msg.velocity.y = velocity(1); 
        state_msg.velocity.z = velocity(2); 
        map<int,int> estimated_landmarks = filter_.getEstimatedLandmarks();
        for (auto it=estimated_landmarks.begin(); it!=estimated_landmarks.end(); ++it) {
            inekf_msgs::Landmark landmark;
            landmark.id = it->first;
            landmark.position.x = X(0,it->second);
            landmark.position.y = X(1,it->second);
            landmark.position.z = X(2,it->second);
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

        // Create and send markers for landmark visualization
        if (publish_landmark_position_markers_) {
            visualization_msgs::MarkerArray landmark_vis_msg;
            for (auto it=estimated_landmarks.begin(); it!=estimated_landmarks.end(); ++it) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = map_frame_id_;
                marker.header.stamp = ros::Time::now();
                marker.header.seq = seq;
                marker.ns = "landmarks";
                marker.id = it->first;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = X(0,it->second);
                marker.pose.position.y = X(1,it->second);
                marker.pose.position.z = X(2,it->second);
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = sqrt(P(3+3*(it->second-3),3+3*(it->second-3)));
                marker.scale.y = sqrt(P(4+3*(it->second-3),4+3*(it->second-3)));
                marker.scale.z = sqrt(P(5+3*(it->second-3),5+3*(it->second-3)));
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                landmark_vis_msg.markers.push_back(marker);
            }
            landmark_vis_pub.publish(landmark_vis_msg);
        }

        // Create and send marker for estimated trajectory visualization
        if (publish_trajectory_markers_) {
            visualization_msgs::Marker traj_vis_msg;
            traj_vis_msg.header.frame_id = map_frame_id_;
            traj_vis_msg.header.stamp = ros::Time();
            traj_vis_msg.header.seq = seq;
            traj_vis_msg.ns = "trajectory";
            traj_vis_msg.type = visualization_msgs::Marker::LINE_STRIP;
            traj_vis_msg.action = visualization_msgs::Marker::ADD;
            traj_vis_msg.id = seq;
            traj_vis_msg.scale.x = 0.01;
            traj_vis_msg.color.a = 1.0; // Don't forget to set the alpha!
            traj_vis_msg.color.r = 1.0;
            traj_vis_msg.color.g = 0.0;
            traj_vis_msg.color.b = 0.0;
            traj_vis_msg.lifetime = ros::Duration(100.0);
            geometry_msgs::Point point;
            point.x = position(0);
            point.y = position(1);
            point.z = position(2);
            if (seq>0){
                traj_vis_msg.points.push_back(point_last);
                traj_vis_msg.points.push_back(point);
                traj_vis_pub.publish(traj_vis_msg);
            }   
            point_last = point;
        }

        seq++;
        loop_rate.sleep();
    }

}
