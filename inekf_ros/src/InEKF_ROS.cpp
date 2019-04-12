/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF_ROS.cpp
 *  @author Ross Hartley
 *  @brief  Source file for a ROS wrapper of the Invariant EKF 
 *  @date   September 27, 2018
 **/

#include "InEKF_ROS.h"
using namespace std;
using namespace inekf;

// Constructor
InEKF_ROS::InEKF_ROS(ros::NodeHandle n) : n_(n), enabled_(false) {}


// Checks if filter is enabled
bool InEKF_ROS::enabled() { return enabled_; }


// Checks if the filter's bias is initialized 
bool InEKF_ROS::biasInitialized() { return bias_initialized_; }


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
    if (nh.getParam("noise/contact_std", std)) { 
        params.setContactNoise(std);
    }
    filter_.setNoiseParams(params);

    // Print out initialization
    cout << "Noise parameters are set to: \n";
    cout << filter_.getNoiseParams() << endl;

    // Set inital IMU bias (static_bias_initialization flag with overwrite)
    vector<double> param_vec;
    if (nh.getParam("prior/gyroscope_bias", param_vec)) { 
        ROS_ASSERT(param_vec.size() == 3);
        bg0_ << param_vec[0], param_vec[1], param_vec[2];
    }
    if (nh.getParam("prior/accelerometer_bias", param_vec)) { 
        ROS_ASSERT(param_vec.size() == 3);
        ba0_ << param_vec[0], param_vec[1], param_vec[2];
    }

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



    // -----  Settings --------
    nh.param<string>("settings/map_frame_id", map_frame_id_, "/map");
    nh.param<bool>("settings/enable_landmarks", enable_landmarks_, false);
    nh.param<bool>("settings/enable_kinematics", enable_kinematics_, false);
    nh.param<bool>("settings/initialize_state_from_first_observation", initialize_state_from_first_observation_, false);
    nh.param<bool>("settings/static_bias_initialization", static_bias_initialization_, false);
    nh.param<bool>("settings/flat_ground", flat_ground_, false);

    // ----- Observation Noise Models ---- // 
    nh.param<double>("noise/landmark_std", std, 0.1);
    observation_covariance_landmark_ = std*std*Eigen::Matrix3d::Identity();
    

    // Create publishers visualization markers if requested
    nh.param<bool>("settings/publish_visualization_markers", publish_visualization_markers_, false);
    if (publish_visualization_markers_) {
        string visualization_markers_topic;
        nh.param<string>("settings/visualization_markers_topic", visualization_markers_topic, "/markers");
        visualization_pub_ = n_.advertise<visualization_msgs::MarkerArray>(visualization_markers_topic, 1000);
        ROS_INFO("Visualization topic publishing under %s.", visualization_markers_topic.c_str());
    }
}


// Process Data
void InEKF_ROS::run() {
    // Subscribe to all publishers
    this->subscribe();
    // Start main processing thread
    filtering_thread_ = std::thread([this]{this->mainFilteringThread();});
    ros::spin();
}    


// Initialize state
void InEKF_ROS::initState() {
    ros::NodeHandle nh("~");
    RobotState state;
    Eigen::Matrix3d R_init = Eigen::Matrix3d::Identity();
    Eigen::Vector3d v_init = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_init = Eigen::Vector3d::Zero();
    shared_ptr<Measurement> m_ptr;


    if (initialize_state_from_first_observation_) {
        // TODO (landmark initialization)
        ROS_INFO("Waiting for first observation to initialize state.");

        // --- Initialize orientation from IMU -----
        // Block until first IMU measurement is received
        while (ros::ok()) {
            // Try next item (Blocking)
            m_queue_.pop(m_ptr);
            if (m_ptr->getType() == IMU) {
                imu_prev_ = dynamic_pointer_cast<ImuMeasurement>(m_ptr);
                R_init = imu_prev_->getRotation(); 
                break;
            }
            this_thread::sleep_for(chrono::milliseconds(1));
        }

        // --- Initialize position from kinematics or landmark measurement -----
        // Block until first contact or landmark measurement is received
        bool p_initialized = false;
        while (!p_initialized) {
            // Try next item (Blocking)
            m_queue_.pop(m_ptr);
            switch (m_ptr->getType()) {
                case CONTACT: {
                    auto contacts = dynamic_pointer_cast<ContactMeasurement>(m_ptr);
                    filter_.setContacts(contacts->getData());
                    break;
                }
                case KINEMATIC: {
                    auto kinematics = dynamic_pointer_cast<KinematicMeasurement>(m_ptr);
                    inekf::vectorKinematics kinematics_vec = kinematics->getData();
                    map<int,bool> contacts = filter_.getContacts();
                    for (int i=0; i<kinematics_vec.size(); ++i) {
                        auto search = contacts.find(kinematics_vec[i].id);
                        if (search != contacts.end()) {
                            // If contact is indicated, use kinematics to initialize position
                            if (search->second == true) {
                                ROS_INFO("Initializing position using kinematics. Position of contact id %i is set to the origin.", search->first);
                                p_init = -R_init*kinematics_vec[i].pose.block<3,1>(0,3); // Assume foot position is (0,0,0)
                                p_initialized = true;
                                break;
                            }
                        } 
                    }
                    break;
                }
                case LANDMARK: {
                    auto landmarks = dynamic_pointer_cast<LandmarkMeasurement>(m_ptr);
                    inekf::vectorLandmarks landmarks_vec = landmarks->getData();
                    ROS_INFO("Initializing position using landmark measurement. Position of landmark id %i is set to the origin.", landmarks_vec[0].id); 
                    p_init = -R_init*landmarks_vec[0].position; // Assume landmark position is (0,0,0)
                    p_initialized = true;
                }
            }
            this_thread::sleep_for(chrono::microseconds(1));
        }



    } else {
        // Use parameter server to initialize (if set)
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
    }


    // Set Initial Covariance
    double std;
    if (nh.getParam("prior/base_orientation_std", std)) { 
        state.setRotationCovariance(std*std*Eigen::Matrix3d::Identity());
    }
    if (nh.getParam("prior/base_velocity_std", std)) { 
        state.setVelocityCovariance(std*std*Eigen::Matrix3d::Identity());
    }
    if (nh.getParam("prior/base_position_std", std)) { 
        state.setPositionCovariance(std*std*Eigen::Matrix3d::Identity());
    }
    if (nh.getParam("prior/gyroscope_bias_std", std)) { 
        state.setGyroscopeBiasCovariance(std*std*Eigen::Matrix3d::Identity());
    }
    if (nh.getParam("prior/accelerometer_bias_std", std)) { 
        state.setAccelerometerBiasCovariance(std*std*Eigen::Matrix3d::Identity());
    }
  
    // Set initial state information
    state.setRotation(R_init);
    state.setVelocity(v_init);
    state.setPosition(p_init);
    state.setGyroscopeBias(bg0_);
    state.setAccelerometerBias(ba0_);
    
    // Move covariance to left invariant if needed
    if (filter_.getErrorType() == inekf::ErrorType::LeftInvariant) {
        Eigen::MatrixXd P = state.getP();
        Eigen::MatrixXd AdjInv =  Eigen::MatrixXd::Identity(state.dimP(),state.dimP());
        AdjInv.block(0,0,state.dimP()-state.dimTheta(),state.dimP()-state.dimTheta()) = inekf::Adjoint_SEK3(state.Xinv()); 
        state.setP(AdjInv*P*AdjInv.transpose());
    } 
    filter_.setState(state);

    // Print out initialization
    cout << "Robot's state is initialized to: \n";
    cout << filter_.getState() << endl;

    // Block until first IMU measurement is received
    while (ros::ok()) {
        // Try next item (Blocking)
        m_queue_.pop(m_ptr);
        if (m_ptr->getType() == IMU) {
            ROS_INFO("First IMU measurement received. Enabling Filter.");
            t_prev_ = m_ptr->getTime();
            t_ = t_prev_; // Set initial value to prevent crash (TODO: better way to initialize this)
            imu_prev_ = dynamic_pointer_cast<ImuMeasurement>(m_ptr);
            break;
        }
        this_thread::sleep_for(chrono::milliseconds(1));
    }

    // Enable filter
    enabled_ = true;
}


// Initialize InEKF bias estimates 
void InEKF_ROS::initBias() {
    // Return immediately if static bias initialization is disabled
    if (!static_bias_initialization_) {
        bias_initialized_ = true;
        return;
    }

    // Initialize bias based on orientation and static assumption
    shared_ptr<Measurement> m_ptr;
    Eigen::Vector3d g; g << 0,0,-9.81;
    std::vector<Eigen::Matrix<double,6,1>,Eigen::aligned_allocator<Eigen::Matrix<double,6,1>>> bias_init_vec;

    // Initialize Bias
    while (!bias_initialized_) {
        m_queue_.pop(m_ptr); // Try next item (Blocking)
        if (m_ptr->getType() == IMU) {
            auto imu_ptr = dynamic_pointer_cast<ImuMeasurement>(m_ptr);
        
            // Store imu data with gravity removed
            Eigen::Matrix3d R = imu_ptr->getRotation();
            Eigen::Vector3d w, a;
            Eigen::Matrix<double,6,1> v = imu_ptr->getData();
            w << v.head<3>();
            a << v.tail<3>();
            Eigen::Vector3d ag = R.transpose()*(R*a + g);
            v << w, ag;
            bias_init_vec.push_back(v); 

            // Compute average bias of stored data
            if (bias_init_vec.size() > 2000) {
                Eigen::Matrix<double,6,1> avg = Eigen::Matrix<double,6,1>::Zero();
                for (int i=0; i<bias_init_vec.size(); ++i) {
                    avg = (avg + bias_init_vec[i]).eval();
                }
                avg = (avg/bias_init_vec.size()).eval();
                std::cout << "IMU bias initialized to: " << avg.transpose() << std::endl;
                bg0_ = avg.head<3>();
                ba0_ = avg.tail<3>();
                bias_initialized_ = true;
            }
            // Sleep for a bit to allow data to enter the queue
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    }   
}


// Subscribe to all publishers
void InEKF_ROS::subscribe() {
    // Create private node handle
    ros::NodeHandle nh("~");

    // ---- Setup Subscribers ------
    string imu_topic;
    nh.param<string>("settings/imu_topic", imu_topic, "/imu");

    // Retrieve imu frame_id 
    ROS_INFO("Waiting for IMU message...");
    sensor_msgs::Imu::ConstPtr imu_msg = ros::topic::waitForMessage<sensor_msgs::Imu>(imu_topic);
    imu_frame_id_ = imu_msg->header.frame_id;
    ROS_INFO("IMU message received. IMU frame is set to %s.", imu_frame_id_.c_str());

    // Retrieve camera frame_id and transformation between camera and imu
    string landmarks_topic;
    if (enable_landmarks_) {
        nh.param<string>("settings/landmarks_topic", landmarks_topic, "/landmarks");
        ROS_INFO("Waiting for Landmark message...");
        inekf_msgs::LandmarkArray::ConstPtr landmark_msg = ros::topic::waitForMessage<inekf_msgs::LandmarkArray>(landmarks_topic);
        string camera_frame_id = landmark_msg->header.frame_id;
        ROS_INFO("Landmark message received. Camera frame is set to %s.", camera_frame_id.c_str());

        ROS_INFO("Waiting for tf lookup between frames %s and %s...", imu_frame_id_.c_str(), camera_frame_id.c_str());
        tf::TransformListener listener;
        try {
            listener.waitForTransform(imu_frame_id_, camera_frame_id, ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(imu_frame_id_, camera_frame_id, ros::Time(0), imu_to_camera_transform_);
            ROS_INFO("Tranform between frames %s and %s was found.", imu_frame_id_.c_str(), camera_frame_id.c_str());
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s. Using identity transform.",ex.what());
            imu_to_camera_transform_ = tf::StampedTransform( tf::Transform::getIdentity(), ros::Time::now(), imu_frame_id_, camera_frame_id);
        }   
    }

    // Subscribe to IMU publisher
    ROS_INFO("Subscribing to %s.", imu_topic.c_str());
    imu_sub_ = n_.subscribe(imu_topic, 1000, &InEKF_ROS::imuCallback, this);

    // Subscribe to Landmark publisher
    if (enable_landmarks_) {
        ROS_INFO("Subscribing to %s.", landmarks_topic.c_str());
        landmarks_sub_ = n_.subscribe(landmarks_topic, 1000, &InEKF_ROS::landmarkCallback, this);
        // landmarks_sub_ = n_.subscribe(landmarks_topic, 1000, &InEKF_ROS::aprilTagCallback, this);
    }

    // Subscribe to Kinematics and Contact publishers
    if (enable_kinematics_) {
        string kinematics_topic, contact_topic;
        nh.param<string>("settings/kinematics_topic", kinematics_topic, "/kinematics");
        nh.param<string>("settings/contact_topic", contact_topic, "/contact");
        ROS_INFO("Subscribing to %s.", kinematics_topic.c_str());
        ROS_INFO("Subscribing to %s.", contact_topic.c_str());
        kinematics_sub_ = n_.subscribe(kinematics_topic, 1000, &InEKF_ROS::kinematicsCallback, this);
        contact_sub_ = n_.subscribe(contact_topic, 1000, &InEKF_ROS::contactCallback, this);
    }

    // ---- Setup Publishers ------
    string pose_topic, state_topic;
    nh.param<string>("settings/base_frame_id", base_frame_id_, "/imu");
    nh.param<string>("settings/pose_topic", pose_topic, "/pose");
    nh.param<string>("settings/state_topic", state_topic, "/state");

    ROS_INFO("Map frame id set to %s.", map_frame_id_.c_str());
    ROS_INFO("Base frame id set to %s.", base_frame_id_.c_str());
    ROS_INFO("Pose topic publishing under %s.", pose_topic.c_str());
    ROS_INFO("State topic publishing under %s.", state_topic.c_str());

    // TODO: Convert output from IMU frame to base frame 
    ROS_INFO("Waiting for tf lookup between frames %s and %s...", imu_frame_id_.c_str(), base_frame_id_.c_str());
    tf::TransformListener listener;
    try {
        listener.waitForTransform(imu_frame_id_, base_frame_id_, ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform(imu_frame_id_, base_frame_id_, ros::Time(0), imu_to_base_transform_);
        ROS_INFO("Tranform between frames %s and %s was found.", imu_frame_id_.c_str(), base_frame_id_.c_str());
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s. Using identity transform.",ex.what());
        imu_to_base_transform_ = tf::StampedTransform( tf::Transform::getIdentity(), ros::Time::now(), imu_frame_id_, base_frame_id_);
    } 

    // Create publishers for pose and state messages
    pose_pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1000);
    state_pub_ = n_.advertise<inekf_msgs::State>(state_topic, 1000);

}


// IMU Callback function
void InEKF_ROS::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    shared_ptr<Measurement> ptr(new ImuMeasurement(msg));
    m_queue_.push(ptr);
}


// Landmark Callback function
void InEKF_ROS::landmarkCallback(const inekf_msgs::LandmarkArray::ConstPtr& msg) {
    shared_ptr<Measurement> ptr(new LandmarkMeasurement(msg, imu_to_camera_transform_, observation_covariance_landmark_));
    m_queue_.push(ptr);
}  


// Kinematics Callback function
void InEKF_ROS::kinematicsCallback(const inekf_msgs::KinematicsArray::ConstPtr& msg) {
    shared_ptr<Measurement> ptr(new KinematicMeasurement(msg));
    m_queue_.push(ptr);
}


// Contact Callback function
void InEKF_ROS::contactCallback(const inekf_msgs::ContactArray::ConstPtr& msg) {
    shared_ptr<Measurement> ptr(new ContactMeasurement(msg));
    m_queue_.push(ptr);
}


// Main filtering logic
void InEKF_ROS::mainFilteringThread() {
    while(ros::ok()) {
        // TODO: rosservice calls for filter and bias reset

        // If filter is fully enabled, update the state estimate and publish 
        if (enabled_) {
            this->update();
            this->publish();
        } else {
            if (bias_initialized_) {
                // Initialize estimator if switched on and a contact or landmark is detected
                this->initState(); // enables the estimator
            } else {
                // Initialize InEKF IMU bias estimate
                this->initBias();
            }
        }
    }
}

// Update the state estimate
void InEKF_ROS::update() {
    shared_ptr<Measurement> m_ptr;

    // Throw warning if measurement queue is getting too large
    if (m_queue_.size() > MAX_QUEUE_SIZE) {
        ROS_WARN("Measurement queue size (%d) is greater than MAX_QUEUE_SIZE. Filter is not running realtime!", m_queue_.size());
    }   
    // Wait until buffer is full
    while(m_queue_.size() < QUEUE_BUFFER_SIZE) {
        this_thread::sleep_for(chrono::microseconds(1));
    }    
    // Retrieve next measurement (Blocking)   
    m_queue_.pop(m_ptr);
    ROS_DEBUG("Time: %f", m_ptr->getTime());  
    // Handle measurement
    switch (m_ptr->getType()) {  
        case IMU: {
            ROS_DEBUG("Propagating state with IMU measurements.");
            auto imu = dynamic_pointer_cast<ImuMeasurement>(m_ptr);
            t_ = imu->getTime();
            dt_ = t_ - t_prev_;
            if (dt_ > 0.1) {
                ROS_ERROR("TIMESTEP WAY TOO LARGE, IGNORE: %f", dt_);
            } else {  
                filter_.Propagate(imu->getData(), dt_);
            }     
            t_prev_ = t_; 
            imu_prev_ = imu; 
            break;  
        }
        case LANDMARK: {  
            // Correct state based on landmark measurements
            ROS_DEBUG("Correcting state with LANDMARK measurements.");
            auto landmarks = dynamic_pointer_cast<LandmarkMeasurement>(m_ptr); 
            filter_.CorrectLandmarks(landmarks->getData());  
            // Remove old landmarks
            const int num_landmarks_to_keep = 10;
            int largest_id = -1;
            inekf::vectorLandmarks vl = landmarks->getData();
            for (int i=0; i<vl.size(); ++i) {
                if (vl[i].id > largest_id) {
                    largest_id = vl[i].id;
                } 
            }
            std::vector<int> landmarks_to_keep;
            for (int id=largest_id; id>largest_id-num_landmarks_to_keep; --id) {
                landmarks_to_keep.push_back(id);
            }
            filter_.KeepLandmarks(landmarks_to_keep);  
            // Publish markers
            if (publish_visualization_markers_) { 
                this->publishLandmarkMeasurementMarkers(landmarks); 
            }  
            break;
        }
        case KINEMATIC: {
            ROS_DEBUG("Correcting state with KINEMATIC measurements.");
            auto kinematics = dynamic_pointer_cast<KinematicMeasurement>(m_ptr);
            auto kin = kinematics->getData();
            filter_.CorrectKinematics(kinematics->getData());
            if (publish_visualization_markers_) {
                this->publishKinematicMeasurementMarkers(kinematics);
            }    
            break;  
        }     
        case CONTACT: {            
            ROS_DEBUG("Setting filter's contact state with CONTACT measurements.");
            auto contacts = dynamic_pointer_cast<ContactMeasurement>(m_ptr);
            filter_.setContacts(contacts->getData());
            // --- Abosulte positon contact measurement (z) --- //
            if (flat_ground_) {
                Eigen::Vector3d measurement; measurement << 0,0,0; // Measure 0 ground height
                Eigen::Matrix3d covariance = 0.01*Eigen::Matrix3d::Identity();
                Eigen::Vector3d indices; indices << 0,0,1; // Specify that we are measuring only the z component
                vector<pair<int,bool>> contacts_vec = contacts->getData();
                for (int i=0; i<contacts_vec.size(); ++i){
                    if (contacts_vec[i].second == true) {
                        filter_.CorrectContactPosition(contacts_vec[i].first, measurement, covariance, indices);
                    }
                }
            }
            break;  
        }
        default:     
            ROS_ERROR("Unknown measurement, skipping...");
    }
    // std::cout << filter_.getState() << std::endl;
}


// Publish line markers between IMU and detected landmarks
void InEKF_ROS::publishLandmarkMeasurementMarkers(shared_ptr<LandmarkMeasurement> ptr){
    visualization_msgs::MarkerArray markers_msg;
    visualization_msgs::Marker landmark_measurement_msg;
    landmark_measurement_msg.header.frame_id = map_frame_id_;
    landmark_measurement_msg.header.stamp = ros::Time(ptr->getTime());
    landmark_measurement_msg.header.seq = 0;
    landmark_measurement_msg.ns = "landmark_measurements";
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

    geometry_msgs::Point base_point, landmark_point;
    RobotState state = filter_.getState();
    Eigen::MatrixXd X = state.getWorldX();
    Eigen::Vector3d position = X.block<3,1>(0,4);
    base_point.x = position(0);
    base_point.y = position(1);
    base_point.z = position(2);

    vectorLandmarks measured_landmarks = ptr->getData();
    mapIntVector3d prior_landmarks = filter_.getPriorLandmarks();
    map<int,int> estimated_landmarks = filter_.getEstimatedLandmarks();
    for (auto it=measured_landmarks.begin(); it!=measured_landmarks.end(); ++it) {
        // Search through prior landmarks
        auto search_prior = prior_landmarks.find(it->id);
        if (search_prior != prior_landmarks.end()) {
            landmark_point.x = search_prior->second(0);
            landmark_point.y = search_prior->second(1);
            landmark_point.z = search_prior->second(2);
            landmark_measurement_msg.points.push_back(base_point);
            landmark_measurement_msg.points.push_back(landmark_point);
            continue;
        }
        // Search through estimated landmarks
        auto search_estimated = estimated_landmarks.find(it->id);
        if (search_estimated != estimated_landmarks.end()) {
            landmark_point.x = X(0,search_estimated->second);
            landmark_point.y = X(1,search_estimated->second);
            landmark_point.z = X(2,search_estimated->second);
            landmark_measurement_msg.points.push_back(base_point);
            landmark_measurement_msg.points.push_back(landmark_point);  
            continue;
        }
    }
    // Publish
    markers_msg.markers.push_back(landmark_measurement_msg);
    visualization_pub_.publish(markers_msg);
}


// Publish line markers between IMU and detected contact positions
void InEKF_ROS::publishKinematicMeasurementMarkers(shared_ptr<KinematicMeasurement> ptr){
    visualization_msgs::MarkerArray markers_msg;
    visualization_msgs::Marker kinematic_measurement_msg;
    kinematic_measurement_msg.header.frame_id = map_frame_id_;
    kinematic_measurement_msg.header.stamp = ros::Time(ptr->getTime());
    kinematic_measurement_msg.header.seq = 0;
    kinematic_measurement_msg.ns = "kinematic_measurements";
    kinematic_measurement_msg.type = visualization_msgs::Marker::LINE_LIST;
    kinematic_measurement_msg.action = visualization_msgs::Marker::ADD;
    kinematic_measurement_msg.id = 0;
    kinematic_measurement_msg.scale.x = 0.01;
    kinematic_measurement_msg.scale.y = 0.01;
    kinematic_measurement_msg.scale.z = 0.01;
    kinematic_measurement_msg.color.a = 1.0; // Don't forget to set the alpha!
    kinematic_measurement_msg.color.r = 1.0;
    kinematic_measurement_msg.color.g = 0.0; 
    kinematic_measurement_msg.color.b = 1.0;

    geometry_msgs::Point base_point, contact_point;
    RobotState state = filter_.getState();
    Eigen::MatrixXd X = state.getWorldX();
    Eigen::Vector3d position = X.block<3,1>(0,4);
    base_point.x = position(0);
    base_point.y = position(1);
    base_point.z = position(2);

    vectorKinematics measured_kinematics = ptr->getData();
    map<int,int> estimated_contacts = filter_.getEstimatedContactPositions();
    for (auto it=measured_kinematics.begin(); it!=measured_kinematics.end(); ++it) {
        // Search through estimated contacts
        auto search_estimated = estimated_contacts.find(it->id);
        if (search_estimated != estimated_contacts.end()) {
            contact_point.x = X(0,search_estimated->second);
            contact_point.y = X(1,search_estimated->second);
            contact_point.z = X(2,search_estimated->second);
            kinematic_measurement_msg.points.push_back(base_point);
            kinematic_measurement_msg.points.push_back(contact_point);
            continue;
        }
    }
    // Publish
    markers_msg.markers.push_back(kinematic_measurement_msg);
    visualization_pub_.publish(markers_msg);
}


// Publishes the output of the fitler over ROS messages
void InEKF_ROS::publish() {
    ROS_DEBUG("Publishing data at time: %f", t_);
    ros::Time timestamp = ros::Time(t_);

    // Extract current state estimate
    RobotState state = filter_.getState();
    Eigen::MatrixXd X = state.getWorldX();
    Eigen::MatrixXd P = state.getP();
    Eigen::Quaternion<double> orientation(X.block<3,3>(0,0));
    orientation.normalize();
    Eigen::Vector3d velocity = X.block<3,1>(0,3);
    Eigen::Vector3d position = X.block<3,1>(0,4);
    Eigen::Vector3d bg = state.getGyroscopeBias();
    Eigen::Vector3d ba = state.getAccelerometerBias();

    // Create and send pose message
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.seq = seq_;
    pose_msg.header.stamp = timestamp;
    pose_msg.header.frame_id = map_frame_id_; 

    // Transform from imu frame to base frame
    tf::Transform imu_pose;
    imu_pose.setRotation( tf::Quaternion(orientation.x(),orientation.y(),orientation.z(),orientation.w()) );
    imu_pose.setOrigin( tf::Vector3(position(0),position(1),position(2)) );
    tf::Transform base_pose = imu_pose*imu_to_base_transform_;
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
    // TODO: Need to compute and fill in covariance of base
    pose_pub_.publish(pose_msg);

    // Create and send tf message
    tf_broadcaster_.sendTransform(tf::StampedTransform(base_pose, timestamp, map_frame_id_, base_frame_id_));

    // Create and send State message
    inekf_msgs::State state_msg;
    state_msg.header.seq = seq_;
    state_msg.header.stamp = timestamp;
    state_msg.header.frame_id = map_frame_id_; 
    state_msg.orientation.w = orientation.w();
    state_msg.orientation.x = orientation.x();
    state_msg.orientation.y = orientation.y();  
    state_msg.orientation.z = orientation.z();
    state_msg.position.x = position(0); 
    state_msg.position.y = position(1); 
    state_msg.position.z = position(2); 
    state_msg.velocity.x = velocity(0);   
    state_msg.velocity.y = velocity(1); 
    state_msg.velocity.z = velocity(2); 
    for (int i=0; i<9; ++i) {
        for (int j=0; j<9; ++j) {
            state_msg.covariance[9*i+j] = P(i,j);
        }
    }
    map<int,int> estimated_landmarks = filter_.getEstimatedLandmarks();
    for (auto it=estimated_landmarks.begin(); it!=estimated_landmarks.end(); ++it) {
        inekf_msgs::VectorWithId landmark;
        landmark.id = it->first;
        landmark.position.x = X(0,it->second);
        landmark.position.y = X(1,it->second);
        landmark.position.z = X(2,it->second);
        state_msg.landmarks.push_back(landmark);
    }
    map<int,int> estimated_contacts = filter_.getEstimatedContactPositions();
    for (auto it=estimated_contacts.begin(); it!=estimated_contacts.end(); ++it) {
        inekf_msgs::VectorWithId contact;
        contact.id = it->first;
        contact.position.x = X(0,it->second);
        contact.position.y = X(1,it->second);
        contact.position.z = X(2,it->second);
        state_msg.contacts.push_back(contact);
    }
    state_msg.gyroscope_bias.x = bg(0); 
    state_msg.gyroscope_bias.y = bg(1); 
    state_msg.gyroscope_bias.z = bg(2); 
    state_msg.accelerometer_bias.x = ba(0); 
    state_msg.accelerometer_bias.y = ba(1); 
    state_msg.accelerometer_bias.z = ba(2); 
    state_pub_.publish(state_msg);
  
    // Create and send markers for visualization
    if (publish_visualization_markers_) {
        visualization_msgs::MarkerArray markers_msg;

        // Publish pose covariance samples
        visualization_msgs::Marker marker_pose;
        marker_pose.header.frame_id = map_frame_id_;
        marker_pose.header.stamp = timestamp;
        marker_pose.header.seq = seq_;
        marker_pose.ns = "imu_pose";
        marker_pose.id = 0;
        marker_pose.type = visualization_msgs::Marker::SPHERE_LIST;
        marker_pose.action = visualization_msgs::Marker::ADD;
        marker_pose.scale.x = 0.01;
        marker_pose.scale.y = 0.01;
        marker_pose.scale.z = 0.01;
        marker_pose.color.a = 1.0; // Don't forget to set the alpha!
        marker_pose.color.r = 0.0;
        marker_pose.color.g = 1.0;
        marker_pose.color.b = 0.0;
        marker_pose.lifetime = ros::Duration(dt_);
        // Sample N points in the lie algebra and project to the manifold
        inekf::ErrorType error_type = filter_.getErrorType();
        Eigen::Matrix<double,4,4> X_pose = Eigen::Matrix<double,4,4>::Identity();
        X_pose.block<3,3>(0,0) = X.block<3,3>(0,0);
        X_pose.block<3,1>(0,3) = X.block<3,1>(0,4);
        Eigen::Matrix<double,6,6> P_pose;
        P_pose.block<3,3>(0,0) = P.block<3,3>(0,0);
        P_pose.block<3,3>(0,3) = P.block<3,3>(0,6);
        P_pose.block<3,3>(3,0) = P.block<3,3>(6,0);
        P_pose.block<3,3>(3,3) = P.block<3,3>(6,6);

        Eigen::MatrixXd L( P_pose.llt().matrixL() );
        std::default_random_engine generator;
        std::normal_distribution<double> distribution(0,1);
        const int N = 1000;
        for (int i=0; i<N; ++i) {
            Eigen::VectorXd xi;
            xi.resize(6);
            for (int j=0; j<6; ++j) {
                xi(j) = distribution(generator);
            }
            xi = (L*xi).eval(); 
            Eigen::Matrix<double,4,4> X_sample = Eigen::Matrix<double,4,4>::Identity();
            if (error_type == inekf::ErrorType::LeftInvariant) {
                X_sample = X_pose*inekf::Exp_SEK3(xi);
            } else if (error_type == inekf::ErrorType::RightInvariant) {
                X_sample = inekf::Exp_SEK3(xi)*X_pose;
            }
            
            geometry_msgs::Point point;
            point.x = X_sample(0,3);
            point.y = X_sample(1,3);
            point.z = X_sample(2,3);
            marker_pose.points.push_back(point);
        }
        markers_msg.markers.push_back(marker_pose);

        // Add prior landmarks
        mapIntVector3d prior_landmarks = filter_.getPriorLandmarks();
        for (auto it=prior_landmarks.begin(); it!=prior_landmarks.end(); ++it) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = map_frame_id_;
            marker.header.stamp = timestamp;
            marker.header.seq = seq_;
            marker.ns = "prior_landmarks";
            marker.id = it->first;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = it->second(0);
            marker.pose.position.y = it->second(1);
            marker.pose.position.z = it->second(2);
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.lifetime = ros::Duration(dt_);
            markers_msg.markers.push_back(marker);
        }

        // Add estimated landmarks
        for (auto it=estimated_landmarks.begin(); it!=estimated_landmarks.end(); ++it) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = map_frame_id_;
            marker.header.stamp = timestamp;
            marker.header.seq = seq_;
            marker.ns = "estimated_landmarks";
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
            marker.lifetime = ros::Duration(dt_);
            markers_msg.markers.push_back(marker);
        }        

        // Add estimated contacts
        map<int,int> estimated_contacts = filter_.getEstimatedContactPositions();
        for (auto it=estimated_contacts.begin(); it!=estimated_contacts.end(); ++it) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = map_frame_id_;
            marker.header.stamp = timestamp;
            marker.header.seq = seq_;
            marker.ns = "estimated_contacts";
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
            marker.lifetime = ros::Duration(dt_);
            markers_msg.markers.push_back(marker);
        }

        // Add trajectory
        visualization_msgs::Marker traj_marker;
        traj_marker.header.frame_id = map_frame_id_;
        traj_marker.header.stamp = timestamp;
        traj_marker.header.seq = seq_;
        traj_marker.ns = "trajectory";
        traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
        traj_marker.action = visualization_msgs::Marker::ADD;
        traj_marker.id = seq_;
        traj_marker.scale.x = 0.01;
        traj_marker.color.a = 1.0; // Don't forget to set the alpha!
        traj_marker.color.r = 1.0;
        traj_marker.color.g = 0.0;
        traj_marker.color.b = 0.0;
        traj_marker.lifetime = ros::Duration(100.0);
        geometry_msgs::Point point;
        point.x = position(0);
        point.y = position(1);
        point.z = position(2);
        if (seq_ > 0){
            traj_marker.points.push_back(point_prev_);
            traj_marker.points.push_back(point);
            markers_msg.markers.push_back(traj_marker);
        }   
        point_prev_ = point;

        // Publish markers
        visualization_pub_.publish(markers_msg);
    }
    seq_++;
}
