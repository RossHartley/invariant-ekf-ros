#include "InEKF_ROS.h"
using namespace std;
using namespace inekf;

// Constructor
InEKF_ROS::InEKF_ROS(ros::NodeHandle n) : n_(n) {}

// Initialize ROS node and filter
void InEKF_ROS::init() {
    // Initialize invariant-ekf filter
    cout << "Robot's state is initialized to: \n";
    cout << filter_.getState() << endl;
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
}

// IMU Callback function
void InEKF_ROS::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    shared_ptr<Measurement> ptr(new Measurement(msg));
    m_queue_.push(ptr);
}


void InEKF_ROS::mainFilteringThread() {
    cout << "Inside Main Filtering Thread\n";
    Eigen::VectorXd m;
    shared_ptr<Measurement> m_ptr, imu_ptr_last;
    double t, t_last;

    // Block until first IMU measurement is received
    while (true) {
        if (!m_queue_.empty()) { 
            m_queue_.pop(m_ptr);
            if (m_ptr->getType() == IMU) {
                ROS_INFO("First IMU measurement received. Starting Filter.");
                t_last = m_ptr->getTime();
                imu_ptr_last = m_ptr;
                break;
            }
        }
        this_thread::sleep_for(chrono::milliseconds(10));
    }
    
    // Main loop
    while (true) {
        if (m_queue_.size() > MAX_QUEUE_SIZE) {
            ROS_WARN("Measurement queue size (%d) is greater than MAX_QUEUE_SIZE. Filter is not realtime!", m_queue_.size());
            // cout << "Queue size: " <<  m_queue_.size() << endl;
        }   
        // Retrieve next measurement (Blocking)
        m_queue_.pop(m_ptr);
        // Handle measurement
        switch (m_ptr->getType()) {
            case IMU:
                //ROS_INFO("Propagating state with IMU measurement.");
                t = m_ptr->getTime();
                m = imu_ptr_last->getData();
                //cout << "t, t_last, dt: " << t << ", " << t_last << ", " << t-t_last << endl;
                //cout << "IMU data: \n" << imu_ptr_last->getData() << endl;
                filter_.Propagate(m, t - t_last);
                //cout << "queue size: " << m_queue_.size() << endl;
                //cout << filter_.getState() << endl;
                t_last = t;
                imu_ptr_last = m_ptr;
                break;
            default:
                cout << "Unknown measurement, skipping...\n";
        }
    }
}


// Thread for publishing the output of the filter
void InEKF_ROS::outputPublishingThread() {
    // Create pose publisher
    ros::Publisher pose_pub = n_.advertise<geometry_msgs::PoseStamped>("/pose", 1000);
    static tf::TransformBroadcaster tf_broadcaster;
    uint32_t pose_seq = 0;

    ros::Rate loop_rate(100);
    // Main loop
    while(true) {
        // Create pose message
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.seq = pose_seq;
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

        // Send pose message
        pose_pub.publish(pose_msg);
        pose_seq++;

        // Create tf message
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(position(0),position(1),position(2)) );
        transform.setRotation( tf::Quaternion(orientation.x(),orientation.y(),orientation.z(),orientation.w()) );
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/imu"));

        loop_rate.sleep();
    }

}
