#include "InEKF_ROS.h"
using namespace std;

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
    //this_thread::sleep_for(chrono::milliseconds(1000));
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
        // if (!m_queue_.empty()) {
            m_queue_.pop(m_ptr); // Blocking
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
        // }
    }


    // while (1) {
    //     cout << "size: " << m_queue_.size() << endl;
    //     while (!m_queue_.empty()) {
    //         m_queue_.pop(m_ptr);
    //         //cout << *m_ptr << endl;
    //         cout << "new size: " << m_queue_.size() << endl;
    //     }
    //     cout << "done!!!!!!!!!!!!!"  << endl;
    //     this_thread::sleep_for(chrono::milliseconds(1000));
    // }
}