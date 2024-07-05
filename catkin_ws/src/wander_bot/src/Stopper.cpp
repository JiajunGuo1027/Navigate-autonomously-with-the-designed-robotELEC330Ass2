#include "Stopper.h"
#include "geometry_msgs/Twist.h"

// Constructor for the Stopper class
Stopper::Stopper() : p_error_(0), i_error_(0), d_error_(0), prev_error_(0), prev_time_(ros::Time::now()) {
    // Initialize PID parameters
    Kp_ = 6.0; // Proportionality coefficient
    Ki_ = 0.01; // Integration coefficient
    Kd_ = 0.01; // Differential coefficient

    // Advertise a new publisher for the robot's velocity command topic
    commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Subscribe to the simulated robot's laser scan topic
    laserSub = node.subscribe("scan", 1, &Stopper::scanCallback, this);
}

// Function to move the robot forward
void Stopper::moveForward() {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = FORWARD_SPEED;
    commandPub.publish(msg);
}

// Callback function for processing laser scan data
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
    isObstacleInFront = false;

    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

    int midIndex = (minIndex + maxIndex) / 2;
    bool leftObstacle = false;
    bool rightObstacle = false;

    // Loop through laser scan data to detect obstacles
    for (int currentIndex = minIndex; currentIndex <= maxIndex; currentIndex++) {
        if (scan->ranges[currentIndex] < MIN_DIST_FROM_OBSTACLE) {
            if (scan->ranges[currentIndex] > 0.01) {
                isObstacleInFront = true;
                if (currentIndex <= midIndex) {
                    rightObstacle = true;
                } else {
                    leftObstacle = true;
                }
            }
        }
    }

    // If an obstacle is detected in front of the robot
    if (isObstacleInFront) {
        // Calculate distance error
        double error = MIN_DIST_FROM_OBSTACLE - scan->ranges[midIndex];

        // Calculate current time
        ros::Time current_time = ros::Time::now();

        // Calculate time interval
        double dt = (current_time - prev_time_).toSec();

        // Update errors for PID controller
        p_error_ = error;
        i_error_ += error * dt;
        d_error_ = (error - prev_error_) / dt;

        // Calculate PID output
        double pid_output = Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;

        // Limit PID output value
        double angular_speed = std::max(std::min(pid_output, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED);

        // Set velocity message
        geometry_msgs::Twist msg;
        msg.linear.x = 0;
        msg.angular.z = angular_speed;

        // Publish velocity message
        commandPub.publish(msg);

        // Update previous error and time
        prev_error_ = error;
        prev_time_ = current_time;
    } else {
        moveForward();
    }
}

// Function to start moving the robot
void Stopper::startMoving() {
    ros::Rate rate(10);
    ROS_INFO("Start moving");

// Keep the loop until user presses ctrl+c or the robot got too close to an obstacle
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

// Function to stop the robot's movement
void Stopper::stopMoving() {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = 0;
    commandPub.publish(msg);
}

