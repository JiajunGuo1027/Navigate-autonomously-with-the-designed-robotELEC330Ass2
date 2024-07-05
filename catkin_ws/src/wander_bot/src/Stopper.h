#ifndef WANDER_BOT_SRC_STOPPER_H_
#define WANDER_BOT_SRC_STOPPER_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Stopper {
public:
    // Tunable parameters
    constexpr static double FORWARD_SPEED = 0.3;
    constexpr static double MIN_SCAN_ANGLE = -15.0/180*M_PI;
    constexpr static double MAX_SCAN_ANGLE = +15.0/180*M_PI;
    constexpr static float MIN_DIST_FROM_OBSTACLE = 0.8;

    Stopper();

    void startMoving();
    void stopMoving();

    bool isObstacleInFront = false;

private:
    ros::NodeHandle node;
    ros::Publisher commandPub; // Publisher to the robot's velocity command topic
    ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
    ros::Time prev_time_; // Time of the last update

    void moveForward();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    // PID controller variables
    double p_error_; // Error
    double i_error_; // Accumulative error
    double d_error_; // Error difference
    double prev_error_; //Last error
    double Kp_, Ki_, Kd_; // PID parameter


    //Parameters
    double TURN_SPEED = 2.0;
    double MAX_ANGULAR_SPEED = TURN_SPEED;
};

#endif