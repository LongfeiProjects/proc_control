//
// Created by olavoie on 1/13/18.
//

#include "ControlInput.h"

namespace proc_control{

    ControlInput::ControlInput(const ros::NodeHandlePtr &nh): nh_(nh) {

        navigationOdomSubscriber_ = nh_->subscribe("/proc_navigation/odom", 100, &ControlInput::OdometryCallback, this);
        last_time_ = std::chrono::steady_clock::now();

    }

    ControlInput::~ControlInput(){

        navigationOdomSubscriber_.shutdown();

    }


    void ControlInput::OdometryCallback(const nav_msgs::Odometry::ConstPtr &odomIn) {

        linearPosition_[0] = odomIn->pose.pose.position.x;
        linearPosition_[1] = odomIn->pose.pose.position.y;
        linearPosition_[2] = odomIn->pose.pose.position.z;
        angularPosition_[0] = odomIn->pose.pose.orientation.x * DEGREE_TO_RAD;
        angularPosition_[1] = odomIn->pose.pose.orientation.y * DEGREE_TO_RAD;
        angularPosition_[2] = odomIn->pose.pose.orientation.z * DEGREE_TO_RAD;

        linearVelocity_[0] = odomIn->twist.twist.linear.x;
        linearVelocity_[1] = odomIn->twist.twist.linear.y;
        linearVelocity_[2] = odomIn->twist.twist.linear.z;
        angularVelocity_[0] = odomIn->twist.twist.angular.x;
        angularVelocity_[1] = odomIn->twist.twist.angular.y;
        angularVelocity_[2] = odomIn->twist.twist.angular.z;
        ComputeAcceleration();

    }

    void ControlInput::ComputeAcceleration(){

        std::chrono::steady_clock::time_point time_now = std::chrono::steady_clock::now();

        double deltaTime_s = double(std::chrono::duration_cast<std::chrono::seconds>(time_now - last_time_).count());

        for(int i = 0; i < 3; i++){
            linearAcceleration_[i]  = (linearVelocity_[i] - lastLinearVelocity_[i]) / deltaTime_s;
            angularAcceleration_[i] = (angularVelocity_[i] - lastAngularVelocity_[i]) / deltaTime_s;
        }

        last_time_ = time_now;
        lastLinearVelocity_ = linearVelocity_;
        angularVelocity_ = lastAngularVelocity_;
    }


}