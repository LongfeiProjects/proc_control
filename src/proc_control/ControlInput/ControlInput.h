//
// Created by olavoie on 1/13/18.
//

#ifndef PROC_CONTROL_CONTROLINPUT_H
#define PROC_CONTROL_CONTROLINPUT_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <chrono>

#include "proc_control/EnableControl.h"
#include "provider_kill_mission/KillSwitchMsg.h"
#include "proc_control/property.h"

namespace proc_control {

    class ControlInput {
    public:

        const double DEGREE_TO_RAD = M_PI/180.0;

        //==============================================================================
        // C / D T O R S   S E C T I O N
        //------------------------------------------------------------------------------
        ControlInput(const ros::NodeHandlePtr &nh);
        ~ControlInput();

        //==========================================================================
        // P U B L I C   M E T H O D S

        void OdometryCallback(const nav_msgs::Odometry::ConstPtr &odomIn);

        Eigen::Vector3d GetLinearPosition() const { return linearPosition_; };
        Eigen::Vector3d GetLinearVelocity() const { return linearVelocity_; };
        Eigen::Vector3d GetLinearAcceleration() const { return linearAcceleration_; };

        Eigen::Vector3d GetAngularPosition() const { return angularPosition_; };
        Eigen::Vector3d GetAngularVelocity() const { return angularVelocity_; };
        Eigen::Vector3d GetAngularAcceleration() const { return angularAcceleration_; };


    private:

        void ComputeAcceleration();

        //==========================================================================
        // P R I V A T E   M E M B E R S

        ros::NodeHandlePtr nh_;

        // Subscriber
        ros::Subscriber navigationOdomSubscriber_;

        Eigen::Vector3d linearPosition_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d angularPosition_ = Eigen::Vector3d(0.0, 0.0, 0.0);

        Eigen::Vector3d linearVelocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d lastLinearVelocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d angularVelocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d lastAngularVelocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);

        Eigen::Vector3d linearAcceleration_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d angularAcceleration_ = Eigen::Vector3d(0.0, 0.0, 0.0);

        std::chrono::steady_clock::time_point last_time_;

    };
}


#endif //PROC_CONTROL_CONTROLINPUT_H
