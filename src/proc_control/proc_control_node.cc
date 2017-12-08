/**
 * \file	proc_control_node.cc
 * \author	Jeremie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \coauthor Francis Masse <francis.masse05@gmail.com>
 * \date	10/17/16
 *
 * \copyright Copyright (c) 2017 S.O.N.I.A. AUV All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. AUV software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. AUV software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. AUV software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "proc_control_node.h"
#include <proc_control/TargetReached.h>

namespace proc_control {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ProcControlNode::ProcControlNode(const ros::NodeHandlePtr &nh) :
    nh_(nh),
    stability_count_(0) {
    manual_actuation_x_ = 0.0;
    manual_actuation_y_ = 0.0;
    manual_actuation_z_= 0.0;
    manual_actuation_roll_= 0.0;
    manual_actuation_pitch_= 0.0;
    manual_actuation_yaw_ = 0.0;

  for(int i = 0; i < 6; i++) {
    enable_control_[i] = false;
  }

  navigation_odom_subscriber_ =
      nh->subscribe("/proc_navigation/odom", 100, &ProcControlNode::OdomCallback, this);
  target_is_reached_publisher_ =
      nh->advertise<proc_control::TargetReached>("/proc_control/target_reached", 100);
  enable_control_server_ =
      nh->advertiseService("/proc_control/enable_control", &ProcControlNode::EnableControlServiceCallback, this);
  enable_thrusters_server_ =
      nh->advertiseService("/proc_control/enable_thrusters", &ProcControlNode::EnableThrusterServiceCallback, this);
  target_publisher_ =
                nh->advertise<proc_control::PositionTarget>("/proc_control/current_target", 100);

  manual_actuation_server_ = nh_->advertiseService("/controller_mission/manual_actuation",
                                                   &ProcControlNode::SetManualActuationCallback, this);

}

//------------------------------------------------------------------------------
//
ProcControlNode::~ProcControlNode() { }

//==============================================================================
// M E T H O D   S E C T I O N

//-----------------------------------------------------------------------------
//
void ProcControlNode::Control() {
  std::lock_guard<std::mutex> lock(local_position_mutex);

  std::chrono::steady_clock::time_point now_time = std::chrono::steady_clock::now();
  auto diff = now_time - last_time_;

  double deltaTime_s = double(std::chrono::duration_cast<std::chrono::nanoseconds>(diff).count())/(double(1E9));

  if(deltaTime_s > (0.0001f) ) {

    // Calculate the error
    double z_error;

    z_error = targeted_position_ - world_position_[Z];

    proc_control::TargetReached msg_target_reached;

    if (fabs(z_error) <= 0.3){
        msg_target_reached.target_is_reached = static_cast<unsigned char> (1);
        target_is_reached_publisher_.publish(msg_target_reached);
    } else{
        msg_target_reached.target_is_reached = static_cast<unsigned char> (0);
        target_is_reached_publisher_.publish(msg_target_reached);
    }

    std::array<double, 6> error = {{0,0,0,0,0,0}};

    error[Z] = z_error;

    // Calculate required actuation
    std::array<double, 6> actuation = algorithm_manager_.GetActuationForError(error);
    std::array<double, 3> actuation_lin = {actuation[X], actuation[Y], actuation[Z]};
    std::array<double, 3> actuation_rot = {actuation[ROLL], actuation[PITCH], actuation[YAW]};

    if (!enable_control_[Z])  actuation_lin[Z] = 0.0f;

    if (enable_control_[X])
        actuation[X] = manual_actuation_x_;
    if (enable_control_[Y])
        actuation_lin[Y] = manual_actuation_y_;
    if (enable_control_[ROLL])
        actuation_rot[ROLL] = manual_actuation_roll_;
    if (enable_control_[PITCH])
        actuation_rot[PITCH] = manual_actuation_pitch_;
    if (enable_control_[YAW])
        actuation_rot[YAW] = manual_actuation_yaw_;

    // Process the actuation
    thruster_manager_.CommitEigen(actuation);
  }

  last_time_ = now_time;
}

//-----------------------------------------------------------------------------
//
void ProcControlNode::PublishTargetedPosition() {
  proc_control::PositionTarget msg;
  msg.X = 0;
  msg.Y = 0;
  msg.Z = targeted_position_;
  msg.ROLL = 0;
  msg.PITCH = 0;
  msg.YAW = 0;
  target_publisher_.publish(msg);
}

//-----------------------------------------------------------------------------
//
void ProcControlNode::OdomCallback(const nav_msgs::Odometry::ConstPtr &odo_in) {
  std::lock_guard<std::mutex> lock(local_position_mutex);

  world_position_[X] = odo_in->pose.pose.position.x;
  world_position_[Y] = odo_in->pose.pose.position.y;
  world_position_[Z] = odo_in->pose.pose.position.z;
  world_position_[ROLL] = odo_in->pose.pose.orientation.x;
  world_position_[PITCH] = odo_in->pose.pose.orientation.y;
  world_position_[YAW] = odo_in->pose.pose.orientation.z;
}

//-----------------------------------------------------------------------------
//
bool ProcControlNode::EnableControlServiceCallback(proc_control::EnableControlRequest &request,
                                                   proc_control::EnableControlResponse &response) {
  // If don't care, reuse same value, else check if enable or not.
  if (request.X != request.DONT_CARE) {
    if (request.X == request.ENABLE) {
      enable_control_[X] = true;
    } else {
      enable_control_[X] = false;
    }
  }

  if (request.Y != request.DONT_CARE) {
    if (request.Y == request.ENABLE) {
      enable_control_[Y] = true;
    } else {
      enable_control_[Y] = false;
    }
  }

  if (request.Z != request.DONT_CARE) {
    if (request.Z == request.ENABLE) {
      enable_control_[Z] = true;
      targeted_position_ = world_position_[Z];
    } else {
      enable_control_[Z] = false;
      targeted_position_ = 0.0;
    }
  }

  if (request.ROLL != request.DONT_CARE) {
    if (request.ROLL == request.ENABLE) {
      enable_control_[ROLL] = true;
    } else {
      enable_control_[ROLL] = false;
    }
  }

  if (request.PITCH != request.DONT_CARE) {
    if (request.PITCH == request.ENABLE) {
      enable_control_[PITCH] = true;
    } else {
      enable_control_[PITCH] = false;
    }
  }

  if (request.YAW != request.DONT_CARE) {
    if (request.YAW == request.ENABLE) {
      enable_control_[YAW] = true;
    } else {
      enable_control_[YAW] = false;
    }
  }

  PublishTargetedPosition();

  std::vector<std::string> tmp{"X", "Y", "Z", "ROLL", "PITCH", "YAW"};
  std::cout << "Active control: ";
  for (int i = 0; i < 6; i++) {
    std::cout << tmp[i] + " : " + (enable_control_[i] ? "true" : "false") + "\t";
  }
  std::cout << std::endl;
  return true;
}

bool ProcControlNode::SetManualActuationCallback(controller_mission::SetManualActuationRequest &request,
                                                 controller_mission::SetManualActuationResponse &response) {
    targeted_position_ = request.ActuationZ;
    manual_actuation_x_ = request.ActuationX;
    manual_actuation_y_ = request.ActuationY;
    manual_actuation_roll_ = request.ActuationROLL;
    manual_actuation_pitch_ = request.ActuationROLL;
    manual_actuation_yaw_ = request.ActuationYAW;
    return true;
}

//-----------------------------------------------------------------------------
//
bool ProcControlNode::EnableThrusterServiceCallback(proc_control::EnableThrustersRequest &request,
                                                    proc_control::EnableThrustersResponse &response) {
  this->thruster_manager_.SetEnable(request.isEnable);

  return true;
}


} // namespace proc_control
