/**
 * \file	proc_control_node.h
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

#ifndef PROC_CONTROL_CONTROL_SYSTEM_H
#define PROC_CONTROL_CONTROL_SYSTEM_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <proc_control/PositionTarget.h>
#include <chrono>

#include "proc_control/EnableControl.h"
#include "proc_control/EnableThrusters.h"
#include "proc_control/thruster/thruster_manager.h"
#include "proc_control/algorithm/AlgorithmManager.h"
#include "proc_control/SetPositionTarget.h"
#include "proc_control/GetPositionTarget.h"
#include "proc_control/ClearWaypoint.h"
#include "proc_control/SetBoundingBox.h"
#include "proc_control/ResetBoundingBox.h"
#include <provider_kill_mission/KillSwitchMsg.h>

#include "controller_mission/SetManualActuation.h"

#include <mutex>

namespace proc_control {

class ProcControlNode {
 public:
  //==========================================================================
  // C O N S T  ,  T Y P E D E F   A N D   E N U M

  static constexpr double DegreeToRad = M_PI / 180.0f;
  typedef std::array<double, 6> OdometryInfo;

  //==========================================================================
  // P U B L I C   C / D T O R S

  ProcControlNode(const ros::NodeHandlePtr &nh);
  ~ProcControlNode();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void Control();

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  void PublishTargetedPosition();

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odo_in);
  bool EnableControlServiceCallback(proc_control::EnableControlRequest &request,
                                    proc_control::EnableControlResponse &response);
  bool SetManualActuationCallback(controller_mission::SetManualActuationRequest &request,
                                        controller_mission::SetManualActuationResponse &response);
  bool EnableThrusterServiceCallback(proc_control::EnableThrustersRequest &request,
                                     proc_control::EnableThrustersResponse &response);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;

  ros::Subscriber navigation_odom_subscriber_;

  ros::Publisher target_publisher_;
  ros::Publisher target_is_reached_publisher_;

  ros::ServiceServer enable_control_server_;
  ros::ServiceServer enable_thrusters_server_;
  ros::ServiceServer manual_actuation_server_;



  AlgorithmManager algorithm_manager_;
  proc_control::ThrusterManager thruster_manager_;

  OdometryInfo world_position_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double targeted_position_ = 0;
  double manual_actuation_x_, manual_actuation_y_, manual_actuation_z_, manual_actuation_roll_, manual_actuation_pitch_, manual_actuation_yaw_;
  std::array<bool, 6> enable_control_;

  int stability_count_;
  std::chrono::steady_clock::time_point last_time_;

  mutable std::mutex local_position_mutex;
};

} // namespace proc_control

#endif //PROC_CONTROL_CONTROL_SYSTEM_H
