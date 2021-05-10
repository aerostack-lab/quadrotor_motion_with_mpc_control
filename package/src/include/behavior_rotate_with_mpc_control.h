/*!********************************************************************************
 * \brief     rotate implementation
 * \authors   Alberto Rodelgo
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef ROTATE_PID_CONTROL_H
#define ROTATE_PID_CONTROL_H

// System
#include <string>
#include "math.h"

// ROS
#include "std_srvs/Empty.h"
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// Aerostack msgs
#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>
#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>

#include <geometry_msgs/TwistStamped.h>
#include "aerostack_msgs/FlightState.h"
#include <geometry_msgs/PoseStamped.h>
#include "aerostack_msgs/FlightActionCommand.h"

// Aerostack libraries
#include <behavior_execution_controller.h>
#include "aerostack_msgs/SetControlMode.h"
#include <yaml-cpp/yaml.h>


namespace quadrotor_motion_with_mpc_control
{
class BehaviorRotateWithMpcControl : public BehaviorExecutionController
{
  // Constructor
public:
  BehaviorRotateWithMpcControl();
  ~BehaviorRotateWithMpcControl();

private:
  // Congfig variables
  ros::NodeHandle node_handle;
  std::string nspace; 
  // Subscriber
  ros::Subscriber self_localization_speed_sub;
  ros::Subscriber self_localization_pose_sub;  
  ros::Subscriber flight_status_sub;   
  //Publishers
  ros::Publisher motion_reference_pose_pub;
  ros::Publisher flightaction_pub;
  
  // Messages
  geometry_msgs::TwistStamped estimated_speed_msg;
  geometry_msgs::PoseStamped estimated_pose_msg;
  geometry_msgs::PoseStamped reference_pose;
  aerostack_msgs::FlightState flight_state_msg;

  float distance;
  bool quadrotor_moving;
  double angle;
  double angle2;
  double current_angle;
  tf2::Quaternion q_rot;

  bool received_speed;
private:
  // BehaviorExecutionController
  void onConfigure();
  void onActivate();
  void onDeactivate();
  void onExecute();
  bool checkSituation();
  void checkGoal();
  void checkProgress();
  void checkProcesses();

  bool checkQuadrotorStopped();

public: 
  // Callbacks
  void flightStatusCallBack(const aerostack_msgs::FlightState &msg);
  void selfLocalizationSpeedCallBack(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void selfLocalizationPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
};
}

#endif
