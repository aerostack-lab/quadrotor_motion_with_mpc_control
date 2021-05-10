/*
 * Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <Eigen/Geometry>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "mav_control_interface_impl.h"
#include "parameters.h"

namespace mav_control_interface {

constexpr double MavControlInterfaceImpl::kOdometryWatchdogTimeout;

/*     AEROSTACK       
- Added start and stop services
- Changed topics
*/

MavControlInterfaceImpl::MavControlInterfaceImpl(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                                                 std::shared_ptr<PositionControllerInterface> controller,
                                                 std::shared_ptr<RcInterfaceBase> rc_interface)
    : nh_(nh),
      private_nh_(private_nh),
      rc_interface_(rc_interface)
{
  ros::NodeHandle interface_nh(private_nh, "control_interface");
  controller_ = controller;

  stop_server_srv = nh_.advertiseService("mav_linear_mpc/stop",
                                                                &MavControlInterfaceImpl::stopSrvCall, this);
  start_server_srv = nh_.advertiseService("mav_linear_mpc/start",
                                                                 &MavControlInterfaceImpl::startSrvCall, this);

  rc_interface_->registerUpdatedCallback(&MavControlInterfaceImpl::RcUpdatedCallback, this);

  takeoff_server_ = nh.advertiseService("takeoff", &MavControlInterfaceImpl::TakeoffCallback, this);
  back_to_position_hold_server_ = nh.advertiseService("back_to_position_hold",
                                                      &MavControlInterfaceImpl::BackToPositionHoldCallback,
                                                      this);

  state_machine_.reset(new state_machine::StateMachine(nh_, private_nh_, controller));

  Parameters p;
  interface_nh.param("rc_teleop_max_carrot_distance_position", p.rc_teleop_max_carrot_distance_position_,
                     Parameters::kDefaultRcTeleopMaxCarrotDistancePosition);

  interface_nh.param("rc_teleop_max_carrot_distance_yaw", p.rc_teleop_max_carrot_distance_yaw_,
                     Parameters::kDefaultRcTeleopMaxCarrotDistanceYaw);

  if (p.rc_teleop_max_carrot_distance_yaw_ > M_PI / 2.0) {
    p.rc_teleop_max_carrot_distance_yaw_ = M_PI / 2.0;
    ROS_WARN("rc_teleop_max_carrot_distance_yaw_ was limited to pi/2. This is by far enough.");
  }

  interface_nh.param("rc_max_roll_pitch_command", p.rc_max_roll_pitch_command_,
                     Parameters::kDefaultRcMaxRollPitchCommand);
  interface_nh.param("rc_max_yaw_rate_command", p.rc_max_yaw_rate_command_,
                     Parameters::kDefaultRcMaxYawRateCommand);
  interface_nh.param("takeoff_distance", p.takeoff_distance_, Parameters::kDefaultTakeoffDistance);
  interface_nh.param("takeoff_time", p.takeoff_time_, Parameters::kDefaultTakeoffTime);

  p.stick_deadzone_ = Deadzone<double>(rc_interface->getStickDeadzone());
  state_machine_->SetParameters(p);

  ROS_INFO_STREAM("Created control interface for controller " << controller->getName() <<
                  " and RC " << rc_interface_->getName());

  received_pose = false;
  node_started = false;
}

/*     AEROSTACK       
- Added stop service (shutdown subscribers)
*/
bool MavControlInterfaceImpl::stopSrvCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  if (node_started)
  {
    //Shutdown subscribers
    command_trajectory_subscriber_.shutdown();
    command_trajectory_array_subscriber_.shutdown();
    self_localization_pose_sub_.shutdown();
    self_localization_speed_sub_.shutdown();
    state_machine_->stop();
    node_started = false;
    return true;
  }
  else
  {
    ROS_WARN("Node %s received a stop call when it was already stopped", ros::this_node::getName().c_str());
    return false;
  }
}

/*     AEROSTACK       
- Added start service (create subscribers)
- Changed topics
- Changed odometry to pose and speed for Aerostack
*/
bool MavControlInterfaceImpl::startSrvCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  if (!node_started)
  {
    odometry_watchdog_ = nh_.createTimer(ros::Duration(kOdometryWatchdogTimeout),
                                        &MavControlInterfaceImpl::OdometryWatchdogCallback, this, false, true);

    command_trajectory_subscriber_ = nh_.subscribe("motion_reference/pose", 1,
                                                  &MavControlInterfaceImpl::CommandPoseCallback, this);

    command_trajectory_array_subscriber_ = nh_.subscribe("motion_reference/trajectory", 1,
        &MavControlInterfaceImpl::CommandTrajectoryCallback, this);

    self_localization_pose_sub_ = nh_.subscribe("self_localization/pose", 1,
                                  &MavControlInterfaceImpl::selfLocalizationPoseCallback, this,
                                  ros::TransportHints().tcpNoDelay());

    self_localization_speed_sub_ = nh_.subscribe("self_localization/speed", 1,
                                  &MavControlInterfaceImpl::selfLocalizationSpeedCallback, this,
                                  ros::TransportHints().tcpNoDelay());
    state_machine_->start();
    node_started = true;
    return true;
  }
  else
  {
    ROS_WARN("Node %s received a start call when it was already running", ros::this_node::getName().c_str());
    return false;
  }
}

MavControlInterfaceImpl::~MavControlInterfaceImpl()
{
  state_machine_->stop();
}

void MavControlInterfaceImpl::RcUpdatedCallback(const RcInterfaceBase& rc_interface)
{
  state_machine_->process_event(
      state_machine::RcUpdate(rc_interface_->getRcData(), rc_interface_->isActive(), rc_interface_->isOn()));
}

void MavControlInterfaceImpl::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{

  mav_msgs::EigenTrajectoryPoint reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*msg, &reference);

  mav_msgs::EigenTrajectoryPointDeque references;
  references.push_back(reference);

  state_machine_->process_event(state_machine::ReferenceUpdate(references));
}

void MavControlInterfaceImpl::CommandTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
  int array_size = msg->points.size();
  if (array_size == 0)
    return;
  mav_msgs::EigenTrajectoryPointDeque references;
  mav_msgs::eigenTrajectoryPointDequeFromMsg(*msg, &references);
  state_machine_->process_event(state_machine::ReferenceUpdate(references));
}

/*     AEROSTACK       
- Added pose callback
*/
void MavControlInterfaceImpl::selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  self_localization_pose_msg = *msg;
  received_pose = true;
}

/*     AEROSTACK       
- Added speed callback
- It creates an odometry msg from the pose and speed 
*/
void MavControlInterfaceImpl::selfLocalizationSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
  if (!received_pose){
    return;
  }
  // WORLDFRAME to BODYFRAME
  self_localization_speed_msg = *msg;

  nav_msgs::Odometry odom_msg;
  tf::Quaternion q(self_localization_pose_msg.pose.orientation.x, self_localization_pose_msg.pose.orientation.y, 
  self_localization_pose_msg.pose.orientation.z, self_localization_pose_msg.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double y, p, r;
  m.getEulerYPR(y, p, r);

  Eigen::Vector3f BodyFrame;
  Eigen::Vector3f GlobalFrame;
  Eigen::Matrix3f RotationMat;

  GlobalFrame(0,0) = (+1)*self_localization_speed_msg.twist.linear.x;
  GlobalFrame(1,0) = (+1)*self_localization_speed_msg.twist.linear.y;
  GlobalFrame(2,0) = 0;

  RotationMat(0,0) = cos(y);
  RotationMat(1,0) = -sin(y);
  RotationMat(2,0) = 0;

  RotationMat(0,1) = sin(y);
  RotationMat(1,1) = cos(y);
  RotationMat(2,1) = 0;

  RotationMat(0,2) = 0;
  RotationMat(1,2) = 0;
  RotationMat(2,2) = 1;

  BodyFrame = RotationMat.transpose().inverse()*GlobalFrame;

  //Publish odometry
  odom_msg.header = self_localization_pose_msg.header;
  odom_msg.pose.pose = self_localization_pose_msg.pose;
  odom_msg.twist.twist = self_localization_speed_msg.twist;
  odom_msg.twist.twist.linear.x = BodyFrame(0);
  odom_msg.twist.twist.linear.y = BodyFrame(1);

  // Get odometry from self_localization topics 
  mav_msgs::EigenOdometry odometry;
  eigenOdometryFromMsg(odom_msg, &odometry);
  this->processOdometry(odometry);
}

void MavControlInterfaceImpl::processOdometry(mav_msgs::EigenOdometry odometry)
{
  ROS_INFO_ONCE("Control interface got first odometry message.");
  // Stamp odometry upon reception to be robust against timestamps "in the future".
  odometry.timestamp_ns = ros::Time::now().toNSec();
  state_machine_->process_event(state_machine::OdometryUpdate(odometry));
}

void MavControlInterfaceImpl::OdometryWatchdogCallback(const ros::TimerEvent& e)
{
  state_machine_->process_event(state_machine::OdometryWatchdog());
}

bool MavControlInterfaceImpl::TakeoffCallback(std_srvs::Empty::Request& request,
                                              std_srvs::Empty::Response& response)
{
  ROS_INFO("Take off event sent");
  state_machine_->process_event(state_machine::Takeoff());
  return true;
}

bool MavControlInterfaceImpl::BackToPositionHoldCallback(std_srvs::Empty::Request& request,
                                                         std_srvs::Empty::Response& response)
{
  state_machine_->process_event(state_machine::BackToPositionHold());
  return true;
}

}  // end namespace mav_control_interface

