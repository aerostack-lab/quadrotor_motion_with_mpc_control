/*!********************************************************************************
 * \brief     follow_path implementation
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

#include "../include/behavior_follow_path.h"
#include <pluginlib/class_list_macros.h>

namespace quadrotor_motion_with_mpc_control
{
BehaviorFollowPath::BehaviorFollowPath() : BehaviorExecutionController() { 
  setName("follow_path_with_mpc_control");
  setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
}

BehaviorFollowPath::~BehaviorFollowPath() {}

void BehaviorFollowPath::onConfigure()
{
  node_handle = getNodeHandle();
  nspace = getNamespace();
  ros::param::get("~drone_max_speed", DRONE_MAX_SPEED);
}


bool BehaviorFollowPath::checkSituation()
{
  return true;
}

void BehaviorFollowPath::checkGoal(){ 
  if(initiated && remaining_points <= 1 && checkFinalDistance() < 0.1 && checkQuadrotorStopped()){
    initiated = false;
    BehaviorExecutionController::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
  } 
}

void BehaviorFollowPath::onExecute()
{
  
}

double BehaviorFollowPath::checkFinalDistance(){
  return abs(sqrt(pow(last_path_point.pose.position.x-estimated_pose_msg.pose.position.x,2)
  +pow(last_path_point.pose.position.y-estimated_pose_msg.pose.position.y,2)
  +pow(last_path_point.pose.position.z-estimated_pose_msg.pose.position.z,2)));
  
}

void BehaviorFollowPath::checkProgress() {
  if(path_blocked){
    path_blocked=false;
    path_blocked_sub.shutdown();
    BehaviorExecutionController::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
  }
  if (!execute) BehaviorExecutionController::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);

  //Quadrotor is too far from the target and it is not moving
  last_target_pose = current_target_pose;
  float targets_distance = abs(sqrt(pow(last_target_pose.pose.position.x-current_target_pose.pose.position.x,2)+pow(last_target_pose.pose.position.y-current_target_pose.pose.position.y,2)+pow(last_target_pose.pose.position.z-current_target_pose.pose.position.z,2)));
  float quadrotor_distance = abs(sqrt(pow(current_target_pose.pose.position.x-estimated_pose_msg.pose.position.x,2)+pow(current_target_pose.pose.position.y-estimated_pose_msg.pose.position.y,2)+pow(current_target_pose.pose.position.z-estimated_pose_msg.pose.position.z,2)));
  if(remaining_points > 0 && targets_distance > quadrotor_distance * 2 && checkQuadrotorStopped()) BehaviorExecutionController::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
}

void BehaviorFollowPath::onActivate()
{
 //Subscribers
  self_localization_speed_sub = node_handle.subscribe("/" + nspace + "/self_localization/speed", 1, &BehaviorFollowPath::selfLocalizationSpeedCallBack, this);
  self_localization_pose_sub = node_handle.subscribe("/" + nspace + "/self_localization/pose", 1, &BehaviorFollowPath::selfLocalizationPoseCallBack, this);
  motion_reference_pose_sub = node_handle.subscribe("/" + nspace + "/motion_reference/assumed_pose", 1, &BehaviorFollowPath::motionReferencePoseCallBack, this);
  motion_reference_remaining_path = node_handle.subscribe("/" + nspace + "/motion_reference/remaining_path", 1, &BehaviorFollowPath::motionReferenceRemainingPathCallBack, this);
  path_blocked_sub = node_handle.subscribe("/" + nspace + "/environnment/path_blocked_by_obstacle", 1, &BehaviorFollowPath::pathBlockedCallBack, this);
  
  //Publishers
  motion_reference_trajectory_pub = node_handle.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/" + nspace + "/motion_reference/trajectory",1, true);
  flightaction_pub = node_handle.advertise<aerostack_msgs::FlightActionCommand>("/"+nspace+"/actuator_command/flight_action", 1, true);

  //Get current drone pose
  geometry_msgs::PoseStamped::ConstPtr sharedPose;
  geometry_msgs::PoseStamped drone_initial_pose;
  sharedPose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/" + nspace + "/self_localization/pose",node_handle);
  if(sharedPose != NULL){
    drone_initial_pose = *sharedPose;
  }
  remaining_points = 0;
  initiated = false;
  execute = true;
  path_blocked=false;

  //Checks if path distance is not too long and argument is defined
  std::string arguments = getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  //constant or path_facing
  std::string yaw_parameter = "path_facing"; //default value
  if(config_file["yaw"].IsDefined()){
    yaw_parameter=config_file["yaw"].as<std::string>();
  }

  float distance = 0;
  if(config_file["path"].IsDefined()){
    std::vector<std::vector<double>> points=config_file["path"].as<std::vector<std::vector<double>>>();
    geometry_msgs::Transform trans_ref;
    trajectory_msgs::MultiDOFJointTrajectoryPoint point_ref;
    
    //MPC goes faster to first point. We add its current position as a first point to avoid this
    if(yaw_parameter == "constant"){
      //First point: drone pose
      trans_ref.translation.x = drone_initial_pose.pose.position.x;
      trans_ref.translation.y = drone_initial_pose.pose.position.y;
      trans_ref.translation.z = drone_initial_pose.pose.position.z;
      trans_ref.rotation = drone_initial_pose.pose.orientation;
      point_ref.transforms.clear();
      point_ref.transforms.push_back(trans_ref);
      reference_path.points.push_back(point_ref);
      //Second point
      trans_ref.translation.x = points[0][0];
      trans_ref.translation.y = points[0][1];
      trans_ref.translation.z = points[0][2];
      trans_ref.rotation = drone_initial_pose.pose.orientation;
      //Calculates the distance to the point so it can estimate the time
      double distance_to_point = abs(sqrt(pow(drone_initial_pose.pose.position.x-points[0][0],2)
      +pow(drone_initial_pose.pose.position.y-points[0][1],2)
      +pow(drone_initial_pose.pose.position.z-points[0][2],2)));
      //Calculates time to reach point
      double time_to_point;
      time_to_point += distance_to_point/DRONE_MAX_SPEED;
      ros::Duration d(time_to_point);
      point_ref.time_from_start = d;
      point_ref.transforms.clear();
      point_ref.transforms.push_back(trans_ref);
      reference_path.points.push_back(point_ref);
      for(int i=1;i<points.size();i++){
        trans_ref.translation.x = points[i][0];
        trans_ref.translation.y = points[i][1];
        trans_ref.translation.z = points[i][2];
        trans_ref.rotation = drone_initial_pose.pose.orientation;
        //Calculates the distance to the point so it can estimate the time
        distance_to_point = abs(sqrt(pow(points[i-1][0]-points[i][0],2)+pow(points[i-1][1]-points[i][1],2)+pow(points[i-1][2]-points[i][2],2)));
        distance += distance_to_point;
        //Calculates time to reach point
        time_to_point += distance_to_point/DRONE_MAX_SPEED;
        ros::Duration d2(time_to_point);
        point_ref.time_from_start = d2;
        point_ref.transforms.clear();
        point_ref.transforms.push_back(trans_ref);
        reference_path.points.push_back(point_ref);
      }
    }else{//Path facing
      //First point: drone pose
      trans_ref.translation.x = drone_initial_pose.pose.position.x;
      trans_ref.translation.y = drone_initial_pose.pose.position.y;
      trans_ref.translation.z = drone_initial_pose.pose.position.z;
      trans_ref.rotation = drone_initial_pose.pose.orientation;
      point_ref.transforms.clear();
      point_ref.transforms.push_back(trans_ref);
      reference_path.points.push_back(point_ref);
      //Second point
      trans_ref.translation.x = points[0][0];
      trans_ref.translation.y = points[0][1];
      trans_ref.translation.z = points[0][2];
      //Orientation faces next target point
      tf2::Quaternion orientation_quaternion;
      orientation_quaternion.setRPY(0, 0, atan2((trans_ref.translation.y-drone_initial_pose.pose.position.y),(trans_ref.translation.x-drone_initial_pose.pose.position.x)));
      trans_ref.rotation.w = orientation_quaternion.getW();
      trans_ref.rotation.x = orientation_quaternion.getX();
      trans_ref.rotation.y = orientation_quaternion.getY();
      trans_ref.rotation.z = orientation_quaternion.getZ();  
      //Calculates the distance to the point so it can estimate the time
      double distance_to_point = abs(sqrt(pow(drone_initial_pose.pose.position.x-points[0][0],2)
      +pow(drone_initial_pose.pose.position.y-points[0][1],2)
      +pow(drone_initial_pose.pose.position.z-points[0][2],2)));
      //Calculates time to reach point
      double time_to_point;
      time_to_point += distance_to_point/DRONE_MAX_SPEED;
      ros::Duration d(time_to_point);
      point_ref.time_from_start = d;
      point_ref.transforms.clear();
      point_ref.transforms.push_back(trans_ref);
      reference_path.points.push_back(point_ref);
      for(int i=1;i<points.size();i++){
        trans_ref.translation.x = points[i][0];
        trans_ref.translation.y = points[i][1];
        trans_ref.translation.z = points[i][2];
        //Orientation faces next target point
        tf2::Quaternion orientation_quaternion;
        orientation_quaternion.setRPY(0, 0, atan2((trans_ref.translation.y-points[i-1][1]),(trans_ref.translation.x-points[i-1][0])));
        trans_ref.rotation.w = orientation_quaternion.getW();
        trans_ref.rotation.x = orientation_quaternion.getX();
        trans_ref.rotation.y = orientation_quaternion.getY();
        trans_ref.rotation.z = orientation_quaternion.getZ();  
        //Calculates the distance to the point so it can estimate the time
        distance_to_point = abs(sqrt(pow(points[i-1][0]-points[i][0],2)+pow(points[i-1][1]-points[i][1],2)+pow(points[i-1][2]-points[i][2],2)));
        distance += distance_to_point;
        //Calculates time to reach point
        time_to_point += distance_to_point/DRONE_MAX_SPEED;
        ros::Duration d2(time_to_point);
        point_ref.time_from_start = d2;
        point_ref.transforms.clear();
        point_ref.transforms.push_back(trans_ref);
        reference_path.points.push_back(point_ref);
      }
    }

    if (distance > MAX_DISTANCE){
      setErrorMessage("Error: Path is too long");
      std::cout<<"Error: Path is too long"<<std::endl;    
      execute = false;     
      return; 
    }

  }else{
    setErrorMessage("Error: Path is not defined");
    std::cout<<"Error: Path is not defined"<<std::endl;    
    execute = false;   
    return;
  }
  //Send trajectory
  reference_path.header.stamp = ros::Time::now();
  motion_reference_trajectory_pub.publish(reference_path);
  //Send flight action
  aerostack_msgs::FlightActionCommand flight_action_msg;
  flight_action_msg.action = aerostack_msgs::FlightActionCommand::MOVE;
  flightaction_pub.publish(flight_action_msg);
}

void BehaviorFollowPath::onDeactivate()
{
  reference_path.points.clear();
  self_localization_pose_sub.shutdown();
  motion_reference_trajectory_pub.shutdown();
  self_localization_speed_sub.shutdown();
  motion_reference_remaining_path.shutdown();
  motion_reference_pose_sub.shutdown();
  path_blocked_sub.shutdown();
}

bool BehaviorFollowPath::checkQuadrotorStopped()
{
  if (received_speed){
    if (abs(estimated_speed_msg.twist.linear.x) <= 0.30 && abs(estimated_speed_msg.twist.linear.y) <= 0.30 && abs(estimated_speed_msg.twist.linear.z) <= 0.15){
        return true;
    }else{
      return false;
    }    
  }else{
    return false;
  }
}

void BehaviorFollowPath::checkProcesses() 
{ 
 
}


// Callbacks
void BehaviorFollowPath::pathBlockedCallBack(const std_msgs::Bool &msg){
  path_blocked = msg.data;
}

void BehaviorFollowPath::selfLocalizationSpeedCallBack(const geometry_msgs::TwistStamped &msg){ 
  estimated_speed_msg = msg; 
  received_speed = true;
}
void BehaviorFollowPath::selfLocalizationPoseCallBack(const geometry_msgs::PoseStamped &msg){
  estimated_pose_msg = msg;
}
void BehaviorFollowPath::motionReferencePoseCallBack(const geometry_msgs::PoseStamped &msg){
  if (current_target_pose.pose.position.x != msg.pose.position.x || current_target_pose.pose.position.y != msg.pose.position.y || current_target_pose.pose.position.z != msg.pose.position.z ){
      last_target_pose = current_target_pose;
      current_target_pose = msg;
  }
}

void BehaviorFollowPath::motionReferenceRemainingPathCallBack(const nav_msgs::Path &msg){
  remaining_path = msg;
  if (remaining_path.poses.size() == 1){
    last_path_point = remaining_path.poses[0];
  }
  remaining_points = remaining_path.poses.size();
  initiated = true;
}

}
PLUGINLIB_EXPORT_CLASS(quadrotor_motion_with_mpc_control::BehaviorFollowPath, nodelet::Nodelet)