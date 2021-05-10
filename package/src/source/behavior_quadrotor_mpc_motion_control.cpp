/*!********************************************************************************
 * \brief     quadrotor_mpc_motion_control implementation
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

#include "../include/behavior_quadrotor_mpc_motion_control.h"

namespace quadrotor_motion_with_mpc_control
{
BehaviorQuadrotorMpcMotionControl::BehaviorQuadrotorMpcMotionControl() : BehaviorExecutionController() { 
  setName("quadrotor_mpc_motion_control"); 
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING); 
}

BehaviorQuadrotorMpcMotionControl::~BehaviorQuadrotorMpcMotionControl() {}

void BehaviorQuadrotorMpcMotionControl::onConfigure()
{
  node_handle = getNodeHandle();
  nspace = getNamespace();
}

bool BehaviorQuadrotorMpcMotionControl::checkSituation()
{
  return true;
}

void BehaviorQuadrotorMpcMotionControl::onActivate()
{
  std::cout << "ACTIVATED MPC" << std::endl;
  ros::ServiceClient start_controller=node_handle.serviceClient<std_srvs::Empty>("/"+nspace+"/mav_linear_mpc/start");
  std_srvs::Empty req;
  start_controller.call(req);
}

void BehaviorQuadrotorMpcMotionControl::onDeactivate(){
  std::cout << "DEACTIVATED MPC" << std::endl;
  ros::ServiceClient stop_controller=node_handle.serviceClient<std_srvs::Empty>("/"+nspace+"/mav_linear_mpc/stop");
  std_srvs::Empty req;
  stop_controller.call(req);
}
void BehaviorQuadrotorMpcMotionControl::onExecute(){}
void BehaviorQuadrotorMpcMotionControl::checkGoal(){}
void BehaviorQuadrotorMpcMotionControl::checkProgress(){}
void BehaviorQuadrotorMpcMotionControl::checkProcesses(){}

}
PLUGINLIB_EXPORT_CLASS(quadrotor_motion_with_mpc_control::BehaviorQuadrotorMpcMotionControl, nodelet::Nodelet)
