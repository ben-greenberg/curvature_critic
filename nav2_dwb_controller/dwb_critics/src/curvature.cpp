/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "dwb_critics/curvature.hpp"
#include <math.h>
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::CurvatureCritic, dwb_core::TrajectoryCritic)

using nav2_util::declare_parameter_if_not_declared;

namespace dwb_critics
{

void CurvatureCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".scale", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".maximum_penalty", rclcpp::ParameterValue(1.0));

  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".scale", scale_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".maximum_penalty", maximum_penalty_);

}

double CurvatureCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  if(!traj.poses.empty())
  {
    const auto & last_pose  = traj.poses.back();
    const auto & first_pose = traj.poses.front();
    // return fminimum(maximum_penalty_, scale_ * fabs((last_pose.theta - first_pose.theta)/sqrt(pow((last_pose.y - first_pose.y), 2) + pow((last_pose.x - first_pose.x), 2))));
    return  scale_ * fabs((last_pose.theta - first_pose.theta) * (1 + pow((0.5), sqrt(pow((last_pose.y - first_pose.y), 2) + pow((last_pose.x - first_pose.x), 2)))));
  }
  else
    return 0;
}

}  // namespace dwb_critics
