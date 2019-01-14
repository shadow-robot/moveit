/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ryan Luna */

#include <moveit/benchmarks/ModifiedBenchmarkOptionsWithoutTweaksAndRestarts.h>

using namespace moveit_ros_benchmarks;

ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::ModifiedBenchmarkOptionsWithoutTweaksAndRestarts()
{
}

ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::ModifiedBenchmarkOptionsWithoutTweaksAndRestarts(const std::string& ros_namespace)
{
  readBenchmarkOptions(ros_namespace);
}

ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::~ModifiedBenchmarkOptionsWithoutTweaksAndRestarts()
{
}

void ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::setNamespace(const std::string& ros_namespace)
{
  readBenchmarkOptions(ros_namespace);
}

void ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::readBenchmarkOptions(const std::string& ros_namespace)
{
  ros::NodeHandle nh(ros_namespace);

  XmlRpc::XmlRpcValue benchmark_config;
  if (nh.getParam("benchmark_config", benchmark_config))
  {
    readWarehouseOptions(nh);
    readBenchmarkParameters(nh);
    readPlannerNames(nh);
  }
  else
  {
    ROS_WARN("No benchmark_config found on param server");
  }
}

const std::string& ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getHostName() const
{
  return hostname_;
}

int ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getPort() const
{
  return port_;
}

const std::string& ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getSceneName() const
{
  return scene_name_;
}

int ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getNumRuns() const
{
  return runs_;
}

double ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getTimeout() const
{
  return timeout_;
}

const std::string& ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getBenchmarkName() const
{
  return benchmark_name_;
}

const std::string& ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getGroupName() const
{
  return group_name_;
}

const std::string& ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getOutputDirectory() const
{
  return output_directory_;
}

const std::string& ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getQueryRegex() const
{
  return query_regex_;
}

const std::string& ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getMetricChoice() const
{
  return metric_choice_;
}

const std::string& ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getStartStateRegex() const
{
  return start_state_regex_;
}

const std::string& ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getGoalConstraintRegex() const
{
  return goal_constraint_regex_;
}

const std::string& ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getPathConstraintRegex() const
{
  return path_constraint_regex_;
}

const std::string& ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getTrajectoryConstraintRegex() const
{
  return trajectory_constraint_regex_;
}

void ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getGoalOffsets(std::vector<double>& offsets) const
{
  offsets.resize(6);
  memcpy(&offsets[0], goal_offsets, 6 * sizeof(double));
}

const std::map<std::string, std::vector<std::string>>& ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getPlannerConfigurations() const
{
  return planners_;
}

void ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getPlannerPluginList(std::vector<std::string>& plugin_list) const
{
  plugin_list.clear();
  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners_.begin(); it != planners_.end();
       ++it)
    plugin_list.push_back(it->first);
}

const std::string& ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getWorkspaceFrameID() const
{
  return workspace_.header.frame_id;
}

const moveit_msgs::WorkspaceParameters& ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::getWorkspaceParameters() const
{
  return workspace_;
}

void ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::readWarehouseOptions(ros::NodeHandle& nh)
{
  nh.param(std::string("benchmark_config/warehouse/host"), hostname_, std::string("127.0.0.1"));
  nh.param(std::string("benchmark_config/warehouse/port"), port_, 33829);

  if (!nh.getParam("benchmark_config/warehouse/scene_name", scene_name_))
    ROS_WARN("Benchmark scene_name NOT specified");

  ROS_INFO("Benchmark host: %s", hostname_.c_str());
  ROS_INFO("Benchmark port: %d", port_);
  ROS_INFO("Benchmark scene: %s", scene_name_.c_str());
}

void ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::readBenchmarkParameters(ros::NodeHandle& nh)
{
  nh.param(std::string("benchmark_config/parameters/name"), benchmark_name_, std::string(""));
  nh.param(std::string("benchmark_config/parameters/runs"), runs_, 10);
  nh.param(std::string("benchmark_config/parameters/timeout"), timeout_, 10.0);
  nh.param(std::string("benchmark_config/parameters/output_directory"), output_directory_, std::string(""));
  nh.param(std::string("benchmark_config/parameters/queries"), query_regex_, std::string(".*"));
  nh.param(std::string("benchmark_config/parameters/metric_choice"), metric_choice_, std::string("relevancy"));
  nh.param(std::string("benchmark_config/parameters/start_states"), start_state_regex_, std::string(""));
  nh.param(std::string("benchmark_config/parameters/goal_constraints"), goal_constraint_regex_, std::string(""));
  nh.param(std::string("benchmark_config/parameters/path_constraints"), path_constraint_regex_, std::string(""));
  nh.param(std::string("benchmark_config/parameters/trajectory_constraints"), trajectory_constraint_regex_,
           std::string(""));

  if (!nh.getParam(std::string("benchmark_config/parameters/group"), group_name_))
    ROS_WARN("Benchmark group NOT specified");

  if (nh.hasParam("benchmark_config/parameters/workspace"))
    readWorkspaceParameters(nh);

  // Reading in goal_offset (or defaulting to zero)
  nh.param(std::string("benchmark_config/parameters/goal_offset/x"), goal_offsets[0], 0.0);
  nh.param(std::string("benchmark_config/parameters/goal_offset/y"), goal_offsets[1], 0.0);
  nh.param(std::string("benchmark_config/parameters/goal_offset/z"), goal_offsets[2], 0.0);
  nh.param(std::string("benchmark_config/parameters/goal_offset/roll"), goal_offsets[3], 0.0);
  nh.param(std::string("benchmark_config/parameters/goal_offset/pitch"), goal_offsets[4], 0.0);
  nh.param(std::string("benchmark_config/parameters/goal_offset/yaw"), goal_offsets[5], 0.0);

  ROS_INFO("Benchmark name: '%s'", benchmark_name_.c_str());
  ROS_INFO("Benchmark #runs: %d", runs_);
  ROS_INFO("Benchmark timeout: %f secs", timeout_);
  ROS_INFO("Benchmark group: %s", group_name_.c_str());
  ROS_INFO("Benchmark query regex: '%s'", query_regex_.c_str());
  ROS_INFO("Optimization metric choice: '%s'", metric_choice_.c_str());
  ROS_INFO("Benchmark start state regex: '%s':", start_state_regex_.c_str());
  ROS_INFO("Benchmark goal constraint regex: '%s':", goal_constraint_regex_.c_str());
  ROS_INFO("Benchmark path constraint regex: '%s':", path_constraint_regex_.c_str());
  ROS_INFO("Benchmark goal offsets (%f %f %f, %f %f %f)", goal_offsets[0], goal_offsets[1], goal_offsets[2],
           goal_offsets[3], goal_offsets[4], goal_offsets[5]);
  ROS_INFO("Benchmark output directory: %s", output_directory_.c_str());
  ROS_INFO_STREAM("Benchmark workspace: " << workspace_);
}

void ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::readWorkspaceParameters(ros::NodeHandle& nh)
{
  // Make sure all params exist
  if (!nh.getParam("benchmark_config/parameters/workspace/frame_id", workspace_.header.frame_id))
    ROS_WARN("Workspace frame_id not specified in benchmark config");

  nh.param(std::string("benchmark_config/parameters/workspace/min_corner/x"), workspace_.min_corner.x, 0.0);
  nh.param(std::string("benchmark_config/parameters/workspace/min_corner/y"), workspace_.min_corner.y, 0.0);
  nh.param(std::string("benchmark_config/parameters/workspace/min_corner/z"), workspace_.min_corner.z, 0.0);

  nh.param(std::string("benchmark_config/parameters/workspace/max_corner/x"), workspace_.max_corner.x, 0.0);
  nh.param(std::string("benchmark_config/parameters/workspace/max_corner/y"), workspace_.max_corner.y, 0.0);
  nh.param(std::string("benchmark_config/parameters/workspace/max_corner/z"), workspace_.max_corner.z, 0.0);

  workspace_.header.stamp = ros::Time::now();
}

void ModifiedBenchmarkOptionsWithoutTweaksAndRestarts::readPlannerNames(ros::NodeHandle& nh) //This actually only reads the name of the planners and plugins, and doesn't fetch at any moment their parameters! :(
{
  planners_.clear();

  XmlRpc::XmlRpcValue planner_sets;
  if (nh.getParam("benchmark_config/planners", planner_sets))
  {
    if (planner_sets.getType() != XmlRpc::XmlRpcValue::TypeArray) //an array of structs plugin+associatedPlanner(s)
    {
      ROS_ERROR("Expected a list of structures, each composed of planner lists associated with the plugin they come from");
      return;
    }

    for (int i = 0; i < planner_sets.size(); ++i)
    {
      if (planner_sets[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_WARN("Improper YAML type for planner(s)' origins");
        continue;
      }
      
      if (!planner_sets[i].hasMember("plugin") || !planner_sets[i].hasMember("planners"))
      {
        ROS_WARN("Malformed YAML for planner(s)' origins. It either lacks the plugin name and/or its associated chosen planner(s) to benchmark");
        continue;
      }

      if (planner_sets[i]["planners"].getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_WARN("Expected a list of planner(s) to benchmark");
        continue;
      }

      std::string plugin = planner_sets[i]["plugin"];
      ROS_INFO("Reading in planner names for plugin '%s'", plugin.c_str());

      std::vector<std::string> planners;
      for (int j = 0; j < planner_sets[i]["planners"].size(); ++j)
        planners.push_back(planner_sets[i]["planners"][j]);

      for (std::size_t j = 0; j < planners.size(); ++j)
        ROS_INFO("  [%lu]: %s", j, planners[j].c_str());

      planners_[plugin] = planners;
    }
  }
}