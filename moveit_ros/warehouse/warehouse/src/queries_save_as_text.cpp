/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>
//#include <moveit/robot_model/joint_model.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

typedef std::pair<geometry_msgs::Point, geometry_msgs::Quaternion> LinkConstraintPair;
typedef std::map<std::string, LinkConstraintPair> LinkConstraintMap;

void collectLinkConstraints(const moveit_msgs::Constraints& constraints, LinkConstraintMap& lcmap)
{
  for (std::size_t i = 0; i < constraints.position_constraints.size(); ++i)
  {
    LinkConstraintPair lcp;
    const moveit_msgs::PositionConstraint& pc = constraints.position_constraints[i];
    lcp.first = pc.constraint_region.primitive_poses[0].position;
    lcmap[constraints.position_constraints[i].link_name] = lcp;
  }

  for (std::size_t i = 0; i < constraints.orientation_constraints.size(); ++i)
  {
    if (lcmap.count(constraints.orientation_constraints[i].link_name))
    {
      lcmap[constraints.orientation_constraints[i].link_name].second =
          constraints.orientation_constraints[i].orientation;
    }
    else
    {
      ROS_WARN("Orientation constraint for %s has no matching position constraint",
               constraints.orientation_constraints[i].link_name.c_str());
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_warehouse_as_text", ros::init_options::AnonymousName);

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")("host", boost::program_options::value<std::string>(), "Host for the DB.")
    ("port", boost::program_options::value<std::size_t>(), "Port for the DB.")
    ("position", "Defines queries as start state + end effector position instead of full joints goal state.");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }
  
  // Set up db
  warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase();
  if (vm.count("host") && vm.count("port"))
    conn->setParams(vm["host"].as<std::string>(), vm["port"].as<std::size_t>());
  if (!conn->connect())
    return 1;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);

  moveit_warehouse::PlanningSceneStorage pss(conn);
  moveit_warehouse::RobotStateStorage rss(conn);
  moveit_warehouse::ConstraintsStorage cs(conn);

  std::vector<std::string> scene_names;
  pss.getPlanningSceneNames(scene_names);
  
  for (std::size_t i = 0; i < scene_names.size(); ++i)
  {
    moveit_warehouse::PlanningSceneWithMetadata pswm;
    if (pss.getPlanningScene(pswm, scene_names[i]))
    {
      ROS_INFO("Saving scene '%s'", scene_names[i].c_str());
      psm.getPlanningScene()->setPlanningSceneMsg(static_cast<const moveit_msgs::PlanningScene&>(*pswm));
      std::ofstream fout((scene_names[i] + ".scene").c_str());
      psm.getPlanningScene()->saveGeometryToStream(fout);
      fout.close();

      std::vector<std::string> robotStateNames;
      robot_model::RobotModelConstPtr km = psm.getRobotModel();
      // Get start states for scene
      std::stringstream rsregex;
      rsregex << ".*" << scene_names[i] << ".*";
      rss.getKnownRobotStates(rsregex.str(), robotStateNames);
      
      // Get goal constraints for scene
      std::vector<std::string> constraintNames;

      std::stringstream csregex;
      csregex << ".*" << scene_names[i] << ".*";
      cs.getKnownConstraints(csregex.str(), constraintNames);


      // RBRICOUT
      std::vector<std::string> query_names;
      
      std::stringstream pssregex;
      pssregex << ".*";
      pss.getPlanningQueriesNames(pssregex.str(), query_names, scene_names[i]);
      
      std::ofstream qfout((scene_names[i] + ".queries").c_str());
      qfout << scene_names[i] << std::endl;

      for (std::size_t k = 0; k < query_names.size(); ++k)
	{
	  moveit_warehouse::MotionPlanRequestWithMetadata planning_query;
	  try
	    {
	      pss.getPlanningQuery(planning_query, scene_names[i], query_names[k]);
	    }
	  catch (std::exception& ex)
	    {
	      ROS_ERROR("Error loading motion planning query '%s': %s", query_names[i].c_str(), ex.what());
	      continue;
	    }
	    
	  moveit_msgs::MotionPlanRequest plan_request = static_cast<moveit_msgs::MotionPlanRequest>(*planning_query);
	  
	  moveit_msgs::RobotState startState = plan_request.start_state;
	  sensor_msgs::JointState jointState = startState.joint_state;
	  
	  qfout << query_names[k] << std::endl;
	  qfout << plan_request.group_name << std::endl;
	  qfout << "START" << std::endl;
	    
	  for (std::size_t p = 0; p < jointState.position.size(); ++p){
	    qfout << jointState.name[p] << " = ";
	    qfout << jointState.position[p] << std::endl;
	  }
	  qfout << "." << std::endl;

	  std::vector<moveit_msgs::Constraints> query_goal_constraints = plan_request.goal_constraints;
	    
	  if (query_goal_constraints.size() == 1)
	    {
	      moveit_msgs::Constraints query_goal = query_goal_constraints[0];
	      qfout << "GOAL" << std::endl;

	      if(vm.count("position"))
		{
		  qfout << "position_constraint" << std::endl;

		  std::vector<moveit_msgs::JointConstraint> joint_constraints = query_goal.joint_constraints;
		  std::vector<moveit_msgs::PositionConstraint> position_constraints = query_goal.position_constraints;
		  std::vector<moveit_msgs::OrientationConstraint> orientation_constraints = query_goal.orientation_constraints;
		  if(joint_constraints.size() != 0)
		    {
		      std::vector<double> joint_values = {0, 0, 0, 0, 0, 0};
		      for(int i = 0; i<joint_values.size(); ++i)
			{
			  joint_values[i] = joint_constraints[i].position;
			}
		  
		      moveit::core::RobotState my_state(km);

		      my_state.setJointGroupPositions(plan_request.group_name, joint_values);
		      
		      const moveit::core::JointModel* joint_eef = my_state.getJointModel(joint_constraints[joint_constraints.size()-1].joint_name);
		      std::string link_eef = joint_eef->getChildLinkModel()->getName();
		      std::string link_header = startState.joint_state.header.frame_id;

		      const Eigen::Affine3d& link_pose = my_state.getGlobalLinkTransform(link_eef);
		      geometry_msgs::Transform transform;
		      tf::transformEigenToMsg(link_pose, transform);
		  
		      qfout << "Link_header " << link_header << std::endl;
		      qfout << "Link_end_effector " << link_eef << std::endl;
		      
		      qfout << "Position =";
		      qfout << " " << transform.translation.x;
		      qfout << " " << transform.translation.y;
		      qfout << " " << transform.translation.z << std::endl;
		      qfout << "Position_tolerance = " << "0.1 0.1 0.1" << std::endl;
		      qfout << "Orientation =";
		      qfout << " " << transform.rotation.x;
		      qfout << " " << transform.rotation.y;
		      qfout << " " << transform.rotation.z;
		      qfout << " " << transform.rotation.w << std::endl;
		      qfout << "Orientation_tolerance = " << "0.1 0.1 0.1" << std::endl;
		      qfout << "." << std::endl;
		    }
		  else if (position_constraints.size() != 0 && orientation_constraints.size() != 0 && position_constraints[0].link_name == orientation_constraints[0].link_name)
		    {
		      moveit_msgs::BoundingVolume volume = position_constraints[0].constraint_region;

		      qfout << position_constraints[0].header.frame_id << std::endl;
		      qfout << position_constraints[0].link_name << std::endl;
		      
		      if(volume.primitive_poses.size() != 0)
			{
			  qfout << "Position =";
			  qfout << " " << volume.primitive_poses[0].position.x;
			  qfout << " " << volume.primitive_poses[0].position.y;
			  qfout << " " << volume.primitive_poses[0].position.z << std::endl;
			}
		      
		      if(volume.primitives.size() != 0)
			{
			  qfout << "Position_tolerance =";
			  qfout << " " << volume.primitives[0].dimensions[0];
			  qfout << " " << volume.primitives[0].dimensions[1];
			  qfout << " " << volume.primitives[0].dimensions[2] << std::endl;
			}
		      if(orientation_constraints[0].link_name != "")
			{
			  qfout << "Orientation =";
			  qfout << " " << orientation_constraints[0].orientation.x;
			  qfout << " " << orientation_constraints[0].orientation.y;
			  qfout << " " << orientation_constraints[0].orientation.z;
			  qfout << " " << orientation_constraints[0].orientation.w << std::endl;
			  qfout << "Orientation_tolerance =";
			  qfout << " " << orientation_constraints[0].absolute_x_axis_tolerance;
			  qfout << " " << orientation_constraints[0].absolute_y_axis_tolerance;
			  qfout << " " << orientation_constraints[0].absolute_z_axis_tolerance << std::endl;
		        }
		      qfout << "." << std::endl;
		    }
		  else
		    {
		      ROS_ERROR("No proper constraint: need joint constraint or position constraint.");
		    }
		}
	      else
		{
		  qfout << "joint_constraint" << std::endl;
		  
		  std::vector<moveit_msgs::JointConstraint> joint_constraints = query_goal.joint_constraints;
		  for(auto iter = joint_constraints.begin(); iter != joint_constraints.end(); iter++)
		    {
		      qfout << iter->joint_name << " = ";
		      qfout << iter->position << " ";
		      qfout << iter->tolerance_above << " " << iter->tolerance_below << std::endl;
		    }
		  qfout << "." << std::endl;
		}
	    }
	}
      qfout.close();
    }
  }

  ROS_INFO("Done.");

  return 0;
}
