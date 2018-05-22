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
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <ros/ros.h>

#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_interface/planning_interface.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";


void parseStartFormat(std::istream& in, planning_scene_monitor::PlanningSceneMonitor* psm,
                moveit_warehouse::RobotStateStorage* rs, moveit_msgs::RobotState& startState)
{
  std::map<std::string, double> v;
  
  std::string joint;
  std::string marker;
  double value;
  in >> joint;
  
  while (joint != "." && in.good() && !in.eof())
    {
      in >> marker;
      if (marker != "=")
	joint = ".";
      else
	in >> value;
      v[joint] = value;
      if (joint != ".")
	in >> joint;
    }
	
  if (!v.empty())
  {
    robot_state::RobotState st = psm->getPlanningScene()->getCurrentState();
    st.setVariablePositions(v);
    moveit_msgs::RobotState msg;
    robot_state::robotStateToRobotStateMsg(st, msg);
    startState = msg;
  }
}


void parseGoalFormat(std::istream& in, planning_scene_monitor::PlanningSceneMonitor* psm,
	       moveit_warehouse::RobotStateStorage* rs, moveit_msgs::Constraints& goalState)
{
  std::string joint;
  std::string marker;
  in >> joint;
  
  double pos;
  double tol_above;
  double tol_below;
  
  moveit_msgs::Constraints msg;
  std::vector<moveit_msgs::JointConstraint> joint_constraints;
  
  while (joint != "." && in.good() && !in.eof())
    {
      in >> marker;
      if (marker != "=")
	joint = ".";
      else{
	in >> pos;
	in >> tol_above;
	in >> tol_below;
	moveit_msgs::JointConstraint joint_constraint;
	joint_constraint.joint_name = joint;
	joint_constraint.position = pos;
	joint_constraint.tolerance_above = tol_above;
	joint_constraint.tolerance_below = tol_below;
	
	joint_constraints.push_back(joint_constraint);
      }
      if (joint != ".")
	in >> joint;
    }
  msg.joint_constraints = joint_constraints;
  goalState = msg;
}

void parseGoalFormatPosition(std::istream& in, planning_scene_monitor::PlanningSceneMonitor* psm,
	       moveit_warehouse::RobotStateStorage* rs, moveit_msgs::Constraints& goalState)
{
  std::string link_eef;
  std::string link_header;
  std::string label;
  std::string marker;
  in >> label;
  in >> link_header;
  in >> label;
  in >> link_eef;

  geometry_msgs::Quaternion orientation;
  std::vector<double> tolerance_angle = {0,0,0};

  geometry_msgs::Point position;
  std::vector<double> tolerance_pos = {0,0,0};

  in >> label;
  in >> marker;
  if(label == "Position" && marker == "=")
    {
      in >> position.x >> position.y >> position.z;
    }
  in >> label;
  in >> marker;
  if(label == "Position_tolerance" && marker == "=")
    {
      in >> tolerance_pos[0] >> tolerance_pos[1] >> tolerance_pos[2];
    }
  in >> label;
  in >> marker;
  if(label == "Orientation" && marker == "=")
    {
      in >> orientation.x >> orientation.y >> orientation.z >> orientation.w;
    }
  in >> label;
  in >> marker;
  if(label == "Orientation_tolerance" && marker == "=")
    {
      in >> tolerance_angle[0] >> tolerance_angle[1] >> tolerance_angle[2];
    }
  in >> label;
  if(label != "."){
    ROS_ERROR("Parsing failed.");
  }
  
  moveit_msgs::Constraints msg;

  moveit_msgs::MotionPlanRequest req;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = link_header;
  pose.pose.position = position;
  pose.pose.orientation = orientation;
  
  msg = kinematic_constraints::constructGoalConstraints(link_eef, pose, tolerance_pos, tolerance_angle);
  goalState = msg;
}

void parseQueriesFormat(std::istream& in, planning_scene_monitor::PlanningSceneMonitor* psm,
                  moveit_warehouse::RobotStateStorage* rs, moveit_warehouse::ConstraintsStorage* cs, moveit_warehouse::PlanningSceneStorage* pss)
{
  std::string scene_name;
  in >> scene_name;
  std::string group_name;

  if(pss->hasPlanningScene(scene_name))
    {
    while (in.good() && !in.eof())
      {
	std::string query_name;
	in >> query_name;
	in >> group_name;
	moveit_msgs::RobotState startState;
	moveit_msgs::Constraints goalState;

	if (in.good() && !in.eof())
	  {
	    std::string start_type;
	    in >> start_type;

	    // Get the start state of the query
	    if(start_type == "START" && in.good() && !in.eof())
	      {
		parseStartFormat(in, psm, rs, startState);
	      }
	    else
	      {
		ROS_ERROR("Unknown query type: '%s'", start_type.c_str());
	      }

	    std::string goal_type;
	    in >> goal_type;
	    
	    // Get the goal of the query as a set of joint_constraints
	    if(goal_type == "GOAL" && in.good() && !in.eof())
	      {
		std::string joint_constraint;
		in >> joint_constraint;
		if(joint_constraint == "joint_constraint")
		  parseGoalFormat(in, psm, rs, goalState);
		if(joint_constraint == "position_constraint")
		  parseGoalFormatPosition(in, psm, rs, goalState);
	      }
	    else
	      {
		ROS_ERROR("Unknown query type: '%s'", goal_type.c_str());
	      }

	    if(goalState.joint_constraints.size() || (goalState.position_constraints.size() && goalState.orientation_constraints.size()))
	      {
		moveit_msgs::MotionPlanRequest planning_query;

		planning_query.group_name = group_name; //TODO
		planning_query.start_state = startState;
		planning_query.goal_constraints = {goalState};
		
		pss->addPlanningQuery(planning_query, scene_name, query_name);
		
		ROS_INFO("Loaded query '%s' to scene '%s'", query_name.c_str(), scene_name.c_str());
	      }
	    else
	      {
		ROS_ERROR("Unknown query: ERROR");
	      }
	  }
      }
    }
  else
    {
      ROS_ERROR("The scene doesn't exist!");
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "import_from_text_to_warehouse", ros::init_options::AnonymousName);

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("queries", boost::program_options::value<std::string>(), "Name of file containing motion planning queries.")
    ("scene", boost::program_options::value<std::string>(), "Name of file containing motion planning scene.")
    ("host", boost::program_options::value<std::string>(),"Host for the DB.")
    ("port", boost::program_options::value<std::size_t>(), "Port for the DB.");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help") || argc == 1)  // show help if no parameters passed
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
  
  ros::NodeHandle nh;
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
  if (!psm.getPlanningScene())
  {
    ROS_ERROR("Unable to initialize PlanningSceneMonitor");
    return 1;
  }

  moveit_warehouse::PlanningSceneStorage pss(conn);
  moveit_warehouse::ConstraintsStorage cs(conn);
  moveit_warehouse::RobotStateStorage rs(conn);

  if (vm.count("scene"))
    {
      std::ifstream fin(vm["scene"].as<std::string>().c_str());
      psm.getPlanningScene()->loadGeometryFromStream(fin);
      fin.close();
      moveit_msgs::PlanningScene psmsg;
      psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
      pss.addPlanningScene(psmsg);
    }
  
  if (vm.count("queries"))
    {
      std::ifstream fin(vm["queries"].as<std::string>().c_str());
      if (fin.good() && !fin.eof())
	{
	  parseQueriesFormat(fin, &psm, &rs, &cs, &pss);
	}
      fin.close();
    }
  return 0;
}
