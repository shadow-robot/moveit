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
/* Author: Raphael Bricout */
/* Author: Shadow Software Team */

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
#include <eigen_conversions/eigen_msg.h>

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
<<<<<<< HEAD
  desc.add_options()
    ("help", "Show help message")
    ("host", boost::program_options::value<std::string>(), "Host for the DB.")
    ("port", boost::program_options::value<std::size_t>(), "Port for the DB.")
    ("queries_format", "Save queries in a good format.");
=======
  desc.add_options()("help", "Show help message")("host", boost::program_options::value<std::string>(), "Host for the "
                                                                                                        "DB.")(
      "port", boost::program_options::value<std::size_t>(), "Port for the DB.")(
      "cartesian", "Save queries in cartesian space (start and end pose of eef)")("scene", "Saves the scene.")(
      "eef", boost::program_options::value<std::string>(),
      "Specify the end effector. Default: last link.")("group_prefix", boost::program_options::value<std::string>(),
                                                       "Specify the group prefix you'd like to plan with.")(
      "output_directory", boost::program_options::value<std::string>(), "Directory to save files");
>>>>>>> trying_to_use_once_for_all_the_50random_queries_set

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  std::string cartesian_prefix = "";
  if (vm.count("cartesian"))
  {
    cartesian_prefix = "cartesian_";
  }

  std::string link_eef = "";
  if (vm.count("eef"))
  {
    link_eef = vm["eef"].as<std::string>();
  }
  else
  {
    ROS_INFO("If you want to export the queries in cartesian space, you can specify the eef.");
  }

  std::string output_dir = "";
  if (vm.count("output_directory"))
  {
    output_dir = vm["output_directory"].as<std::string>();
    if (output_dir.size() && output_dir[output_dir.size() - 1] != '/')
      output_dir.append("/");

    // Ensure directories exist
    boost::filesystem::create_directories(output_dir);
  }
  else
  {
    ROS_INFO("Specify output_directory to save the files in a specific directory");
  }

  std::string group_prefix = "";
  if (vm.count("group_prefix"))
  {
    group_prefix = vm["group_prefix"].as<std::string>();
  }
  else
  {
    ROS_INFO(
        "If you have a problem with a composite robot, try to use the group_prefix option to specify the prefix of the "
        "group you want to save the queries");
  }

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
<<<<<<< HEAD
  
=======

  std::string scene_filename;
  std::string queries_filename;
>>>>>>> trying_to_use_once_for_all_the_50random_queries_set
  for (std::size_t i = 0; i < scene_names.size(); ++i)
  {
    moveit_warehouse::PlanningSceneWithMetadata pswm;
    if (pss.getPlanningScene(pswm, scene_names[i]))
    {
      psm.getPlanningScene()->setPlanningSceneMsg(static_cast<const moveit_msgs::PlanningScene&>(*pswm));
      if (vm.count("scene"))
      {
        scene_filename = output_dir + (scene_names[i] + ".scene").c_str();
        ROS_INFO("Saving scene '%s'", scene_filename.c_str());

        std::ofstream fout(scene_filename);
        psm.getPlanningScene()->saveGeometryToStream(fout);
        fout.close();
      }
      else
      {
        std::vector<std::string> robotStateNames;
        robot_model::RobotModelConstPtr km = psm.getRobotModel();
        // Get start states for scene
        std::stringstream rsregex;
        rsregex << ".*" << scene_names[i] << ".*";
        rss.getKnownRobotStates(rsregex.str(), robotStateNames);

        // Get goal constraints for scene
        std::vector<std::string> constraintNames;

<<<<<<< HEAD
      std::stringstream csregex;
      csregex << ".*" << scene_names[i] << ".*";
      cs.getKnownConstraints(csregex.str(), constraintNames);
      
      std::ofstream qfout((scene_names[i] + ".queries").c_str());
      qfout << scene_names[i] << std::endl;
      
      //Save queries with a good format to be able to load them
      if(vm.count("queries_format")){

	std::vector<std::string> query_names;
      
	std::stringstream pssregex;
	pssregex << ".*";
	pss.getPlanningQueriesNames(pssregex.str(), query_names, scene_names[i]);
	
	//std::ofstream qfout((scene_names[i] + ".queries").c_str());
	//qfout << scene_names[i] << std::endl;
	
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
	    qfout << "START" << std::endl;
	    
	    for (std::size_t p = 0; p < jointState.position.size(); ++p){
	      qfout << jointState.name[p] << " = ";
	      qfout << jointState.position[p] << std::endl;
	    }
	    qfout << "." << std::endl;
	    
	    std::vector<moveit_msgs::Constraints> query_goal_constraints = plan_request.goal_constraints;
	    
	    if (query_goal_constraints.size() == 1) // a regular RViz query should have 1 constraint
	      {
		moveit_msgs::Constraints query_goal = query_goal_constraints[0];
		qfout << "GOAL" << std::endl;
		qfout << "joint_constraint" << std::endl;
		
		if(query_goal.joint_constraints.size()){
		  std::vector<moveit_msgs::JointConstraint> joint_constraints = query_goal.joint_constraints;
		  for(auto iter = joint_constraints.begin(); iter != joint_constraints.end(); iter++){
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
      
      else if (!(robotStateNames.empty() && constraintNames.empty()))
	{
	  //std::ofstream qfout((scene_names[i] + ".queries").c_str());
	  //qfout << scene_names[i] << std::endl;
	  if (robotStateNames.size())
	    {
	      qfout << "start" << std::endl;
	      qfout << robotStateNames.size() << std::endl;
	      for (std::size_t k = 0; k < robotStateNames.size(); ++k)
		{
		  ROS_INFO("Saving start state %s for scene %s", robotStateNames[k].c_str(), scene_names[i].c_str());
		  qfout << robotStateNames[k] << std::endl;
		  moveit_warehouse::RobotStateWithMetadata robotState;
		  rss.getRobotState(robotState, robotStateNames[k]);
		  robot_state::RobotState ks(km);
		  robot_state::robotStateMsgToRobotState(*robotState, ks, false);
		  ks.printStateInfo(qfout);
		  qfout << "." << std::endl;
		}
	    }
	  
	  if (constraintNames.size())
	    {
	      qfout << "goal" << std::endl;
	      qfout << constraintNames.size() << std::endl;
	      for (std::size_t k = 0; k < constraintNames.size(); ++k)
		{
		  ROS_INFO("Saving goal %s for scene %s", constraintNames[k].c_str(), scene_names[i].c_str());
		  qfout << "link_constraint" << std::endl;
		  qfout << constraintNames[k] << std::endl;
		  moveit_warehouse::ConstraintsWithMetadata constraints;
		  cs.getConstraints(constraints, constraintNames[k]);
		  
		  LinkConstraintMap lcmap;
		  collectLinkConstraints(*constraints, lcmap);
            for (LinkConstraintMap::iterator iter = lcmap.begin(); iter != lcmap.end(); iter++)
	      {
		std::string link_name = iter->first;
		LinkConstraintPair lcp = iter->second;
		qfout << link_name << std::endl;
		qfout << "xyz " << lcp.first.x << " " << lcp.first.y << " " << lcp.first.z << std::endl;
		Eigen::Quaterniond orientation(lcp.second.w, lcp.second.x, lcp.second.y, lcp.second.z);
		Eigen::Vector3d rpy = orientation.matrix().eulerAngles(0, 1, 2);
		qfout << "rpy " << rpy[0] << " " << rpy[1] << " " << rpy[2] << std::endl;
	      }
            qfout << "." << std::endl;
		}
	    }
	  qfout.close();
	}
=======
        std::stringstream csregex;
        csregex << ".*" << scene_names[i] << ".*";
        cs.getKnownConstraints(csregex.str(), constraintNames);

        std::vector<std::string> query_names;
        std::stringstream pssregex;
        pssregex << ".*";
        pss.getPlanningQueriesNames(pssregex.str(), query_names, scene_names[i]);

        queries_filename = output_dir + (cartesian_prefix + scene_names[i] + ".queries").c_str();
        std::ofstream qfout(queries_filename);
        qfout << scene_names[i] << std::endl;

        for (std::size_t k = 0; k < query_names.size(); ++k)
        {
          ROS_INFO("Saving query '%s' from scene '%s'", query_names[k].c_str(), scene_names[i].c_str());
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
          std::vector<moveit_msgs::Constraints> query_goal_constraints = plan_request.goal_constraints;
          if (query_goal_constraints.size() == 1)
          {
            moveit_msgs::Constraints query_goal = query_goal_constraints[0];
            moveit_msgs::RobotState startState = plan_request.start_state;
            sensor_msgs::JointState jointState = startState.joint_state;

            std::vector<moveit_msgs::JointConstraint> joint_constraints = query_goal.joint_constraints;
            std::vector<moveit_msgs::PositionConstraint> position_constraints = query_goal.position_constraints;
            std::vector<moveit_msgs::OrientationConstraint> orientation_constraints =
                query_goal.orientation_constraints;

            // Save queries defined in joint space
            if (joint_constraints.size() != 0)
            {
              if (vm.count("cartesian"))
              {
                query_names[k] = "cartesian_" + query_names[k];
              }
              qfout << query_names[k] << std::endl;

              // Save the start State
              qfout << "START" << std::endl;
              for (std::size_t p = 0; p < jointState.position.size(); ++p)
              {
                if (jointState.name[p].compare(0, group_prefix.length(), group_prefix) == 0)
                {
                  qfout << jointState.name[p] << " = ";
                  qfout << jointState.position[p] << std::endl;
                }
              }
              qfout << "." << std::endl;

              // Save the goal state
              qfout << "GOAL" << std::endl;

              // save queries using end-effector position
              if (vm.count("cartesian"))
              {
                qfout << "position_constraint" << std::endl;

                moveit::core::RobotState goal_state(km);
                for (int i = 0; i < joint_constraints.size(); ++i)
                {
                  goal_state.setJointPositions(joint_constraints[i].joint_name, { joint_constraints[i].position });
                }

                const moveit::core::JointModel* joint_eef =
                    goal_state.getJointModel(joint_constraints[joint_constraints.size() - 1].joint_name);

                if (link_eef == "")
                {
                  link_eef = joint_eef->getChildLinkModel()->getName();
                }
                const Eigen::Affine3d& link_pose = goal_state.getGlobalLinkTransform(link_eef);

                geometry_msgs::Transform transform;
                tf::transformEigenToMsg(link_pose, transform);
                qfout << "End_effector = " << link_eef << std::endl;
                qfout << "Position =";
                qfout << " " << transform.translation.x;
                qfout << " " << transform.translation.y;
                qfout << " " << transform.translation.z << std::endl;
                qfout << "Orientation =";
                qfout << " " << transform.rotation.x;
                qfout << " " << transform.rotation.y;
                qfout << " " << transform.rotation.z;
                qfout << " " << transform.rotation.w << std::endl;
                qfout << "." << std::endl;
              }
              // save queries using joint constraints
              else
              {
                qfout << "joint_constraint" << std::endl;
                std::vector<moveit_msgs::JointConstraint> joint_constraints = query_goal.joint_constraints;
                if (joint_constraints.size() != 0)
                {
                  for (auto iter = joint_constraints.begin(); iter != joint_constraints.end(); iter++)
                  {
                    qfout << iter->joint_name << " = ";
                    qfout << iter->position << std::endl;
                  }
                }
                qfout << "." << std::endl;
              }
            }
          }
        }

        // Saving link constaints
        if (!(robotStateNames.empty() && constraintNames.empty()))
        {
          if (robotStateNames.size())
          {
            qfout << "start" << std::endl;
            qfout << robotStateNames.size() << std::endl;
            for (std::size_t k = 0; k < robotStateNames.size(); ++k)
            {
              ROS_INFO("Saving start state %s for scene %s", robotStateNames[k].c_str(), scene_names[i].c_str());
              qfout << robotStateNames[k] << std::endl;
              moveit_warehouse::RobotStateWithMetadata robotState;
              rss.getRobotState(robotState, robotStateNames[k]);
              robot_state::RobotState ks(km);
              robot_state::robotStateMsgToRobotState(*robotState, ks, false);
              ks.printStateInfo(qfout);
              qfout << "." << std::endl;
            }
          }

          if (constraintNames.size())
          {
            qfout << "goal" << std::endl;
            qfout << constraintNames.size() << std::endl;
            for (std::size_t k = 0; k < constraintNames.size(); ++k)
            {
              ROS_INFO("Saving goal %s for scene %s", constraintNames[k].c_str(), scene_names[i].c_str());
              qfout << "link_constraint" << std::endl;
              qfout << constraintNames[k] << std::endl;
              moveit_warehouse::ConstraintsWithMetadata constraints;
              cs.getConstraints(constraints, constraintNames[k]);

              LinkConstraintMap lcmap;
              collectLinkConstraints(*constraints, lcmap);
              for (LinkConstraintMap::iterator iter = lcmap.begin(); iter != lcmap.end(); iter++)
              {
                std::string link_name = iter->first;
                LinkConstraintPair lcp = iter->second;
                qfout << link_name << std::endl;
                qfout << "xyz " << lcp.first.x << " " << lcp.first.y << " " << lcp.first.z << std::endl;
                Eigen::Quaterniond orientation(lcp.second.w, lcp.second.x, lcp.second.y, lcp.second.z);
                Eigen::Vector3d rpy = orientation.matrix().eulerAngles(0, 1, 2);
                qfout << "rpy " << rpy[0] << " " << rpy[1] << " " << rpy[2] << std::endl;
              }
              qfout << "." << std::endl;
            }
          }
        }
        qfout.close();
      }
>>>>>>> trying_to_use_once_for_all_the_50random_queries_set
    }
  }
  
  ROS_INFO("Done.");
  
  return 0;
}
