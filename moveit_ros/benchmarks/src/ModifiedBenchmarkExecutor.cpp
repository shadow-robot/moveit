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

#include <moveit/benchmarks/ModifiedBenchmarkExecutor.h>
#include <moveit/utils/lexical_casts.h>
#include <moveit/version.h>
#include <eigen_conversions/eigen_msg.h> // Abstract transformations, such as rotations (represented by angle and axis or by a quaternion), translations, scalings

// For getting the moves displayed in RViz trial1
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h> //not sufficient for publishTrajectoryPath, maybe to REMOVE TODO
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <boost/regex.hpp> // To search for letters or words
#include <boost/progress.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp> // seconds since January 1st of 1970
#include <unistd.h>

#include <math.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // for the additional metrics
#include <tf2/LinearMath/Quaternion.h>

#include <list> // for the map of planners -- associated parameters

#include <iostream> // for writing the unique .log

/* strangely the catkin builder was already not complaining before I even added these so let's comment them
//to read backwards a file from its end:
// https://stackoverflow.com/questions/14834267/reading-a-text-file-backwards-in-c 
#include <limits.h>
#include <string.h>
#include <stdio.h>
//to truncate:
// https://stackoverflow.com/questions/873454/how-to-truncate-a-file-in-c/873653#873653 
//#include <unistd.h> 
#include <sys/types.h> */

#include <chrono> // for debug purpose (should prevent the sequential execution to do the following actions)
#include <thread>


using namespace moveit_ros_benchmarks;

static std::string getHostname()
{
  static const int BUF_SIZE = 1024;
  char buffer[BUF_SIZE];
  int err = gethostname(buffer, sizeof(buffer));
  if (err != 0)
    return std::string();
  else
  {
    buffer[BUF_SIZE - 1] = '\0';
    return std::string(buffer);
  }
}

static const std::string BASE_LINK = "/world"; //try "world" "base_frame" and with prefix "/..."
//static const std::string MARKER_TOPIC = "/rviz_moveit_motion_planning_displays/robot_interaction_interactive_marker_topic/update_full"; //"/moveit_visual_markers";
//static const std::string MARKER_TOPIC = "/moveit_visual_markers";
//static const std::string MARKER_TOPIC = "/execute_trajectory/action_topics";
static const std::string MARKER_TOPIC = "/moveit_visual_markers";

//http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html
const std::string ROBOT_DESCRIPTION = "robot_description";
// The :move_group_interface:`MoveGroup` class can be easily
// setup using just the name of the planning group you would like to control and plan for.
const std::string PLANNING_GROUP = "right_arm"; //or right_arm_and_manipulator or right_arm_and_hand

const std::string DISPLAY_PLANNED_PATH_PARALLEL = "/display_planned_path_parallel"; //trick to get a channel to publish on another trajPath,
//if the default one (/move_group/display_planned_path) is already occupied and one wants to launch several trajs simultaneously

bool JOINT_ANGLE_RESTRICTED;
bool ADAPT_QUERIES;

ModifiedBenchmarkExecutor::ModifiedBenchmarkExecutor(const std::string& robot_description_param)
{
  pss_ = NULL;
  psws_ = NULL;
  rs_ = NULL;
  cs_ = NULL;
  tcs_ = NULL;
  psm_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_description_param)); //pointer
  
	planning_scene_ = psm_->getPlanningScene();
	
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(BASE_LINK, MARKER_TOPIC));
  visual_tools2_.reset(new moveit_visual_tools::MoveItVisualTools(BASE_LINK, MARKER_TOPIC)); //additional layer to display the goal
  //as publishRobotState seems to erase the previous one : https://docs.ros.org/api/moveit_visual_tools/html/moveit__visual__tools_8cpp_source.html#l01382 (only one shared robot state that is updated each time)
  
  visual_tools_->loadPlanningSceneMonitor();
	
  visual_tools_->loadMarkerPub(false,true); //for the traj lines
    
  visual_tools_->loadSharedRobotState();
  visual_tools2_->loadSharedRobotState(); //additional layer to display the goal
  
  visual_tools_->loadRobotStatePub("/display_start_configuration");
  visual_tools2_->loadRobotStatePub("/display_goal_configuration"); //additional layer to pop the goal conf simultaneously with the start one
  
  visual_tools2_->loadTrajectoryPub(DISPLAY_PLANNED_PATH_PARALLEL);//trick to get a channel to publish on another trajPath,
	//if the default one (/move_group/display_planned_path) is already occupied and one wants to launch several trajs simultaneously
  
	visual_tools_->deleteAllMarkers();
  visual_tools_->removeAllCollisionObjects();

  // Initialize the class loader for planner plugins
  try
  {
    planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  
  //an example that may be useful:
  // https://github.com/davetcoleman/moveit_hrp2/blob/master/hrp2jsknt_moveit_demos/src/hrp2_demos.cpp#L160
  
  // Load the robot model
	robot_model_loader::RobotModelLoader robot_model_loader_;
	robot_model::RobotModelPtr robot_model_ = robot_model_loader_.getModel(); // Get a shared pointer to the robot
  
  robot_state::RobotStatePtr shared_robot_state_;
  shared_robot_state_.reset(new robot_state::RobotState(robot_model_)); // TODO: load this robot state from planning_scene instead
	shared_robot_state_->setToDefaultValues();
}

ModifiedBenchmarkExecutor::~ModifiedBenchmarkExecutor()
{
  if (pss_)
    delete pss_;
  if (psws_)
    delete psws_;
  if (rs_)
    delete rs_;
  if (cs_)
    delete cs_;
  if (tcs_)
    delete tcs_;
  //delete psm_; //error expecting a pointer otherwise
}

void ModifiedBenchmarkExecutor::initialize(const std::vector<std::string>& plugin_classes)
{
  planner_interfaces_.clear();
  // Load the planning plugins
  const std::vector<std::string>& classes = planner_plugin_loader_->getDeclaredClasses();

  for (std::size_t i = 0; i < plugin_classes.size(); ++i)
  {
    std::vector<std::string>::const_iterator it = std::find(classes.begin(), classes.end(), plugin_classes[i]);
    if (it == classes.end())
    {
      ROS_ERROR("Failed to find plugin_class %s", plugin_classes[i].c_str());
      return;
    }

    try
    {
      planning_interface::PlannerManagerPtr p = planner_plugin_loader_->createUniqueInstance(plugin_classes[i]);
      p->initialize(planning_scene_->getRobotModel(), "");

      p->getPlannerConfigurations();
      planner_interfaces_[plugin_classes[i]] = p;
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Exception while loading planner '" << plugin_classes[i] << "': " << ex.what());
    }
  }

  // error check
  if (planner_interfaces_.empty())
    ROS_ERROR("No planning plugins have been loaded. Nothing to do for the benchmarking service.");
  else
  {
    std::stringstream ss;
    for (std::map<std::string, planning_interface::PlannerManagerPtr>::const_iterator it = planner_interfaces_.begin();
         it != planner_interfaces_.end(); ++it)
      ss << it->first << " ";
    ROS_INFO("Available planner instances: %s", ss.str().c_str());
  }
}

void ModifiedBenchmarkExecutor::clear()
{
  if (pss_)
  {
    delete pss_;
    pss_ = NULL;
  }
  if (psws_)
  {
    delete psws_;
    psws_ = NULL;
  }
  if (rs_)
  {
    delete rs_;
    rs_ = NULL;
  }
  if (cs_)
  {
    delete cs_;
    cs_ = NULL;
  }
  if (tcs_)
  {
    delete tcs_;
    tcs_ = NULL;
  }

  benchmark_data_.clear();
  pre_event_fns_.clear();
  post_event_fns_.clear();
  planner_start_fns_.clear();
  planner_completion_fns_.clear();
  query_start_fns_.clear();
  query_end_fns_.clear();
}

void ModifiedBenchmarkExecutor::addPreRunEvent(PreRunEventFunction func)
{
  pre_event_fns_.push_back(func);
}

void ModifiedBenchmarkExecutor::addPostRunEvent(PostRunEventFunction func)
{
  post_event_fns_.push_back(func);
}

void ModifiedBenchmarkExecutor::addPlannerStartEvent(PlannerStartEventFunction func)
{
  planner_start_fns_.push_back(func);
}

void ModifiedBenchmarkExecutor::addPlannerCompletionEvent(PlannerCompletionEventFunction func)
{
  planner_completion_fns_.push_back(func);
}

void ModifiedBenchmarkExecutor::addQueryStartEvent(QueryStartEventFunction func)
{
  query_start_fns_.push_back(func);
}

void ModifiedBenchmarkExecutor::addQueryCompletionEvent(QueryCompletionEventFunction func)
{
  query_end_fns_.push_back(func);
}

bool ModifiedBenchmarkExecutor::runBenchmarks(const ModifiedBenchmarkOptions& opts)
{

	//debug: TODO to be removed
	unsigned int ever_wasnt_first_solved_but_kept_restarts = 0;

  if (planner_interfaces_.size() == 0)
  {
    ROS_ERROR("No planning interfaces configured.  Did you call ModifiedBenchmarkExecutor::initialize?");
    return false;
  }

  std::vector<BenchmarkRequest> queries;
  moveit_msgs::PlanningScene scene_msg;
  
  std::vector<std::vector<double>> jointAnglesMinMax;
  std::list<std::string> limitedInAngleActuatedJointsNames;
  if (initializeBenchmarks(opts, scene_msg, queries, jointAnglesMinMax, limitedInAngleActuatedJointsNames))
  {
    if (!queriesAndPlannersCompatible(queries, opts.getPlannerConfigurations()))
      return false;

    for (std::size_t i = 0; i < queries.size(); ++i)
    {
      ROS_WARN("--------------------------------");
      ROS_WARN("--------------------------------");
      ROS_WARN("QUERY '%s' (%lu of %lu)", queries[i].name.c_str(), i + 1, queries.size()); // (Note that queries are benchmarked in disorder)
      ROS_WARN("--------------------------------");
      ROS_WARN("--------------------------------");
      
      // Configure planning scene
      if (scene_msg.robot_model_name != planning_scene_->getRobotModel()->getName())
      {
        // Clear all geometry from the scene
        planning_scene_->getWorldNonConst()->clearObjects();
        planning_scene_->getCurrentStateNonConst().clearAttachedBodies();
        planning_scene_->getCurrentStateNonConst().setToDefaultValues();

        planning_scene_->processPlanningSceneWorldMsg(scene_msg.world);
      }
      else
        planning_scene_->usePlanningSceneMsg(scene_msg); //Apply changes to this planning scene, e.g the robot state is overwritten

      // Calling query start events // QUITE DONT UNDERSTAND  for what index j stands.. nb runs??
      /*ROS_ERROR("[EXPLORE] query_start_fns_.size() = %lu", query_start_fns_.size());//=0 always currently*/
      // Doesn't matter since it's never filled
      for (std::size_t j = 0; j < query_start_fns_.size(); ++j)
        query_start_fns_[j](queries[i].request, planning_scene_);
      
      /*ROS_ERROR("[EXPLORE] queries[%lu].request =", i);
    	std::vector<double> tmp3 = queries[i].request.start_state.joint_state.position;
    	for (int j=0; j<tmp3.size(); ++j)
    		ROS_ERROR("[EXPLORE] %f", tmp3[j]);*/
      
      
			const bool GENERATE_LOGS = false; //TODO default true but currently I'm working on the animation part.
      //This allows/prevent from writing both the bunch of log files initially generated by my ancestors,
      //+ my unique huge file containing all the queries
      //And hence massively speeds up the benchmark (probably the algorithm itself as well, in a least scale)
      const bool GENERATE_ANIMATION_RVIZ = true;
      
      ros::WallTime start_time = ros::WallTime::now();
      
      unsigned int no_first_kept_restart = 0; //debug TODO to be removed
      runBenchmark(queries[i].request, 
		   						 queries[i].name, 
		    					 options_.getPlannerConfigurations(), 
		   						 options_.getNumRuns(), 
		   						 options_.getMetricChoice(), 
		   						 options_.getSceneName(),
		   						 GENERATE_LOGS,
		   						 GENERATE_ANIMATION_RVIZ,
		   						 jointAnglesMinMax,
		   						 limitedInAngleActuatedJointsNames,
		   						 no_first_kept_restart); //debug
      double duration = (ros::WallTime::now() - start_time).toSec();
      
      ROS_INFO("Benchmark took '%f' seconds.", duration);
      
      //debug:
      ever_wasnt_first_solved_but_kept_restarts = ever_wasnt_first_solved_but_kept_restarts + no_first_kept_restart;

      for (std::size_t j = 0; j < query_end_fns_.size(); ++j)
        query_end_fns_[j](queries[i].request, planning_scene_);
			
			if (GENERATE_LOGS)
      	writeOutput(queries[i], boost::posix_time::to_iso_extended_string(start_time.toBoost()), duration);
    }
		
		//debug:
		ROS_ERROR("[DEBUG] ever_wasnt_first_solved_but_kept_restarts = %u", ever_wasnt_first_solved_but_kept_restarts);
		//conclusion : the dumb nonoptimal fast planners, when not finding any solution, do use the whole countdown we allow them,
		//they are content with not using the whole countdown only when they solve the problem!
		//So never expect to plot a tweaked movement alone on the scene without an initial succeeding first shot to compare with!
		
    return true; //All has gone well //Be careful here well =1, different from return 0 (issues)!!
  }
  return false;
}

bool ModifiedBenchmarkExecutor::queriesAndPlannersCompatible(const std::vector<BenchmarkRequest>& requests,
                                                     const std::map<std::string, std::vector<std::string>>& planners)
{
  // Make sure that the planner interfaces can service the desired queries
  for (std::map<std::string, planning_interface::PlannerManagerPtr>::const_iterator it = planner_interfaces_.begin();
       it != planner_interfaces_.end(); ++it)
  {
    for (std::size_t i = 0; i < requests.size(); ++i)
    {
      if (!it->second->canServiceRequest(requests[i].request))
      {
        ROS_ERROR("Interface '%s' cannot service the benchmark request '%s'", it->first.c_str(),
                  requests[i].name.c_str());
        return false;
      }
    }
  }

  return true;
}

// In order to work with limited joint angles:
// https://stackoverflow.com/questions/4633177/c-how-to-wrap-a-float-to-the-interval-pi-pi
const double     _PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
const double _TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;
// Floating-point modulo
// The result (the remainder) has same sign as the divisor.
// Similar to matlab's mod(); Not similar to fmod() -   Mod(-3,4)= 1   fmod(-3,4)= -3
template<typename T>
T ModifiedBenchmarkExecutor::Mod(T x, T y)
{
  static_assert(!std::numeric_limits<T>::is_exact , "Mod: floating-point type expected");
  if (0. == y)
    return x;

  double m= x - y * floor(x/y);

  // handle boundary cases resulted from floating-point cut off:

  if (y > 0)              // modulo range: [0..y)
  {
    if (m>=y)           // Mod(-1e-16             , 360.    ): m= 360.
      return 0;

    if (m<0 )
    {
      if (y+m == y)
        return 0  ; // just in case...
      else
        return y+m; // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14
    }
  }
  else                    // modulo range: (y..0]
  {
    if (m<=y)           // Mod(1e-16              , -360.   ): m= -360.
      return 0;

    if (m>0 )
    {
      if (y+m == y)
        return 0  ; // just in case...
      else
        return y+m; // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14
    }
  }
  return m;
}
// wrap [rad] angle to [-PI..PI)
inline double ModifiedBenchmarkExecutor::AnyRadToMpiPiExc(double angle)
{
	return Mod(angle + _PI, _TWO_PI) - _PI;
}
// wrap [rad] angle to [0..TWO_PI)
inline double ModifiedBenchmarkExecutor::AnyRadToZeroTwoPiExc(double angle)
{
	return Mod(angle, _TWO_PI);
}
// wrap [deg] angle to [-180..180)
inline double ModifiedBenchmarkExecutor::AnyDegToM180180(double angle)
{
	return Mod(angle + 180., 360.) - 180.;
}
// wrap [deg] angle to [0..360)
inline double ModifiedBenchmarkExecutor::AnyDegTo0360(double angle)
{
	return Mod(angle ,360.);
}

bool ModifiedBenchmarkExecutor::getActuatedJointAngleLimits(
			std::list<std::string>& limitedInAngleActuatedJointsNames,
			std::vector<std::vector<double>>& jointAnglesMinMax)
{
	//////////////////////////////////////////////////////////////////
  // Get the joint angle limits for laterly bringing back the queries into -pi pi IF the joint angles are restricted by the .yaml
  //RETURNS restricted or not (bool)
  //////////////////////////////////////////////////////////////////
  // Here I ASSUME every joint of the move_group are angle limited
  const std::string pathJointLimits =
  "/robot_description_planning/joint_limits";
  XmlRpc::XmlRpcValue jointLimits_Xml = getServerParameters(pathJointLimits);
  //ROS_ERROR("[ENSURE] /robot_description_planning/joint_limits.size() = %d", jointLimits_Xml.size());//=15
  //I skip steps here as one should properly retrieve the joints belonging to the move_group (TODO):
  std::list<std::string> move_group_joints_names{"ra_shoulder_pan_joint","ra_shoulder_lift_joint","ra_elbow_joint",
 "ra_wrist_1_joint","ra_wrist_2_joint","ra_wrist_3_joint"};//ordered
 	//such that it goes from base to wrist
 	for(auto const& ilist: move_group_joints_names)
  {
  	ROS_ERROR("[ENSURE] ilist = %s", ilist.c_str());
  	limitedInAngleActuatedJointsNames.push_back(ilist);
  	XmlRpc::XmlRpcValue tmpJointMultiConstraints_Xml = jointLimits_Xml[ilist.c_str()]; //TODO add robustness (if ilist isn't a key of the Xml map, tell the user to double check the joint_limits.yaml!)
  	ROS_ERROR("[DEBUG] min %f, max %f", (double)tmpJointMultiConstraints_Xml["min_position"], (double)tmpJointMultiConstraints_Xml["max_position"]);
  	std::vector<double> minMax;
  	minMax.push_back((double)tmpJointMultiConstraints_Xml["min_position"]);
  	minMax.push_back((double)tmpJointMultiConstraints_Xml["max_position"]);
  	jointAnglesMinMax.push_back(minMax); //ordered s.t it follows the geometrical order of the joint names (base to wrist) and not the alphabetical one of the rosparam server! :) (useful for displaying in RViz later in the same order than the queries)
  }
  /*//Dunno why this check doesnt work:
  ROS_ERROR("[ENSURE] Verify joint_angle_lims: it");
  for (auto it = limitedInAngleActuatedJointsNames.begin();
			 it != limitedInAngleActuatedJointsNames.end(); ++it)
 	{
 		int i = std::distance(limitedInAngleActuatedJointsNames.begin(), it); //convert it to touble
 		ROS_ERROR("[DEBUG] i = %d", i);
 		ROS_ERROR("[ENSURE] %s: min %f, max %f rad", it->c_str(),
							jointAnglesMinMax[i][1], jointAnglesMinMax[i][2]);
	}*/
	//But this one does so it's ok I trust it:
	ROS_ERROR("[ENSURE] Verify joint_angle_lims:");
	for (int i = 0; i < jointAnglesMinMax.size(); i++)
    for (int j = 0; j < jointAnglesMinMax[i].size(); j++)
      ROS_ERROR("[DEBUG] i = %d : %f", i, jointAnglesMinMax[i][j]);
  // Test whether joint angle are unrestricted or not
  double maximum = 0.;
	for(int i=0; i<jointAnglesMinMax.size(); ++i)
		for(int j=0; j<jointAnglesMinMax[i].size(); ++j)
		  maximum = std::max(std::fabs(jointAnglesMinMax[i][j]), maximum);
  ROS_ERROR("[DEBUG] maximum = %f", maximum);
  if (maximum < _PI)
  	return true;
	else
		return false;
} //TODO generalize to it to be able to get velocity_limit and acceleration_limits as well, but I'm into a rush so no time

bool ModifiedBenchmarkExecutor::initializeBenchmarks(const ModifiedBenchmarkOptions& opts,
																										 moveit_msgs::PlanningScene& scene_msg,
																										 std::vector<BenchmarkRequest>& requests,
																										 std::vector<std::vector<double>>& jointAnglesMinMax,  std::list<std::string>& limitedInAngleActuatedJointsNames)
{
  if (!plannerConfigurationsExist(opts.getPlannerConfigurations(), opts.getGroupName()))
    return false;

  try
  {
    warehouse_ros::DatabaseConnection::Ptr conn = dbloader.loadDatabase();
    conn->setParams(opts.getHostName(), opts.getPort(), 20);
    if (conn->connect())
    {
      pss_ = new moveit_warehouse::PlanningSceneStorage(conn);
      psws_ = new moveit_warehouse::PlanningSceneWorldStorage(conn);
      rs_ = new moveit_warehouse::RobotStateStorage(conn);
      cs_ = new moveit_warehouse::ConstraintsStorage(conn);
      tcs_ = new moveit_warehouse::TrajectoryConstraintsStorage(conn);
    }
    else
    {
      ROS_ERROR("Failed to connect to DB");
      return false;
    }
  }
  catch (std::exception& e)
  {
    ROS_ERROR("Failed to initialize benchmark server: '%s'", e.what());
    return false;
  }

  std::vector<StartState> start_states;
  std::vector<PathConstraints> path_constraints;
  std::vector<PathConstraints> goal_constraints;
  std::vector<TrajectoryConstraints> traj_constraints;
  std::vector<BenchmarkRequest> queries;

  bool ok = loadPlanningScene(opts.getSceneName(), scene_msg) && loadStates(opts.getStartStateRegex(), start_states) &&
            loadPathConstraints(opts.getGoalConstraintRegex(), goal_constraints) &&
            loadPathConstraints(opts.getPathConstraintRegex(), path_constraints) &&
            loadTrajectoryConstraints(opts.getTrajectoryConstraintRegex(), traj_constraints) &&
            loadQueries(opts.getQueryRegex(), opts.getSceneName(), queries);

  if (!ok)
  {
    ROS_ERROR("Failed to load benchmark stuff");
    return false;
  }

  ROS_WARN("Benchmark loaded %lu starts, %lu goals, %lu path constraints, %lu trajectory constraints, and %lu queries",
           start_states.size(), goal_constraints.size(), path_constraints.size(), traj_constraints.size(),
           queries.size()); //repass it as initially a ROS_INFO TODO

  moveit_msgs::WorkspaceParameters workspace_parameters = opts.getWorkspaceParameters();
  // Make sure that workspace_parameters are set
  if (workspace_parameters.min_corner.x == workspace_parameters.max_corner.x &&
      workspace_parameters.min_corner.x == 0.0 &&
      workspace_parameters.min_corner.y == workspace_parameters.max_corner.y &&
      workspace_parameters.min_corner.y == 0.0 &&
      workspace_parameters.min_corner.z == workspace_parameters.max_corner.z &&
      workspace_parameters.min_corner.z == 0.0)
  {
    workspace_parameters.min_corner.x = workspace_parameters.min_corner.y = workspace_parameters.min_corner.z = -5.0;

    workspace_parameters.max_corner.x = workspace_parameters.max_corner.y = workspace_parameters.max_corner.z = 5.0;
  }

  std::vector<double> goal_offset;
  opts.getGoalOffsets(goal_offset);

  // Create the combinations of BenchmarkRequests
  
  // 1) Create requests for combinations of start states,
  //    goal constraints, and path constraints
  
  ROS_ERROR("[EXPLORE] goal_constraints.size() = %lu", goal_constraints.size());//=0
  for (std::size_t i = 0; i < goal_constraints.size(); ++i)
  {
    // Common benchmark request properties
    BenchmarkRequest brequest;
    brequest.name = goal_constraints[i].name;
    brequest.request.workspace_parameters = workspace_parameters;
    brequest.request.goal_constraints = goal_constraints[i].constraints;
    brequest.request.group_name = opts.getGroupName();
    brequest.request.allowed_planning_time = opts.getTimeout();
    brequest.request.num_planning_attempts = 1;

    if (brequest.request.goal_constraints.size() == 1 &&
        brequest.request.goal_constraints[0].position_constraints.size() == 1 &&
        brequest.request.goal_constraints[0].orientation_constraints.size() == 1 &&
        brequest.request.goal_constraints[0].visibility_constraints.size() == 0 &&
        brequest.request.goal_constraints[0].joint_constraints.size() == 0)
      shiftConstraintsByOffset(brequest.request.goal_constraints[0], goal_offset);

    std::vector<BenchmarkRequest> request_combos;
    createRequestCombinations(brequest, start_states, path_constraints, request_combos);
    ROS_ERROR("[EXPLORE] 1) request_combos.size() = %lu", request_combos.size());
    requests.insert(requests.end(), request_combos.begin(), request_combos.end());
  }
  
	JOINT_ANGLE_RESTRICTED = getActuatedJointAngleLimits(
	limitedInAngleActuatedJointsNames, jointAnglesMinMax);
  
  // 2) Existing queries are treated like goal constraints.
  //    Create all combos of query, start states, and path constraints
  ROS_ERROR("[EXPLORE] queries.size() = %lu", queries.size());
  for (std::size_t i = 0; i < queries.size(); ++i)
  {
  	ROS_ERROR("[EXPLORE] query i = %lu", i);
    // Common benchmark request properties
    BenchmarkRequest brequest;
    brequest.name = queries[i].name;
    brequest.request = queries[i].request;
    
    ROS_ERROR("[DEBUG] JOINT_ANGLE_RESTRICTED = %d", JOINT_ANGLE_RESTRICTED);
    if (JOINT_ANGLE_RESTRICTED)
    	AdaptJointSpaceQueryMpiPi(brequest.request);
    
    brequest.request.group_name = opts.getGroupName();
    brequest.request.allowed_planning_time = opts.getTimeout();
    brequest.request.num_planning_attempts = 1;

    // Make sure that workspace_parameters are set
    if (brequest.request.workspace_parameters.min_corner.x == brequest.request.workspace_parameters.max_corner.x &&
        brequest.request.workspace_parameters.min_corner.x == 0.0 &&
        brequest.request.workspace_parameters.min_corner.y == brequest.request.workspace_parameters.max_corner.y &&
        brequest.request.workspace_parameters.min_corner.y == 0.0 &&
        brequest.request.workspace_parameters.min_corner.z == brequest.request.workspace_parameters.max_corner.z &&
        brequest.request.workspace_parameters.min_corner.z == 0.0)
    {
      // ROS_WARN("Workspace parameters are not set for request %s.  Setting defaults", queries[i].name.c_str());
      brequest.request.workspace_parameters = workspace_parameters;
    }

    // Create all combinations of start states and path constraints
    std::vector<BenchmarkRequest> request_combos;
    
    //ROS_WARN("[EXPLORE] 2) start_states.size() = %lu", start_states.size());
    //ROS_WARN("[EXPLORE] 2) path_constraints.size() = %lu", path_constraints.size());
    
    ROS_WARN("[EXPLORE] brequest.request.start_state.joint_state.position =");
    std::vector<double> tmp1 = brequest.request.start_state.joint_state.position;
    for (int j=0; j<tmp1.size(); ++j)
    	ROS_WARN("[EXPLORE] %f", tmp1[j]);
    	
    //ROS_WARN("[EXPLORE] 2) brequest.request.goal_constraints.size() = %lu", brequest.request.goal_constraints.size());//=1
    // http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/Constraints.html :
    //ROS_WARN("[EXPLORE] 2) brequest.request.goal_constraints[0].joint_constraints.size() = %lu", brequest.request.goal_constraints[0].joint_constraints.size());//=6
    /*ROS_WARN("[EXPLORE] 2) brequest.request.goal_constraints[0].joint_constraints.position/tolerance_below/above =");
    std::vector<moveit_msgs::JointConstraint> tmp5 = brequest.request.goal_constraints[0].joint_constraints;
    for (int j=0; j<tmp5.size(); ++j)
    	ROS_WARN("[EXPLORE] %f (tol: below %f / above %f rad)", tmp5[j].position, tmp5[j].tolerance_below, tmp5[j].tolerance_above);*/
    //ROS_WARN("[EXPLORE] 2) brequest.request.goal_constraints[0].position_constraints.size() = %lu", brequest.request.goal_constraints[0].position_constraints.size());//=0
    //ROS_WARN("[EXPLORE] 2) brequest.request.goal_constraints[0].orientation_constraints.size() = %lu", brequest.request.goal_constraints[0].orientation_constraints.size());//=0
    //ROS_WARN("[EXPLORE] 2) brequest.request.goal_constraints[0].visibility_constraints.size() = %lu", brequest.request.goal_constraints[0].visibility_constraints.size());//=0
    
    //std::vector<moveit_msgs::JointConstraint> tmp7 = brequest.request.path_constraints.joint_constraints;
    //ROS_WARN("[EXPLORE] 2bis) brequest.request.path_constraints.joint_constraints.size() = %lu", tmp7.size());//=0
    //std::vector<moveit_msgs::Constraints> tmp8 = brequest.request.trajectory_constraints.constraints;
    //ROS_WARN("[EXPLORE] 2bis) brequest.request.trajectory_constraints.constraints.size() = %lu", tmp8.size());//=0
    ////ROS_ERROR("[EXPLORE] 2bis) brequest.request.trajectory_constraints.constraints[0].joint_constraints.tolerance_below/above =");
    ////std::vector<moveit_msgs::JointConstraint> tmp9 = brequest.request.trajectory_constraints.constraints[0].joint_constraints;
    ////ROS_ERROR("[EXPLORE] 2bis) brequest.request.trajectory_constraints.constraints[0].joint_constraints.tolerance_below/above.size() = %lu", tmp9.size());
    ////for (int j=0; j<tmp9.size(); ++j)
    ////	ROS_ERROR("[EXPLORE] (min %f / max %f rad)", tmp9[j].tolerance_below, tmp9[j].tolerance_above);
    // https://answers.ros.org/question/252573/ignored-joint-limits-by-moveit/
    // https://github.com/ros-industrial/universal_robot/issues/112
    // https://answers.ros.org/question/249253/temporarily-changing-of-joint-limits-in-robotstate/
    //CONCLUSION: I'm gonna have to add manually the angle limits to the fh_2_ur10_moveit_config/config/joint_limits.yaml
    //to overwrite the ones in the urdf and make them loaded to the ros param server
    // https://answers.ros.org/question/310376/why-dont-overridden-joints-limits-via-joint_limitsyaml-get-passed-to-moveit-ik-plugins/
    //AND THUS also have to somehow plot a marker to check that the limits are not violated ! :(
    //Unfortunately again I'm waiting shadow for making the versioning of this repo possible but they seem too busy
    //THey have to add my metric formulas mapped in 0 1 into a tab of their GUI
    //+ I git inited and forked the repo containing the URDF on my github, then giving them the ownance rights and deleting it, as their
    //hand may be private, but still nothing in even their private repos...
    // SO one should have to try this workaround manually.
    //see with
    //rostopic echo /move_group/display_planned_path
    //to ensure the robot is well bounded
    
    //ROS_WARN("[EXPLORE] 2) brequest.request.path_constraints.joint_constraints.size() = %lu", brequest.request.path_constraints.joint_constraints.size());//=0
    //ROS_WARN("[EXPLORE] 2) brequest.request.path_constraints.position_constraints.size() = %lu", brequest.request.path_constraints.position_constraints.size());//=0
    //ROS_WARN("[EXPLORE] 2) brequest.request.path_constraints.orientation_constraints.size() = %lu", brequest.request.path_constraints.orientation_constraints.size());//=0
    //ROS_WARN("[EXPLORE] 2) brequest.request.path_constraints.visibility_constraints.size() = %lu", brequest.request.path_constraints.visibility_constraints.size());//=0
    //ROS_WARN("[EXPLORE] 2) brequest.request.trajectory_constraints.constraints.size() = %lu", brequest.request.trajectory_constraints.constraints.size());//=0
    
    createRequestCombinations(brequest, start_states, path_constraints, request_combos);
    
    ////This shows that request_combos are actually nothing more than a copy of or brequest...
    //ROS_WARN("[EXPLORE] 2) request_combos.size() = %lu", request_combos.size());
    //ROS_WARN("[EXPLORE] request_combos[0].request.start_state.joint_state.position =");
    //std::vector<double> tmp2 = request_combos[0].request.start_state.joint_state.position;
    //for (int j=0; j<tmp2.size(); ++j)
    //	ROS_ERROR("[EXPLORE] %f", tmp2[j]);
    
    requests.insert(requests.end(), request_combos.begin(), request_combos.end());
  }

  // 3) Trajectory constraints are also treated like goal constraints
  for (std::size_t i = 0; i < traj_constraints.size(); ++i)
  {
    // Common benchmark request properties
    BenchmarkRequest brequest;
    brequest.name = traj_constraints[i].name;
    brequest.request.trajectory_constraints = traj_constraints[i].constraints;
    brequest.request.group_name = opts.getGroupName();
    brequest.request.allowed_planning_time = opts.getTimeout();
    brequest.request.num_planning_attempts = 1;

    if (brequest.request.trajectory_constraints.constraints.size() == 1 &&
        brequest.request.trajectory_constraints.constraints[0].position_constraints.size() == 1 &&
        brequest.request.trajectory_constraints.constraints[0].orientation_constraints.size() == 1 &&
        brequest.request.trajectory_constraints.constraints[0].visibility_constraints.size() == 0 &&
        brequest.request.trajectory_constraints.constraints[0].joint_constraints.size() == 0)
      shiftConstraintsByOffset(brequest.request.trajectory_constraints.constraints[0], goal_offset);

    std::vector<BenchmarkRequest> request_combos;
    std::vector<PathConstraints> no_path_constraints;
    createRequestCombinations(brequest, start_states, no_path_constraints, request_combos);
    ROS_WARN("[EXPLORE] 3) request_combos.size() = %lu", request_combos.size());
    requests.insert(requests.end(), request_combos.begin(), request_combos.end());
  }

  options_ = opts;
  return true;
}

void ModifiedBenchmarkExecutor::AdaptJointSpaceQueryMpiPi(moveit_msgs::MotionPlanRequest& query)
{//start and goal configurations in joint space
	//start:
	const std::vector<double> tmpo1 = query.start_state.joint_state.position;
  for (int j=0; j<tmpo1.size(); ++j)
  {
  	ROS_ERROR("[ENSURE] start conf, before : %f rad", tmpo1[j]);
  	query.start_state.joint_state.position[j] = AnyRadToMpiPiExc(tmpo1[j]);
  	ROS_ERROR("[ENSURE] start conf, after : %f rad", query.start_state.joint_state.position[j]);
	}
	
	//goal:
	std::vector<moveit_msgs::JointConstraint> tmpo2 = query.goal_constraints[0].joint_constraints;
    for (int j=0; j<tmpo2.size(); ++j)
    {
    	ROS_ERROR("[ENSURE] goal conf, before : %f rad", tmpo2[j].position);
    	query.goal_constraints[0].joint_constraints[j].position = AnyRadToMpiPiExc(tmpo2[j].position);
    	ROS_ERROR("[ENSURE] goal conf, after : %f rad", query.goal_constraints[0].joint_constraints[j].position);
  	}
}

void ModifiedBenchmarkExecutor::shiftConstraintsByOffset(moveit_msgs::Constraints& constraints,
                                                 const std::vector<double> offset)
{
  Eigen::Affine3d offset_tf(Eigen::AngleAxis<double>(offset[3], Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxis<double>(offset[4], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxis<double>(offset[5], Eigen::Vector3d::UnitZ()));
  offset_tf.translation() = Eigen::Vector3d(offset[0], offset[1], offset[2]);

  geometry_msgs::Pose constraint_pose_msg;
  constraint_pose_msg.position = constraints.position_constraints[0].constraint_region.primitive_poses[0].position;
  constraint_pose_msg.orientation = constraints.orientation_constraints[0].orientation;
  Eigen::Affine3d constraint_pose;
  tf::poseMsgToEigen(constraint_pose_msg, constraint_pose);

  Eigen::Affine3d new_pose = constraint_pose * offset_tf;
  geometry_msgs::Pose new_pose_msg;
  tf::poseEigenToMsg(new_pose, new_pose_msg);

  constraints.position_constraints[0].constraint_region.primitive_poses[0].position = new_pose_msg.position;
  constraints.orientation_constraints[0].orientation = new_pose_msg.orientation;
}

void ModifiedBenchmarkExecutor::createRequestCombinations(const BenchmarkRequest& brequest,
                                                  const std::vector<StartState>& start_states,
                                                  const std::vector<PathConstraints>& path_constraints,
                                                  std::vector<BenchmarkRequest>& requests)
{
  // Use default start state
  if (start_states.empty())
  {
    // Adding path constraints
    
    //ROS_WARN("[EXPLORE] path_constraints.size() = %lu", path_constraints.size());// = 0s
    for (std::size_t k = 0; k < path_constraints.size(); ++k)
    {
      BenchmarkRequest new_brequest = brequest;
      new_brequest.request.path_constraints = path_constraints[k].constraints[0];
      new_brequest.name = brequest.name + "_" + path_constraints[k].name;
      requests.push_back(new_brequest);
    }

    if (path_constraints.empty())
      requests.push_back(brequest);
  }
  else  // Create a request for each start state specified
  {
    for (std::size_t j = 0; j < start_states.size(); ++j)
    {
      BenchmarkRequest new_brequest = brequest;
      new_brequest.request.start_state = start_states[j].state;

      // Duplicate the request for each of the path constraints
      for (std::size_t k = 0; k < path_constraints.size(); ++k)
      {
        new_brequest.request.path_constraints = path_constraints[k].constraints[0];
        new_brequest.name = start_states[j].name + "_" + new_brequest.name + "_" + path_constraints[k].name;
        requests.push_back(new_brequest);
      }

      if (path_constraints.empty())
      {
        new_brequest.name = start_states[j].name + "_" + brequest.name;
        requests.push_back(new_brequest);
      }
    }
  }
}

bool ModifiedBenchmarkExecutor::plannerConfigurationsExist(const std::map<std::string, std::vector<std::string>>& planners,
                                                   const std::string& group_name)
{
  // Make sure planner plugins exist
  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end();
       ++it)
  {
    bool plugin_exists = false;
    for (std::map<std::string, planning_interface::PlannerManagerPtr>::const_iterator planner_it =
             planner_interfaces_.begin();
         planner_it != planner_interfaces_.end() && !plugin_exists; ++planner_it)
    {
      plugin_exists = planner_it->first == it->first;
    }

    if (!plugin_exists)
    {
      ROS_ERROR("Planning plugin '%s' does NOT exist", it->first.c_str());
      return false;
    }
  }

  // Make sure planning algorithms exist within those plugins
  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end();
       ++it)
  {
    planning_interface::PlannerManagerPtr pm = planner_interfaces_[it->first];
    const planning_interface::PlannerConfigurationMap& config_map = pm->getPlannerConfigurations();

    // if the planner is chomp or stomp skip this function and return true for checking planner configurations for the
    // planning group otherwise an error occurs, because for OMPL a specific planning algorithm needs to be defined for
    // a planning group, whereas with STOMP and CHOMP this is not necessary
    if (pm->getDescription().compare("stomp") || pm->getDescription().compare("chomp"))
      continue;

    for (std::size_t i = 0; i < it->second.size(); ++i)
    {
      bool planner_exists = false;
      for (planning_interface::PlannerConfigurationMap::const_iterator map_it = config_map.begin();
           map_it != config_map.end() && !planner_exists; ++map_it)
      {
        std::string planner_name = group_name + "[" + it->second[i] + "]";
        planner_exists = (map_it->second.group == group_name && map_it->second.name == planner_name);
      }

      if (!planner_exists)
      {
        ROS_ERROR("Planner '%s' does NOT exist for group '%s' in pipeline '%s'", it->second[i].c_str(),
                  group_name.c_str(), it->first.c_str());
        std::cout << "There are " << config_map.size() << " planner entries: " << std::endl;
        for (planning_interface::PlannerConfigurationMap::const_iterator map_it = config_map.begin();
             map_it != config_map.end() && !planner_exists; ++map_it)
          std::cout << map_it->second.name << std::endl;
        return false;
      }
    }
  }

  return true;
}

bool ModifiedBenchmarkExecutor::loadPlanningScene(const std::string& scene_name, moveit_msgs::PlanningScene& scene_msg)
{
  bool ok = false;
  try
  {
    if (pss_->hasPlanningScene(scene_name))  // whole planning scene
    {
      moveit_warehouse::PlanningSceneWithMetadata pswm;
      ok = pss_->getPlanningScene(pswm, scene_name);
      scene_msg = static_cast<moveit_msgs::PlanningScene>(*pswm);

      if (!ok)
        ROS_ERROR("Failed to load planning scene '%s'", scene_name.c_str());
    }
    else if (psws_->hasPlanningSceneWorld(scene_name))  // Just the world (no robot)
    {
      moveit_warehouse::PlanningSceneWorldWithMetadata pswwm;
      ok = psws_->getPlanningSceneWorld(pswwm, scene_name);
      scene_msg.world = static_cast<moveit_msgs::PlanningSceneWorld>(*pswwm);
      scene_msg.robot_model_name =
          "NO ROBOT INFORMATION. ONLY WORLD GEOMETRY";  // this will be fixed when running benchmark

      if (!ok)
        ROS_ERROR("Failed to load planning scene '%s'", scene_name.c_str());
    }
    else
      ROS_ERROR("Failed to find planning scene '%s'", scene_name.c_str());
  }
  catch (std::exception& ex)
  {
    ROS_ERROR("Error loading planning scene: %s", ex.what());
  }
  ROS_INFO("Loaded planning scene successfully");
  return ok;
}

bool ModifiedBenchmarkExecutor::loadQueries(const std::string& regex, const std::string& scene_name,
                                    std::vector<BenchmarkRequest>& queries)
{
  if (regex.empty())
    return true;

  std::vector<std::string> query_names;
  try
  {
    pss_->getPlanningQueriesNames(regex, query_names, scene_name);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR("Error loading motion planning query names: %s", ex.what());
    return false;
  }

  if (query_names.empty())
  {
    ROS_ERROR("Scene '%s' has no associated queries", scene_name.c_str());
    return false;
  }

  for (std::size_t i = 0; i < query_names.size(); ++i)
  {
    ROS_ERROR("[EXPLORE] query's name i = %lu", i);
    moveit_warehouse::MotionPlanRequestWithMetadata planning_query;
    try
    {
      pss_->getPlanningQuery(planning_query, scene_name, query_names[i]);
      // https://docs.ros.org/api/moveit_ros_warehouse/html/planning__scene__storage_8cpp_source.html#l00233
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("Error loading motion planning query '%s': %s", query_names[i].c_str(), ex.what());
      continue;
    }

    BenchmarkRequest query;
    query.name = query_names[i];
    query.request = static_cast<moveit_msgs::MotionPlanRequest>(*planning_query); // key point
    // http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/MotionPlanRequest.html
    queries.push_back(query);
    
    /*ROS_ERROR("[EXPLORE] queries.back().request.start_state.joint_state.position.size() = %lu",
    queries.back().request.start_state.joint_state.position.size()); // To know whether a component has 6 or 12 values (only start or goal as well)
    // Turns out a component of queries has 15 joint values ??? Wtf?
    std::vector<double> tmp = queries.back().request.start_state.joint_state.position;
    for (int j=0; j<tmp.size(); ++j)
    	ROS_ERROR("[EXPLORE] %f", tmp[j]);
    //And strangely some values aren't searchable into the file scene_ground_with_boxes.queries!! So I don't know from where come those datas*/
    
  }
  ROS_INFO("Loaded queries successfully");
  return true;
}

bool ModifiedBenchmarkExecutor::loadStates(const std::string& regex, std::vector<StartState>& start_states)
{
  if (regex.size())
  {
    boost::regex start_regex(regex);
    std::vector<std::string> state_names;
    rs_->getKnownRobotStates(state_names);
    for (std::size_t i = 0; i < state_names.size(); ++i)
    {
      boost::cmatch match;
      if (boost::regex_match(state_names[i].c_str(), match, start_regex))
      {
        moveit_warehouse::RobotStateWithMetadata robot_state;
        try
        {
          if (rs_->getRobotState(robot_state, state_names[i]))
          {
            StartState start_state;
            start_state.state = moveit_msgs::RobotState(*robot_state);
            start_state.name = state_names[i];
            start_states.push_back(start_state);
          }
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("Runtime error when loading state '%s': %s", state_names[i].c_str(), ex.what());
          continue;
        }
      }
      
      /*ROS_ERROR("[EXPLORE] start_states.back().state.joint_state.position.size() = %lu",
    	start_states.back().state.joint_state.position.size()); // To know whether a component has 6 values (number of joint for the planning group)
    	std::vector<double> tmp = start_states.back().state.joint_state.position;
    	for (int j=0; j<tmp.size(); ++j)
    		ROS_ERROR("[EXPLORE] %f", tmp[j]);*/
      
    }

    if (start_states.empty())
      ROS_WARN("No stored states matched the provided start state regex: '%s'", regex.c_str());
  }
  ROS_INFO("Loaded states successfully");
  return true;
}

bool ModifiedBenchmarkExecutor::loadPathConstraints(const std::string& regex, std::vector<PathConstraints>& constraints)
{
  if (regex.size())
  {
    std::vector<std::string> cnames;
    cs_->getKnownConstraints(regex, cnames);
    for (std::size_t i = 0; i < cnames.size(); ++i)
    {
      moveit_warehouse::ConstraintsWithMetadata constr;
      try
      {
        if (cs_->getConstraints(constr, cnames[i]))
        {
          PathConstraints constraint;
          constraint.constraints.push_back(*constr);
          constraint.name = cnames[i];
          constraints.push_back(constraint);
        }
      }
      catch (std::exception& ex)
      {
        ROS_ERROR("Runtime error when loading path constraint '%s': %s", cnames[i].c_str(), ex.what());
        continue;
      }
    }
    
    //The following is unachieved, as a ROS WARN is saying : Benchmark loaded 0 starts, 0 goals, 0 path constraints, 0 trajectory constraints, and 50 queries
    /*ROS_ERROR("[EXPLORE] constraints.back().constraints.joint_state.position.size() = %lu",
  	constraints.back().constraints.joint_state.position.size()); // To know whether a component has 6 values (number of joint for the planning group)
  	std::vector<double> tmp = constraints.back().constraints.joint_state.position;
  	for (int j=0; j<tmp.size(); ++j)
    	ROS_ERROR("[EXPLORE] %f", tmp[j]);*/

    if (constraints.empty())
      ROS_WARN("No path constraints found that match regex: '%s'", regex.c_str());
    else
      ROS_INFO("Loaded path constraints successfully");
  }
  return true;
}

bool ModifiedBenchmarkExecutor::loadTrajectoryConstraints(const std::string& regex,
                                                  std::vector<TrajectoryConstraints>& constraints)
{
  if (regex.size())
  {
    std::vector<std::string> cnames;
    tcs_->getKnownTrajectoryConstraints(regex, cnames);

    for (std::size_t i = 0; i < cnames.size(); ++i)
    {
      moveit_warehouse::TrajectoryConstraintsWithMetadata constr;
      try
      {
        if (tcs_->getTrajectoryConstraints(constr, cnames[i]))
        {
          TrajectoryConstraints constraint;
          constraint.constraints = *constr;
          constraint.name = cnames[i];
          constraints.push_back(constraint);
        }
      }
      catch (std::exception& ex)
      {
        ROS_ERROR("Runtime error when loading trajectory constraint '%s': %s", cnames[i].c_str(), ex.what());
        continue;
      }
    }

    if (constraints.empty())
      ROS_WARN("No trajectory constraints found that match regex: '%s'", regex.c_str());
    else
      ROS_INFO("Loaded trajectory constraints successfully");
  }
  return true;
}

template<typename T>
static std::vector<T> slice(std::vector<T> const &v, int m, int n)
{
    auto first = v.cbegin() + m;
    auto last = v.cbegin() + n + 1;

    std::vector<T> vec(first, last);
    return vec;
}

void ModifiedBenchmarkExecutor::runBenchmark(moveit_msgs::MotionPlanRequest request,
					     const std::string& queryName,
					     const std::map<std::string, std::vector<std::string>>& planners, 
					     int runs, 
					     const std::string& metricChoice, 
					     const std::string& sceneName,
					     const bool GENERATE_LOGS,
					     const bool GENERATE_ANIMATION_RVIZ,
					     const std::vector<std::vector<double>>& jointAnglesMinMax,
					     const std::list<std::string>& limitedInAngleActuatedJointsNames,
					     unsigned int& no_first_kept_restart)
{
  benchmark_data_.clear();

  double countdown = request.allowed_planning_time;
  ROS_WARN("I ensure that countdown is in seconds -- countdown = '%lf'", countdown);

  unsigned int num_planners = 0;
  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); 
  		 it != planners.end(); ++it)
    num_planners += it->second.size();

  // Iterate through all planner plugins
  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); 
  		 it != planners.end(); ++it)
  {
    // Iterate through all planners associated with the plugin
    for (std::size_t i = 0; i < it->second.size(); ++i)
    {
      // This container stores all of the benchmark data for this planner
      PlannerBenchmarkData planner_data(runs);

      request.planner_id = it->second[i];

      // Planner start events
      for (std::size_t j = 0; j < planner_start_fns_.size(); ++j)
        planner_start_fns_[j](request, planner_data);

      planning_interface::PlanningContextPtr context = planner_interfaces_[it->first]-> 
      						      															getPlanningContext(planning_scene_, request);
      
			//addition of mine that should be below, but already lot of content    
      // https://www.learncpp.com/cpp-tutorial/78-function-pointers/
			double (*qualityFcnPtr)(const robot_trajectory::RobotTrajectory&); // pointer which points 				to the chosen metric function during all the following, even though one could alternate 			between metrics...
			std::string metric1 ("energy");
			std::string metric2 ("relevancy");
			if (metricChoice.compare(metric1)==0)
			{
				qualityFcnPtr = &evaluate_plan;
				ROS_INFO("The chosen metric, over which optimization will be done, is set on : '%s'." 		   					 "Currently you can change it by acting on iplanr_description/" 
						 		 "benchmark_configs/scene_ground_with_boxes.yaml", metric1.c_str());
			} else if (metricChoice.compare(metric2)==0)
			{
				qualityFcnPtr = &evaluate_plan_cart;
				ROS_INFO("The chosen metric, over which optimization will be done, is set on : '%s'." 		   					 "Currently you can change it by acting on iplanr_description/" 
						 		 "benchmark_configs/scene_ground_with_boxes.yaml", metric2.c_str());
			} else
			{
				ROS_ERROR("In iplanr_description/benchmark_configs/scene_ground_with_boxes.yaml," 
									"you did not set any metric over which do the optimization process." 
									"In parameters list, please add metric_choice: relevancy OR energy");
			}
      			      
      for (int j = 0; j < runs; ++j)
      {
        ROS_WARN("--------------------------------");
        ROS_WARN("--------------------------------");
        ROS_WARN("RUN NUMBER: %d", j+1);
        ROS_WARN("--------------------------------");
        ROS_WARN("--------------------------------");
        
        // Pre-run events
        for (std::size_t k = 0; k < pre_event_fns_.size(); ++k)
          pre_event_fns_[k](request);

      	// Modification of the initial code : addition of a while countdown not exceeded -- tweak.
      	planning_interface::MotionPlanDetailedResponse mp_res, mp_res_before_exceeding, first_mp_res;
      	double total_time = 0., currentRealTime;
      	double planQuality = 0., previousPlanQuality, first_planQuality;
      	bool solved = true; //In case the last mp_res before exceed is not solved but a previous is
      	bool finally_solved = false;
      	int restart = 0;
				bool lastIterationIsConcluding;
      	
      	// Offline acquisition of the planner's parameters from the server in order to tweak them
      	std::map<std::string, std::vector<std::string>> plannersParameterNames = 
      	constructMoveitPlannersParameterNamesDictionnary();
      	const std::string planner = planners.begin()->second[0]; // second[0] bc: planners of type {plugin1_name: ["planner1_name", "planner2_name"], plugin2_name: ["planner1_name]}
				const std::string plannerToBeWritten = planner.substr(0, planner.size()-14).append("wrapped");
				std::string acceptanceFuncExpression = "1-t/T";
      	const std::string pathPlannerParameters = "/moveit_run_benchmark/planner_configs/"+planner;
      	const std::string pathPlannerParamBoundaries = "/moveit_run_parameter_optimizer/" 							       																	 "planner_parameters_boundaries";
      	XmlRpc::XmlRpcValue previousPlannerParameters, 
      			    parametersSet_Xml = getServerParameters(pathPlannerParameters), 
      			    paramBoundariesAndSteps_Xml = getServerParameters(pathPlannerParamBoundaries);
      	std::vector<std::string> vecPlannerParamNames = plannersParameterNames[planner], plannerParamNamesToModify;
      	int nbPlannerParams = vecPlannerParamNames.size();
      	
				//Writing of decisive parameters in an unique .log file through the whole optimization process, this 					block will have to be removed for real time applications
				//Because of laziness, I prefer simply write the acceptance function as a string and conserve the 				already hardcoded expression, rather than each time re-parsing my string function expression which 					would, I believe, drastically slow down the process. 
				//However here is an idea https://archive.codeplex.com/?p=exprtk
      	
				std::ofstream myfile;
				std::string logfileName;
				unsigned int bufLengthMax = 256; //maximal number of bytes (= interpreted chars) on a line of datas, let's see...
				
      	if (GENERATE_LOGS)
      	{ros::param::get("/moveit_run_benchmark/benchmark_config/parameters/logname", logfileName);

					myfile.open(logfileName, std::ios::out | std::ios::app);

					myfile << "planner = " << plannerToBeWritten << std::endl;
					myfile << "metric = " << metricChoice << std::endl;
					myfile << "scene = " << sceneName << std::endl;
					myfile << "query = " << queryName << std::endl;
					myfile << "countdown = " << countdown << std::endl;
					myfile << "acceptance = " << acceptanceFuncExpression << std::endl;
					myfile << "run = " << j+1 << std::endl;
					// Write the one-line ordered list of parameters for writing laterly online their values
					for (std::size_t i = 0; i < nbPlannerParams; ++i)
						myfile << vecPlannerParamNames[i] << ", ";
					myfile << "quality:" << std::endl;
				}

				/*In fact as soon as we launch RViz the planner parameters from the dedicated .yaml file already has 
    			its parameters assigned some values, however those initial values are retrieved from the 
    			ompl_planning.yaml file, hence not necessarily belonging to the intervals of the exploration space
    			we defined in planners_numerical_boundaries_andSteps.yaml. This then not only doesn't respect our
    			space, but also create a super annoying mess into my matlab postprocess (precisely in the way 
    			it handles the n axis associated to the n params) to show the evolution !*/
				initializePlannerParameters(pathPlannerParameters, paramBoundariesAndSteps_Xml, vecPlannerParamNames);
				
	      ////////////////////////////////////////////////////////////
	      //Let's display the inputed current query (start+goal confs)
	      ////////////////////////////////////////////////////////////
		    std::vector<std::string> texts;
		    std::size_t previous_size;// https://github.com/PickNikRobotics/rviz_visual_tools
		    //
				const rviz_visual_tools::colors start_conf_color = rviz_visual_tools::GREEN;
				const rviz_visual_tools::colors goal_conf_color = rviz_visual_tools::RED;
				const rviz_visual_tools::colors traj_rope_color_opti = rviz_visual_tools::CYAN;
				const rviz_visual_tools::colors traj_rope_color_orig = rviz_visual_tools::PINK;
				const rviz_visual_tools::colors text_color = rviz_visual_tools::BLACK;
				rviz_visual_tools::colors color;
				//
		    double alti_min = 1.7, alti_step = 0.075;
		    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
		    //
				moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
				/*// We can also print the name of the end-effector link for this group.
				ROS_WARN("[DEBUG] End effector link: %s", move_group.getEndEffectorLink().c_str());*/
				const robot_state::JointModelGroup* joint_model_group =
																					move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
				if (GENERATE_ANIMATION_RVIZ)
        {
		      //What is always displayed: start and goal confs,
		      //no matter if everything failed:
		      visual_tools_->deleteAllMarkers();
		      
					// construct the start conf:
					// It definitely seems that the queries printed do not match with the scene_ground_with_boxes queries file:
					std::vector<double> start_config_current = slice<double>(request.start_state.joint_state.position,0,5);
					/*for (int j=0; j<start_config_current.size(); ++j)
						ROS_ERROR("[DEBUG] %f rad", start_config_current[j]);*/
		      
		      // construct the goal conf:
		      std::vector<moveit_msgs::JointConstraint> tmp6 = request.goal_constraints[0].joint_constraints;
					std::vector<double> goal_config_current;
					for (int j=0; j<tmp6.size(); ++j)
					{
						/*ROS_ERROR("[DEBUG] %f rad", tmp6[j].position);*/
						goal_config_current.push_back(tmp6[j].position);
					}
			
					// Btw I also found that whithin the RViz 'Motion PLanning' display tree, tick 'Show Trail' and put a tremendous
					// value for its size was doing the job (of displaying start and goal confs),
					// even though the start conf was first displayed, frozen, and only then (at the end of the movement) was frozen the goal conf
					// so it didn't feel like the query was already known beforehand.
					// THOUGH I'M STILL UNSURE ABOUT HOW OR WHERE THE RANDOM START AND GOAL STATES ARE SAMPLED
					// (IT IS FOR SURE NOT IN THIS FILE WHEN WE SEARCH FOR 'combo'),
					// TODO investigate the goal_constraints here http://docs.ros.org/kinetic/api/moveit_msgs/html/definePlanningRequest.html
					// (but I think I did.)
					// AND NEITHER HOW THEY ARE ENSURED TO BE FEASIBLE BUT ANYWAY LET'S TRUST HUMANS...
			
					// https://answers.ros.org/question/11845/rviz-configuration-file-format/ <- where to get and set the RViz latest config
					// I try to save the biggest modifications in benchmarks folder of this moveit fork
					
					// Requires to add a RobotState plugin in RViz listening to the topic "/display_start_configuration":
					// Don't forget to tick in RViz 'show highlights' if your colors aren't rviz_visual_tools::DEFAULT
					visual_tools_->publishRobotState(start_config_current, joint_model_group, start_conf_color);
					ROS_INFO("Start conf gives this.");
					
					// Requires to add a second RobotState plugin in RViz listening to the topic "/display_goal_configuration":
					// Don't forget to tick in RViz 'show highlights' if your colors aren't rviz_visual_tools::DEFAULT
					visual_tools2_->publishRobotState(goal_config_current, joint_model_group, goal_conf_color);
					ROS_INFO("Goal conf gives this.");
					
					// legend above the scene
					std::vector<std::string> listToVector(limitedInAngleActuatedJointsNames.begin(), limitedInAngleActuatedJointsNames.end());
			    for (unsigned int i = goal_config_current.size(); i-- > 0; )
			    //for (auto i = goal_config_current.rbegin(); i != goal_config_current.rend(); ++i)
			    	// https://stackoverflow.com/questions/3610933/iterating-c-vector-from-the-end-to-the-begin
			    	// doesn't work here, I will have *iterator for goal config but not for start config!
			    	// so : https://stackoverflow.com/questions/5458204/unsigned-int-reverse-iteration-with-for-loops
			    	//std::list<std::string>::iterator itCurr = std::next(limitedInAngleActuatedJointsNames.begin(),i);
			    	texts.push_back(listToVector[i] + " : " + std::to_string(start_config_current[i]) + " / " + std::to_string(goal_config_current[i]) +
			    	" rad (min : " + std::to_string(jointAnglesMinMax[i][0]) + ", max : " + std::to_string(jointAnglesMinMax[i][1]) + ")");
			    texts.push_back("start/goal joint configurations + angle limits : (query '" + queryName + "')");
			    texts.push_back("plan computation countdown T = " + std::to_string(countdown) + " sec");
			    
			    for (std::size_t i = 0; i < texts.size(); ++i)
			    {
			    	pose.translation().z() = alti_min + i*alti_step;
			    	visual_tools_->publishText(pose, texts[i], text_color, rviz_visual_tools::XXLARGE, false);
			    }
			    
			    visual_tools_->trigger();
		    } //end display of the queries
				
				unsigned int solved_proof = 0, kept_proof = 0, first_solved = 0;
      	// Solve problem, once, as before,
      	ros::WallTime start = ros::WallTime::now();
				lastIterationIsConcluding = false;
      	solved = context->solve(mp_res); //github.com/ros-planning/moveit/issues/1230
      	if (solved)
      	{
      	  solved_proof +=1; kept_proof +=1; first_solved +=1;
      		// solved_ = calls that found a solution,
      		// kept_ = solved_ that : not exceed the countdown + improve the quality + or are drafted (FR: etre repeche)
      		// first_ = have been solved already before entering the while-tweaking loop
      		
					lastIterationIsConcluding = true; //this is only a hack to get rid of the first-shot-only results in my matlab plots
																						//whose purpose are showing how the optimizer evolutes
					
					mp_res_before_exceeding = mp_res; /*REMOVE THIS LINE IN ORDER TO ONLY DISPLAY IN RVIZ THE "TWEAKED"
					PLANS AND NOT INCLUDE THE FIRST SUCCEEDING ONLY ONE BEFORE EXCEEDING*/
					
      	  planQuality = (*qualityFcnPtr)( *(mp_res.trajectory_[0]) ); // TODO HOW/WHY can it 						exists several found trajectories ? (why do I have to retrieve only the first [0] of 						them?)
      	}
      	total_time += (ros::WallTime::now() - start).toSec();
      	
      	if (solved)
      	{ //putting the things that slow down the process outside the time measurement, as it will be removed sooner or later
					ROS_WARN("FIRST CALL FOUND SOMETHING -- Current quality = %lf out of 1", planQuality); // TO BE REMOVED
					if (GENERATE_LOGS)
					{
						writePlannerParametersAndQuality(parametersSet_Xml, myfile, vecPlannerParamNames, nbPlannerParams, planQuality);
					}
					if (GENERATE_ANIMATION_RVIZ)
					{ //Store results to potentially compare with the last kept evolution of the optimizer, if it exists
						first_mp_res = mp_res_before_exceeding;
						first_planQuality = planQuality;
					}
				}
      	
      	// and as many times as the countdown allows it,
      	while (total_time < countdown)
      	{
      	  ros::WallTime start = ros::WallTime::now();
					lastIterationIsConcluding = false;
      	  if (solved) //then mp_res isn't empty, if not mp_res is empty and then we could as 													legitly return the empty mp_res_before_exceeding as mp_res
      	  { //(We save the previous iteration only if it solved the pb AND ALLOCATED TIME REMAINS.)
      	    mp_res_before_exceeding = mp_res; // addition to the first iteration above
      	    previousPlanQuality = planQuality;
      	    previousPlannerParameters = parametersSet_Xml;
      	  }
      	  
      	  // while tweaking the planner's parameters more or less smartly, though this block could be commented to get simply the best of what each planner randomness has to offer,
      	  alterPlannerParameters(parametersSet_Xml, paramBoundariesAndSteps_Xml,
      	  			 								 vecPlannerParamNames, nbPlannerParams, plannerParamNamesToModify);
					//the alteration needs to not only be kept in RAM but also to be updated on the ROS param server on which relies the planner!
					updateServerParameters(pathPlannerParameters, parametersSet_Xml, plannerParamNamesToModify);
      	  			 
      	  mp_res = planning_interface::MotionPlanDetailedResponse(); //this constructor acts as a 					clearer, otherwise solve(mp_res) would append a set of trajectories to its current 						vector trajectory_ attribute!
      	  restart += 1;
      	  solved = context->solve(mp_res);
      	  if (solved)
      	  {
      	    solved_proof +=1;
      	    // and while comparing the qualities as well to see if these tweaks lead to improvment.
      	    planQuality = (*qualityFcnPtr)( *(mp_res.trajectory_[0]) );
      	    ROS_WARN("RESTART NUMBER %d FOUND SOMETHING -- Current quality = %lf out of 1", restart, planQuality);
	    			ROS_WARN("Current allocated countdown = %f sec", countdown);
      	    if (planQuality > previousPlanQuality) 
						{ //write the new better results
							if (GENERATE_LOGS)
								writePlannerParametersAndQuality(parametersSet_Xml, myfile, vecPlannerParamNames, nbPlannerParams, planQuality);
							kept_proof +=1;
							ROS_ERROR("[DEBUG] kept_proof +=1");
							lastIterationIsConcluding = true;
							ROS_WARN("AND IT IMPROVES THE QUALITY METRIC");
						}
						else //(planQuality <= previousPlanQuality)
      	    { 
      	      currentRealTime = total_time + (ros::WallTime::now() - start).toSec();
      	      if ( !accepted(currentRealTime, countdown) )
      	      { //switch back to the previous solution, unless Acceptance function lets it go
      	      	mp_res = mp_res_before_exceeding;
      	      	planQuality = previousPlanQuality;
      	      	parametersSet_Xml = previousPlannerParameters;
      	      }
      	      else
							{ //write the new worse results
								if (GENERATE_LOGS)
									writePlannerParametersAndQuality(parametersSet_Xml, myfile, vecPlannerParamNames, nbPlannerParams, planQuality);
								kept_proof +=1;
								ROS_ERROR("[DEBUG] kept_proof +=1");
								lastIterationIsConcluding = true;
      	      	ROS_WARN("Current time = %f from the beginning of the run -- "
      	      				 	 "This is a worse quality than the previous one (%f) which was found and kept in memory, "
      	      				 	 "but might be accepted by the simulated annealing acceptance function, "
      	      				 	 "as long as we didn't exceed the countdown (being %f sec)!",
      	      				 	 currentRealTime, previousPlanQuality, countdown);
							}
      	    }
      	  }
      	  total_time += (ros::WallTime::now() - start).toSec();
      	}
      	if (GENERATE_LOGS)
					myfile.close();

      	if ((solved_proof > 0) && (lastIterationIsConcluding == true))
      	{ //then the while iteration that exceeded the countdown is concluding and we appended a useless line to the saving file
      		//that has to be removed since it exceeded the countdown!
      	  finally_solved = true; //TODO error to put it here BUT I don't care about the log bunch...
      	  ROS_INFO("It exists some iteration which has managed to solve the problem. And the one which best solves it, is stored");
      	  if (GENERATE_LOGS)
      		{
						//temporary switch to C language to remove the last line:
						deleteLastLine(logfileName.c_str(), bufLengthMax);
						// just to be sure that we indeed removed the last line: (comeback to C++)
						myfile.open(logfileName, std::ios::out | std::ios::app);
						myfile << std::endl << "succeeding_restarts = " << kept_proof-1 << std::endl; //kept_proof-1 as a line just popped out
					}
      	} else
				{
					if (GENERATE_LOGS)
      		{
						myfile.open(logfileName, std::ios::out | std::ios::app);
						myfile << "succeeding_restarts = " << kept_proof << std::endl;
					}
				}
				if (GENERATE_LOGS)
      	{
					// Also I forgot to write the steps and they are not deducible via diff() in matlab, as it requires 2 restarts at least! So:
					myfile << "step = [";
					for (std::size_t i = 0; i < nbPlannerParams; ++i)
						myfile << " " << paramBoundariesAndSteps_Xml[vecPlannerParamNames[i]]["step"];
					myfile << " ]" << std::endl << std::endl;
					myfile.close();
				}

        // Post-run events
        for (std::size_t k = 0; k < post_event_fns_.size(); ++k)
          post_event_fns_[k](request, mp_res_before_exceeding, planner_data[j]);
          
        // Collect data
        if (GENERATE_LOGS)
      	{
		      start = ros::WallTime::now();
		      collectMetrics(planner_data[j], mp_res_before_exceeding, finally_solved, total_time);
		      double metrics_time = (ros::WallTime::now() - start).toSec();
		      ROS_INFO("Spent %lf seconds collecting metrics", metrics_time);
		    }
        
        if (GENERATE_ANIMATION_RVIZ)
        {
			    /*ROS_WARN("[DEBUG] first_solved = %d", first_solved);
			    ROS_WARN("[DEBUG] kept_proof = %d", kept_proof);*/
			    
			    //the movement animation:
					bool stop_at_first_move = false; //true blocks the whole benchmark process at the first ever found move
			    if (first_solved==0)
			    {
			    	if (kept_proof >=1)
			    	{//this algo leaded to obtain a solution where classic planner couldn't after 1 call
			    	
			    		//observation!!! : the dumb nonoptimal fast planners, when not finding any solution, do use the whole countdown
			    		//we allow them, they are content with not using the whole countdown only when they solve the problem!
							//So never expect to plot a tweaked movement alone on the scene without an initial succeeding first shot to compare with!
			    	
			    		ROS_ERROR("[DEBUG] first_solved == 0");
			    		ROS_ERROR("[DEBUG] kept_proof >=1");
			    		no_first_kept_restart = 1;
			    	}
			    }
			    else // first_solved ==1
			    {
			    	if (kept_proof==1)
			    	{ //the first call to the original planner found smthg, but the algo didn't had time to take over, or failed
			    		previous_size = texts.size();
							texts.push_back(metricChoice + " (metric) : " + std::to_string(first_planQuality*100.) + "%");
							texts.push_back("motion planner : " + planner);
							texts.push_back("(" + std::to_string(j+1) + "th experiment replica)"); //j+1 = the number of the run (out of 5 currently, see the python file findOptimal...)
							for (std::size_t i = previous_size; i < texts.size(); ++i)
							{
								pose.translation().z() = alti_min + i*alti_step;
								if (i != texts.size())
								{
									color = traj_rope_color_orig;
								} else
								{
									color = text_color;
								}
								visual_tools_->publishText(pose, texts[i], color, rviz_visual_tools::XXLARGE, false);
							}
							visual_tools_->trigger();
			
							//trajectory markers
							visual_tools_->publishTrajectoryLine(first_mp_res.trajectory_.back(), joint_model_group, traj_rope_color_orig);
							visual_tools_->trigger(); //forces a refresh with the new trajectory markers
							ROS_INFO("(Originally parametrized planner solution EE path gave this)");
		
							visual_tools_->publishTrajectoryPath(first_mp_res.trajectory_.back(), stop_at_first_move);
							ROS_INFO("Originally parametrized planner solution movement gives this");
				
							std::chrono::seconds dura(10);
							std::cout << "About to wait 10s while movement animation finishes\n";
							std::this_thread::sleep_for( dura );
							std::cout << "Waited 10s after path\n";
			    	}
			    	else if (kept_proof > 1)
			    	{ //original planner found smthg + the algo had time to take over, let's compare the moves!
			    	
			    		/*ROS_ERROR("[DEBUG] first_solved == 1");
			    		ROS_ERROR("[DEBUG] kept_proof > 1");*/
			    	
			    		previous_size = texts.size();
			    		texts.push_back(planner + " (pink) : " + std::to_string(first_planQuality*100.) + "%");
			    		texts.push_back("Versus.");
			    		texts.push_back(plannerToBeWritten + " (cyan) : " + std::to_string(previousPlanQuality*100.) + "%");
							texts.push_back(metricChoice + " (metric) : ");
							texts.push_back("(" + std::to_string(j+1) + "th experiment replica)"); //j+1 = the number of the run (out of 5 currently)
							texts.push_back("acceptance(t) := " + acceptanceFuncExpression);
							for (std::size_t i = previous_size; i < texts.size(); ++i)
							{
								pose.translation().z() = alti_min + i*alti_step;
								switch(i)
								{
									case 8: color = traj_rope_color_orig; //case previous_size and previous_size+2 didnt want to compile :'(
																			break;
									case 10: color = traj_rope_color_opti;
																				break;
									default: color = text_color;
								}
								visual_tools_->publishText(pose, texts[i], color, rviz_visual_tools::XXLARGE, false);
							}
							visual_tools_->trigger();
						
							//trajectory markers
							visual_tools_->publishTrajectoryLine(mp_res_before_exceeding.trajectory_.back(), joint_model_group, traj_rope_color_opti);
							ROS_INFO("(Wrapped planner solution EE path gave this)");
							
							visual_tools_->publishTrajectoryLine(first_mp_res.trajectory_.back(), joint_model_group, traj_rope_color_orig);
							visual_tools_->trigger(); //forces a refresh with the new trajectory markers
							ROS_INFO("(Originally parametrized planner solution EE path gave this)");
		
							visual_tools_->publishTrajectoryPath(mp_res_before_exceeding.trajectory_.back(), stop_at_first_move);
							ROS_INFO("Wrapped planner solution movement gives this");
							
							visual_tools2_->publishTrajectoryPath(first_mp_res.trajectory_.back(), stop_at_first_move);
							ROS_INFO("Originally parametrized planner solution movement gives this");
				
							std::chrono::seconds dura(10);
							// not use sleep() as here :
							// http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/src/doc/move_group_interface_tutorial.html
							// reason : https://stackoverflow.com/questions/49071285/sleep-vs-sleep-for
							std::cout << "About to wait 10s while movement animations finish\n";
							std::this_thread::sleep_for( dura );
							std::cout << "Waited 10s after pathes launching\n";
			    	}
			    } //end animation
				} //end of RViz part
				
			}
			
      // Planner completion events
      for (std::size_t j = 0; j < planner_completion_fns_.size(); ++j)
        planner_completion_fns_[j](request, planner_data);
			
			if (GENERATE_LOGS)
      	benchmark_data_.push_back(planner_data);
    }
  }
}

XmlRpc::XmlRpcValue ModifiedBenchmarkExecutor::getServerParameters(const std::string& path) // the output type is for debug purpose, I want in the end to return only one planner's set of parameters, and in a struct or array type!!
{
  XmlRpc::XmlRpcValue tmp;
  if (ros::param::get(path, tmp)) // http://docs.ros.org/kinetic/api/roscpp/html/namespaceros_1_1param.html#a8946be052ed53e5e243dbd0c9bb23b8a
  // TODO && check that, apart from the type parameter, it exists parameters to tweak (PRMstarkConfigDefault creates an exception, see ompl_planning.yaml)
  {
    /*ROS_INFO("Parameter range = %f", double((*this)["range"]));
    ROS_INFO("Parameter type = %s", std::string((*this)["type"]).c_str());*/
    return tmp;
  }
  else
    ROS_ERROR("No path '%s' found on param server. Type 'rosparam list' in the console to see the paths.", path.c_str());
}

void ModifiedBenchmarkExecutor::updateServerParameters(const std::string& pathPlannerParameters,
																											 XmlRpc::XmlRpcValue& paramSetCurrent, 
																										   const std::vector<std::string>& plannerParamNamesToModify)
{
  for (auto i : plannerParamNamesToModify)
	{
		/*const std::string path = pathPlannerParameters+"/"+i; //to be put directly into rosparam set, just separated for debug purpose
    ros::param::set(path, paramSetCurrent[i]);*/

		ros::param::set(pathPlannerParameters+"/"+i, paramSetCurrent[i]);

 		/*//debug:
		XmlRpc::XmlRpcValue debugXmlRpc = getServerParameters(path);
		if (debugXmlRpc.getType() == XmlRpc::XmlRpcValue::TypeDouble)
			ROS_WARN( "[ON ROS SERVER] parameter '%s' has been updated to %s",
      			 		path.c_str(), std::to_string((double)debugXmlRpc).c_str() );
		else if (debugXmlRpc.getType() == XmlRpc::XmlRpcValue::TypeInt)
			ROS_WARN( "[ON ROS SERVER] parameter '%s' has been updated to %s",
      			 		path.c_str(), std::to_string((int)debugXmlRpc).c_str() );
		else
			ROS_ERROR( "[ON ROS SERVER] debugXmlRpc is neither an double nor an interger!!");*/
	}
}

void ModifiedBenchmarkExecutor::initializePlannerParameters(const std::string& pathPlannerParameters,
																														XmlRpc::XmlRpcValue& parametersBoundaries,
						      																					const std::vector<std::string>& plannerParamNamesToModify)
{ /*In fact as soon as we launch RViz the planner parameters from the dedicated .yaml file already has 
    its parameters assigned some values, however those initial values are retrieved from the 
    ompl_planning.yaml file, hence not necessarily belonging to the intervals of the exploration space
    we defined in planners_numerical_boundaries_andSteps.yaml. This then not only doesn't respect our
    space, but also create a super annoying mess into my matlab postprocess (precisely in the way 
    it handles the n axis associated to the n params) to show the evolution !*/
  for (auto i : plannerParamNamesToModify)
	{
		/*const std::string path = pathPlannerParameters+"/"+i; //to be put directly into rosparam set, just separated for debug purpose
    ros::param::set(path, parametersBoundaries[i]["min"]);*/

		ros::param::set(pathPlannerParameters+"/"+i, parametersBoundaries[i]["min"]);

		/*//debug:
		XmlRpc::XmlRpcValue debugXmlRpc = getServerParameters(path);
		if (debugXmlRpc.getType() == XmlRpc::XmlRpcValue::TypeDouble)
			ROS_WARN( "[ON ROS SERVER] parameter '%s' has been updated to %s",
      			 		path.c_str(), std::to_string((double)debugXmlRpc).c_str() );
		else if (debugXmlRpc.getType() == XmlRpc::XmlRpcValue::TypeInt)
			ROS_WARN( "[ON ROS SERVER] parameter '%s' has been updated to %s",
      			 		path.c_str(), std::to_string((int)debugXmlRpc).c_str() );
		else
			ROS_ERROR( "[ON ROS SERVER] debugXmlRpc is neither an double nor an interger!!");*/
	}
}


void ModifiedBenchmarkExecutor::writePlannerParametersAndQuality(XmlRpc::XmlRpcValue& paramSet, //params got from the ros server
																																 std::ofstream& fileVar,
																																 std::vector<std::string> plannerParamNames,
																																 int nbParams,
																																 double planQuality)
{
		for (std::size_t i = 0; i < nbParams; ++i)
		{
		  fileVar << paramSet[plannerParamNames[i]] << ", ";
		}
		fileVar << planQuality << ";" << std::endl;
}

/*TODO use typedef for e.g 
typedef std::map< std::string, XmlRpc::XmlRpcValue > String2XmlRpcValue; */

void ModifiedBenchmarkExecutor::alterPlannerParameters(XmlRpc::XmlRpcValue& 										   parametersSet_toUpdate, 
						  XmlRpc::XmlRpcValue& parametersBoundaries,
						  std::vector<std::string> plannerParamNames, 							  int nbParams, std::vector<std::string>& plannerParamNamesToModify)
// parametersBoundaries not const& because :
/*error: passing ‘const XmlRpc::XmlRpcValue’ as ‘this’ argument discards qualifiers [-fpermissive]
    parametersSet_toUpdate[plannerParamName] -= parametersBoundaries[plannerParamName]["step"]; */
// Though keeping still the &reference is dangerous but avoids to copy the dict...
{
  //Decide whether to make a move (towards a neighbour set of parameters) with respect to one of the n dims (n params), or to move along up to n dims
  double unirandom_d = std::rand()/((double)(RAND_MAX)+1.); // belongs to [0.,1.[
  												//(double)(RAND_MAX+1) implies "warning: integer overflow in expression"
	//https://stackoverflow.com/questions/2347851/c-a-cure-for-the-warning-integer-overflow-in-expression
  //ROS_WARN("(To verify) that unirandom_d = %f sequence isn't repeated through the very fast loops", unirandom_d); //TODO verify in another way than visually only!
  int moveBetween = floor(nbParams*unirandom_d+1); // a number of axis belonging to [1,nbParams]
  
  //Decide regarding which of the m<=n dims to move
  int draw;
  plannerParamNamesToModify.clear(); //this vector will tell which one of the params to update on the ROS server (different than on RAM!)
  std::string plannerParamName;
  for (unsigned i=0; i < moveBetween; ++i)
  { //begin routine "draw without replacement" (otherwise it's not anymore an infinitesimal move)
    draw = std::rand()%nbParams; // an index belonging to [0,nbParams-1]
    //paramNamesToAlter[i] = plannerParamNames[draw]; //actually not necessary
    plannerParamName = plannerParamNames[draw];
		plannerParamNamesToModify.push_back(plannerParamName);

    //interruption and insertion of the routine "do the alteration for that draw"
    if (parametersSet_toUpdate[plannerParamName].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      alterPlannerParameter<double>(parametersSet_toUpdate, parametersBoundaries, 
      				    plannerParamName);
    } else if (parametersSet_toUpdate[plannerParamName].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      alterPlannerParameter<int>(parametersSet_toUpdate, parametersBoundaries, plannerParamName);
    }
    
    //end of the routine "without replacement"
    plannerParamNames.erase(plannerParamNames.begin()+draw); //update of the remaining parameters to be choosable
    nbParams-=1; //update of the range where to pick a param indx in the vector of possible choices
  }
}

template <typename T>
void ModifiedBenchmarkExecutor::alterPlannerParameter(XmlRpc::XmlRpcValue& parametersSet_toUpdate, 
						      XmlRpc::XmlRpcValue& parametersBoundaries,
						      const std::string& plannerParamName)
{
  int coinflip;
  if ((T)parametersSet_toUpdate[plannerParamName] <= (T)parametersBoundaries[plannerParamName]["min"])
  {
    parametersSet_toUpdate[plannerParamName] = (T)parametersBoundaries[plannerParamName]["min"]+ 
    					     														 (T)parametersBoundaries[plannerParamName]["step"];
    /* unfortunately there isn't available overload for operator+ operator+= and I believe in fact for
    any operator but {operator= and operator==} */
  } else if ( (T)parametersSet_toUpdate[plannerParamName] >= (T)parametersBoundaries[plannerParamName]["max"] )
  {
    parametersSet_toUpdate[plannerParamName] = (T)parametersBoundaries[plannerParamName]["max"] - 
    																					 (T)parametersBoundaries[plannerParamName]["step"];
  } else
  {
    coinflip = std::rand()%2; // an index being 0 or 1
    switch(coinflip)
    {
      case 0 : ROS_WARN( "[For completeness] parameter '%s' has been updated from %s",
      			 						 plannerParamName.c_str(), 
      			 						 std::to_string((T)parametersSet_toUpdate[plannerParamName]).c_str() );
      	       parametersSet_toUpdate[plannerParamName] = (T)parametersSet_toUpdate 
      								[plannerParamName] + (T)parametersBoundaries[plannerParamName]["step"];
      	       ROS_WARN( "to %s", 
      	       		 			 std::to_string((T)parametersSet_toUpdate[plannerParamName]).c_str() );
	       			 break;
      case 1 : ROS_WARN( "[For completeness] parameter '%s' has been updated from %s",
      			 						 plannerParamName.c_str(), 
      			 						 std::to_string((T)parametersSet_toUpdate[plannerParamName]).c_str() );
      	       parametersSet_toUpdate[plannerParamName] = (T)parametersSet_toUpdate 
      								[plannerParamName] - (T)parametersBoundaries[plannerParamName]["step"];
      	       ROS_WARN( "to %s", 
      	       		 			 std::to_string((T)parametersSet_toUpdate[plannerParamName]).c_str() );
    }
    if ((T)parametersSet_toUpdate[plannerParamName] < 0)
      ROS_ERROR("LLLLLLLLLLLLLLLLLOOOOOOOOOOOOOOOOOOOOOOKKKKKKKKKKKKKKKKKKKK"); //TODO investigate
  }
}

bool ModifiedBenchmarkExecutor::accepted(double t, double Tmax)
{ //Here we could also have chosen to input the worse quality than the previous, but since I cannot guarantee the maximum (being 1) to be ever obtainable in presence of obstacles in the scene, let's do it in another way, "more gentle" 
  double unirandom_d = std::rand()/((double)(RAND_MAX)+1.); // belongs to [0.,1.[
  if (unirandom_d > t/Tmax) //.EQV. to: acceptance(t)|T = (1-t/T)*100 %of chances
    return true;
  return false;
} //c.f Simulated Annealing algorithm, although choice for acceptance func is totally free

    /* For laterly alter the joint_projections parameter as well:
    const 		& getRobotActuatedJoints() // I use a hack, which assumes that have any of these: joint and/or velocity and/or acceleration limits, i.e that the topic /robot_description_planning/joint_limits/ exists. Currently this is the only one available which shows the robot joints //TODO Laterly read in somewhere stable, like the .urdf. //TODO Find where to read the joint which stands as end effector (where the ball marker is on, in RViz)
    {
      XmlRpc::XmlRpcValue robot_joints_XmlRpc;
      if (ros::param::get("robot_description_planning/joint_limits", planner_parameters_XmlRpc)) 
      {
        return robot_joints_XmlRpc.structFromXml()
      }
      else
        ROS_WARN("No robot_description_planning/joint_limits found on param server. Type 'rosparam list' in the console to see the paths. The optimization process continue, though it won't smartly tweak its parameters at restarts!!");
    }*/

double ModifiedBenchmarkExecutor::evaluate_plan(const robot_trajectory::RobotTrajectory& p) // kindof energy consumption
{
  int num_of_joints = p.getWayPoint(0).getVariableCount();
  const double pi = boost::math::constants::pi<double>();

  // Joints near the shoulder consume more than those near the EE
  std::vector<int> weights(num_of_joints, 0);
  for(int k = 0; k<num_of_joints; k++){
    weights[k] = num_of_joints - k;
  } //TODO generate it from outside both the evaluate_plan and the run benchmark recorded in time (to speed up)
  
  std::vector<std::vector <double> > plan_array (p.getWayPointCount(), std::vector<double>(num_of_joints));
  for (size_t i = 0 ; i < p.getWayPointCount() ; ++i){
    for (size_t j = 0 ; j < num_of_joints ; ++j){
      plan_array[i][j] = p.getWayPoint(i).getVariablePositions()[j];
    }
  }

  std::vector<std::vector <double> > deltas (p.getWayPointCount()-1, std::vector<double>(num_of_joints));
  for (size_t i = 0 ; i < p.getWayPointCount()-1 ; ++i){
    for (size_t j = 0 ; j < num_of_joints ; ++j){
      deltas[i][j] = plan_array[i+1][j] - plan_array[i][j];
      if(deltas[i][j] < 0) // abs() only works for integers. We can also use fabs() from math.h
				deltas[i][j] = - deltas[i][j];
    }
  }

  std::vector<double> sum_deltas(num_of_joints, 0);
  for (size_t i = 0 ; i < p.getWayPointCount()-1 ; ++i){
    for (size_t j = 0 ; j < num_of_joints ; ++j){
      sum_deltas[j] += deltas[i][j];
    }
  }

  std::vector<double> sum_deltas_weighted(num_of_joints, 0);
  for (size_t j = 0 ; j < num_of_joints ; ++j){
    sum_deltas_weighted[j] = sum_deltas[j] * weights[j];
  }

  double denominator = 0.0;
  for (auto it = sum_deltas_weighted.begin() ; it != sum_deltas_weighted.end(); ++it){
    denominator += *it;
  }

  // https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
  std::vector<double> shortest_diff_goal_start_confs(num_of_joints, 0); // in fact assuming no obstacle that comes modify the shortest trajectory
  for (size_t j = 0 ; j < num_of_joints ; ++j){
    shortest_diff_goal_start_confs[j] = plan_array[p.getWayPointCount()-1][j]-plan_array[0][j]; // but by drawing the angles goal=0.25*pi and start=1.5*pi on a trigonometric circle, abs(goal-start)=1.25=(1+1/4)*pi, which is not the shortest move if one colors the area! Hence the use of a signed angle (i.e which can be negative) ... and the absolute values of the 1-norm later.
    if (shortest_diff_goal_start_confs[j] > pi)
      shortest_diff_goal_start_confs[j] -= 2*pi;
    if (shortest_diff_goal_start_confs[j] <= -pi)
      shortest_diff_goal_start_confs[j] += 2*pi;
  }

  std::vector<double> shortest_diff_weighted(num_of_joints, 0);
  for (size_t j = 0 ; j < num_of_joints ; ++j){
    shortest_diff_weighted[j] = shortest_diff_goal_start_confs[j] * weights[j];
  }

  // the 1-norm
  double numerator = 0.0;
  for (auto it = shortest_diff_weighted.begin() ; it != shortest_diff_weighted.end(); ++it){
    numerator += fabs(*it);
  }

  double plan_quality = numerator/denominator;

  return plan_quality;
}

double ModifiedBenchmarkExecutor::evaluate_plan_cart(const robot_trajectory::RobotTrajectory& p) // kindof how far the actual trajectory is from the end-effector shortest trajectory
{
  int n = p.getWayPointCount();

  std::vector<geometry_msgs::Transform> transforms (n);
  for (size_t i = 0 ; i < n; ++i)
  {
    moveit::core::RobotState goal_state = p.getWayPoint(i); // pos vel accel and effort on the end effector

    const moveit::core::JointModel* joint_eef = goal_state.getJointModel(goal_state.getVariableNames()[goal_state.getVariableCount()-1]); // parent child jointType value

    std::string link_eef = joint_eef->getChildLinkModel()->getName();

    const Eigen::Affine3d& link_pose = goal_state.getGlobalLinkTransform(link_eef); // matrix 4x4

    tf::transformEigenToMsg(link_pose, transforms[i]); // http://docs.ros.org/kinetic/api/eigen_conversions/html/eigen__msg_8cpp_source.html#l00093 their if() occurs when theta>pi or theta<-pi, because then cos(theta/2) becomes <0 (draw a circle to convince yourself). This means that QUATERNION ROTATIONS ARE COMPRISED ONLY BETWEEN [-PI;PI] ! This allows then the use of 2*atan2(scalar part) to find back an associated rotation angle, because it is known that atan2 has an output comprised only between [-pi/2;pi/2] !
// atan2 is better than acos because, if quaternion rotations are in [-pi;pi], their scalar component belongs to [0;1] only, and hence 2acos(this component) belongs to 2[0;pi]=[0;2pi], while 2acos(this component) should belong to [-pi;pi] to respect the convention Eigen:: used!
// But to use atan2 the vector part of the quaternion must be normalized https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Recovering_the_axis-angle_representation (leading btw the whole quaternion to be normalized as well). I am STILL NOT SURE how Eigen constructs a quaternion from a 3x3 rotation matrix, but I assume it uses this algorithm https://arc.aiaa.org/doi/pdf/10.2514/2.4654, in which we can read (1st page) : <<  we are looking for that quaternion q of unit length >>.

  }

  int m = n-1;

  double eef_dist = 0.0;
  double eef_rot  = 0.0;
  
  for(size_t i = 0; i<m; ++i)
  {
    double x, y, z;//, w;
    tf2::Quaternion q1, q2, q1inv, qR;
    x = transforms[i+1].translation.x - transforms[i].translation.x;
    y = transforms[i+1].translation.y - transforms[i].translation.y;
    z = transforms[i+1].translation.z - transforms[i].translation.z;
    tf2::fromMsg(transforms[i+1].rotation,q2); // http://docs.ros.org/kinetic/api/tf2_geometry_msgs/html/c++/namespacetf2.html#a2fdf91676912e510c0002aa66dde2800 (q2 = rotation from an initial frame0 to a frame2)
    tf2::fromMsg(transforms[i  ].rotation,q1); // q1 = other rotation from the same initial frame0 to another frame1
// Then the rotation qRelative that transforms frame1 into frame2 is such that (thinking the way matrices work by multiplication) q2 = qR*q1, hence qR = q2*inv(q1)
// Assuming that a rotation quaternion is a unit quaternion, inv(q1) simply equals conj(q1)
// By the way tf2:: seems aware of this property and the source code of inv() behaves like a conj so perhaps that all the quaternions in tf2 are already normalized, but just to be sure :
    q1.normalize();
    q2.normalize();
    q1inv = q1.inverse();
    qR = q2*q1inv;
    eef_dist += sqrt(x*x+y*y+z*z);
    eef_rot += fabs(2*atan2(sqrt(pow(qR.x(),2)+pow(qR.y(),2)+pow(qR.z(),2)),qR.w())); // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Recovering_the_axis-angle_representation

  }

  double x_t, y_t, z_t, w_t;
  double tot_dist, tot_rot;
  tf2::Quaternion qLast, q0, q0inv, qRtot;
  x_t = transforms[m].translation.x - transforms[0].translation.x;
  y_t = transforms[m].translation.y - transforms[0].translation.y;
  z_t = transforms[m].translation.z - transforms[0].translation.z;
  tf2::fromMsg(transforms[m].rotation,qLast);
  tf2::fromMsg(transforms[0].rotation,q0);
  qLast.normalize();
  q0.normalize();
  q0inv = q0.inverse();
  qRtot = qLast*q0inv;
  
  tot_dist = sqrt(x_t*x_t+y_t*y_t+z_t*z_t);
  tot_rot = fabs(2*atan2(sqrt(pow(qRtot.x(),2)+pow(qRtot.y(),2)+pow(qRtot.z(),2)),qRtot.w()));
  
  double quality = 0.0, quality_cart;

  if(eef_dist > 0.001)
    quality += tot_dist/eef_dist;
  else
    quality += 1;

  if(tot_rot  > 0.001)
    quality += tot_rot/eef_rot;
  else
    quality += 1;
  
  quality_cart = quality/2;
  
  return quality_cart;

}

void ModifiedBenchmarkExecutor::collectMetrics(PlannerRunData& metrics,
                                       const planning_interface::MotionPlanDetailedResponse& mp_res, bool solved,
                                       double total_time)
{
  metrics["time REAL"] = moveit::core::toString(total_time);
  metrics["solved BOOLEAN"] = boost::lexical_cast<std::string>(solved);

  if (solved)
  {
    // Analyzing the trajectory(ies) geometrically
    double L = 0.0;           // trajectory length
    double clearance = 0.0;   // trajectory clearance (average)
    double smoothness = 0.0;  // trajectory smoothness (average)
    bool correct = true;      // entire trajectory collision free and in bounds
    double planQuality = 0.0; // trajectory quality (added attribute)
    double planQualityCart = 0.0;

    double process_time = total_time;
    for (std::size_t j = 0; j < mp_res.trajectory_.size(); ++j)
    {
      correct = true;
      L = 0.0;
      clearance = 0.0;
      smoothness = 0.0;
      const robot_trajectory::RobotTrajectory& p = *mp_res.trajectory_[j];

      // compute plan quality
      planQuality = evaluate_plan(p);
      planQualityCart = evaluate_plan_cart(p);
      
      // compute path length
      for (std::size_t k = 1; k < p.getWayPointCount(); ++k)
        L += p.getWayPoint(k - 1).distance(p.getWayPoint(k));

      // compute correctness and clearance
      collision_detection::CollisionRequest req;
      for (std::size_t k = 0; k < p.getWayPointCount(); ++k)
      {
        collision_detection::CollisionResult res;
        planning_scene_->checkCollisionUnpadded(req, res, p.getWayPoint(k));
        if (res.collision)
          correct = false;
        if (!p.getWayPoint(k).satisfiesBounds())
          correct = false;
        double d = planning_scene_->distanceToCollisionUnpadded(p.getWayPoint(k));
        if (d > 0.0)  // in case of collision, distance is negative
          clearance += d;
      }
      clearance /= (double)p.getWayPointCount();

      // compute smoothness
      if (p.getWayPointCount() > 2)
      {
        double a = p.getWayPoint(0).distance(p.getWayPoint(1));
        for (std::size_t k = 2; k < p.getWayPointCount(); ++k)
        {
          // view the path as a sequence of segments, and look at the triangles it forms:
          //          s1
          //          /\          s4
          //      a  /  \ b       |
          //        /    \        |
          //       /......\_______|
          //     s0    c   s2     s3
          //

          // use Pythagoras generalized theorem to find the cos of the angle between segments a and b
          double b = p.getWayPoint(k - 1).distance(p.getWayPoint(k));
          double cdist = p.getWayPoint(k - 2).distance(p.getWayPoint(k));
          double acosValue = (a * a + b * b - cdist * cdist) / (2.0 * a * b);
          if (acosValue > -1.0 && acosValue < 1.0)
          {
            // the smoothness is actually the outside angle of the one we compute
            double angle = (boost::math::constants::pi<double>() - acos(acosValue));

            // and we normalize by the length of the segments
            double u = 2.0 * angle;  /// (a + b);
            smoothness += u * u;
          }
          a = b;
        }
        smoothness /= (double)p.getWayPointCount();
      }
      metrics["path_" + mp_res.description_[j] + "_correct BOOLEAN"] = boost::lexical_cast<std::string>(correct);
      metrics["path_" + mp_res.description_[j] + "_length REAL"] = boost::lexical_cast<std::string>(L);
      metrics["path_" + mp_res.description_[j] + "_clearance REAL"] = boost::lexical_cast<std::string>(clearance);
      metrics["path_" + mp_res.description_[j] + "_plan_quality REAL"] = boost::lexical_cast<std::string>(planQuality);
      metrics["path_" + mp_res.description_[j] + "_plan_quality_cartesian REAL"] = boost::lexical_cast<std::string>(planQualityCart);
      metrics["path_" + mp_res.description_[j] + "_smoothness REAL"] = boost::lexical_cast<std::string>(smoothness);
      metrics["path_" + mp_res.description_[j] + "_time REAL"] =
          boost::lexical_cast<std::string>(mp_res.processing_time_[j]);
      process_time -= mp_res.processing_time_[j];
    }
    if (process_time <= 0.0)
      process_time = 0.0;
    metrics["process_time REAL"] = moveit::core::toString(process_time);
  }
}

void ModifiedBenchmarkExecutor::writeOutput(const BenchmarkRequest& brequest, const std::string& start_time,
                                    double benchmark_duration)
{
  const std::map<std::string, std::vector<std::string>>& planners = options_.getPlannerConfigurations();

  size_t num_planners = 0;
  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end();
       ++it)
    num_planners += it->second.size();

  std::string hostname = getHostname();
  if (hostname.empty())
    hostname = "UNKNOWN";

  std::string filename = options_.getOutputDirectory();
  if (filename.size() && filename[filename.size() - 1] != '/')
    filename.append("/");

  // Ensure directories exist
  boost::filesystem::create_directories(filename);

  filename += (options_.getBenchmarkName().empty() ? "" : options_.getBenchmarkName() + "_") + brequest.name + "_" +
              getHostname() + "_" + start_time + ".log";
  std::ofstream out(filename.c_str());
  if (!out)
  {
    ROS_ERROR("Failed to open '%s' for benchmark output", filename.c_str());
    return;
  }

  out << "MoveIt! version " << MOVEIT_VERSION << std::endl;
  out << "Experiment " << brequest.name << std::endl;
  out << "Running on " << hostname << std::endl;
  out << "Starting at " << start_time << std::endl;

  // Experiment setup
  moveit_msgs::PlanningScene scene_msg;
  planning_scene_->getPlanningSceneMsg(scene_msg);
  out << "<<<|" << std::endl;
  out << "Motion plan request:" << std::endl << brequest.request << std::endl;
  out << "Planning scene: " << std::endl << scene_msg << std::endl << "|>>>" << std::endl;

  // Not writing optional cpu information

  // The real random seed is unknown.  Writing a fake value
  out << "0 is the random seed" << std::endl;
  out << brequest.request.allowed_planning_time << " seconds per run" << std::endl;
  // There is no memory cap
  out << "-1 MB per run" << std::endl;
  out << options_.getNumRuns() << " runs per planner" << std::endl;
  out << benchmark_duration << " seconds spent to collect the data" << std::endl;

  // No enum types
  out << "0 enum types" << std::endl;

  out << num_planners << " planners" << std::endl;

  size_t run_id = 0;
  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end();
       ++it)
  {
    for (std::size_t i = 0; i < it->second.size(); ++i, ++run_id)
    {
      // Write the name of the planner.
      out << it->second[i] << std::endl;

      // in general, we could have properties specific for a planner;
      // right now, we do not include such properties
      out << "0 common properties" << std::endl;

      // Create a list of the benchmark properties for this planner
      std::set<std::string> properties_set;
      for (std::size_t j = 0; j < benchmark_data_[run_id].size(); ++j)  // each run of this planner
        for (PlannerRunData::const_iterator pit = benchmark_data_[run_id][j].begin();
             pit != benchmark_data_[run_id][j].end(); ++pit)  // each benchmark property of the given run
          properties_set.insert(pit->first);

      // Writing property list
      out << properties_set.size() << " properties for each run" << std::endl;
      for (std::set<std::string>::const_iterator pit = properties_set.begin(); pit != properties_set.end(); ++pit)
        out << *pit << std::endl;

      // Number of runs
      out << benchmark_data_[run_id].size() << " runs" << std::endl;

      // And the benchmark properties
      for (std::size_t j = 0; j < benchmark_data_[run_id].size(); ++j)  // each run of this planner
      {
        // Write out properties in the order we listed them above
        for (std::set<std::string>::const_iterator pit = properties_set.begin(); pit != properties_set.end(); ++pit)
        {
          // Make sure this run has this property
          PlannerRunData::const_iterator runit = benchmark_data_[run_id][j].find(*pit);
          if (runit != benchmark_data_[run_id][j].end())
            out << runit->second;
          out << "; ";
        }
        out << std::endl;  // end of the run
      }
      out << "." << std::endl;  // end the planner
    }
  }

  out.close();
  ROS_INFO("Benchmark results saved to '%s'", filename.c_str());
}

int ModifiedBenchmarkExecutor::xmlRpcValueSize(XmlRpc::XmlRpcValue& someXmlSet) //(const& generates error)
{
  int length = 0;
  /*try
  {*/
  for (XmlRpc::XmlRpcValue::iterator it = someXmlSet.begin() ; it != someXmlSet.end(); ++it)
      length+=1;
  /*}
  catch (XmlRpc::XmlRpcException& e)
  {
      ROS_WARN(e.getMessage().c_str());
      exit(-1);
  }*/
  return length;
} /* other way to investigate
for(XmlRpc::XmlRpcValue whatever : input_xmlrpc_struct){
whatever.getType.....
}
*/

std::map<std::string, std::vector<std::string>> ModifiedBenchmarkExecutor::constructMoveitPlannersParameterNamesDictionnary()
{ //This has for purpose to list all the CURRENT planners implemented in moveit and their TWEAKABLE default parameters (basically, "type" parameter = geometric::... is not tweakable into control:: currently) //TODO currently I did not deal yet with the planners using projection evaluator //TODO retrieve this from a .yaml instead
  std::map<std::string, std::vector<std::string>> map_by_value_08Dec18;
  map_by_value_08Dec18["RRTConnectkConfigDefault"] = {"range"};
  map_by_value_08Dec18["ESTkConfigDefault"] = {"range", 
  					       "goal_bias"};
  map_by_value_08Dec18["RRTkConfigDefault"] = {"range", 
  				               "goal_bias"};
  map_by_value_08Dec18["KPIECEkConfigDefault"] = {"range", 
  					          "goal_bias",
  					          "border_fraction", 
  					          "failed_expansion_score_factor", 
  					          "min_valid_path_fraction", 
  					          "projection_evaluator", 
  					          "longest_valid_segment_fraction"};
  map_by_value_08Dec18["SBLkConfigDefault"] = {"range", 
  					       "projection_evaluator", 
  					       "longest_valid_segment_fraction"};
  map_by_value_08Dec18["LBKPIECEkConfigDefault"] = {"range",
				    	    	    "border_fraction",
				    	            "min_valid_path_fraction",
				    	            "projection_evaluator",
				    	            "longest_valid_segment_fraction"};
  map_by_value_08Dec18["BKPIECEkConfigDefault"] = {"range",
  					           "border_fraction", 
  					           "failed_expansion_score_factor", 
  					           "min_valid_path_fraction", 
  					           "projection_evaluator", 
  					           "longest_valid_segment_fraction"};
  map_by_value_08Dec18["RRTstarkConfigDefault"] = {"range", 
  					           "goal_bias", 
  					           "delay_collision_checking"};
  map_by_value_08Dec18["TRRTkConfigDefault"] = {"range", 
  					        "goal_bias", 
				                "max_states_failed", 
  					        "temp_change_factor", 
  					        "min_temperature", 
  					        "init_temperature", 
  					        "frountier_threshold", 
  					        "frountierNodeRatio", 
  					        "k_constant"};
  map_by_value_08Dec18["PRMkConfigDefault"] = {"max_nearest_neighbors"};
  map_by_value_08Dec18["PRMstarkConfigDefault"] = {}; //empty vector
  return map_by_value_08Dec18;
}

//What's below is C language and all the following functions stand for one purpose : remove the last line of a given file
/* File must be open with 'b' (for binary) in the mode parameter to fopen() */
long ModifiedBenchmarkExecutor::fsize(FILE *binaryStream) //2 million lines maximum
{
  long ofs, ofs2;
  int result;

  // if cursor isn't placed at beginning (0 octets), return error (return something not 0)
  if (fseek(binaryStream, 0, SEEK_SET) != 0 ||
      fgetc(binaryStream) == EOF)
    return 0;

  ofs = 1;

  //while cursot not at end of file, and cursor is well puted on ofs octets from the file beginning, read one char in result
  while ((result = fseek(binaryStream, ofs, SEEK_SET)) == 0 &&
         (result = (fgetc(binaryStream) == EOF)) == 0 &&
         ofs <= LONG_MAX / 4 + 1)
    ofs *= 2;

  /* If the last seek failed, back up to the last successfully seekable offset */
  if (result != 0)
    ofs /= 2;

  for (ofs2 = ofs / 2; ofs2 != 0; ofs2 /= 2)
    if (fseek(binaryStream, ofs + ofs2, SEEK_SET) == 0 &&
        fgetc(binaryStream) != EOF)
      ofs += ofs2;

  /* Return -1 for files longer than LONG_MAX */
  if (ofs == LONG_MAX)
    return -1;

  return ofs + 1;
}

/* File must be open with 'b' in the mode parameter to fopen() */
/* Set file position to size of file before reading last line of file */
char* ModifiedBenchmarkExecutor::getOffsetBeforeLastBuf(char *buf, int n, FILE *binaryStream, off_t& offset)
{ /* and returns the last buf after this position setting, as well as initiates/modifies the value of that position setting = offset */
  long fpos;
  int cpos;

  /* when calling ftell, tells the current value of the position indicator.
  If an error occurs, -1L is returned, and the global variable errno is set to a positive value*/
  if (n <= 1 || (fpos = ftell(binaryStream)) == -1 || fpos == 0)
    return NULL;

  cpos = n - 1;
  buf[cpos] = '\0'; /* The length of a C string (an array containing the characters and terminated with a '\0' character)
  is found by searching for the (first) NUL byte*/

  int second = 0; //flag to catch only the second '\n' char starting from the end (native booleans don't exist in C)
  for (;;) //infinite loop I guess
  {
    int c;

    if (fseek(binaryStream, --fpos, SEEK_SET) != 0 || //failed to move back the cursor of one position unit
        (c = fgetc(binaryStream)) == EOF)
      return NULL;

    if (c == '\n' && second == 1) /* accept at most one '\n' */
      break;
    second = 1;

    /* I believe this block may stand for DOS/Windows where newlines are encoded with both '\n'+'\r' as well,
    due to History where the handcraft printer machines used to \n to scroll down and \r to come back to the extreme left */
    if (c != '\r') //So I think it's always yes under linux/posix
    {
      unsigned char ch = c;
      if (cpos == 0)
      {
        memmove(buf + 1, buf, n - 2);
        ++cpos;
      }
      memcpy(buf + --cpos, &ch, 1); //copy 1 octets from ch to buf so I guess it fills buf from last to first char in the line
    } else
    {
      /*//exploration
      printf("NO\n");*/
    }

    if (fpos == 0)
    {
      fseek(binaryStream, 0, SEEK_SET);
      break;
    }
  }

  memmove(buf, buf + cpos, n - cpos); /* I think it finally moves buf from cpos, which from the beginning decreased to an
  offset lower than n (=256 in this current case), forgetting every empty char before, to the new buf that will be filled with
  only what is necessary (unempty chars)*/

  offset = n-cpos;
  return buf;
}

void ModifiedBenchmarkExecutor::deleteLastLine(const char *filePathPtr, unsigned int bufLengthMax)
{
  off_t globaloffset, localoffset;
  FILE* f;
  if ((f = fopen(filePathPtr, "rb")) == NULL)
  {
    printf("failed to open file \'%s\'\n", filePathPtr);
    return;
  }

  long sz = fsize(f);
  if (sz > 0)
  {
    char buf[bufLengthMax]; //LENGTH MAX OF A LINE I GUESS
    fseek(f, sz, SEEK_SET); //place the cursor after the last char of the last line
    if ( getOffsetBeforeLastBuf(buf, sizeof(buf), f, localoffset) != NULL )
    {
      if (truncate( filePathPtr, (globaloffset = sz - localoffset) ) != 0);
      {
        //printf("Deletion of the exceeding countdown measure writing went wrong!\n");
        /* One mystery that remains is that it does go wrong and prints this above commented print, even though it does the job...*/
      }

      //printf("%s", buf);
    }
    else
    {
      printf("Retrieved a last line of null buffer length!\n");
    }
  }

  fclose(f);
}
