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

#include <boost/regex.hpp> // To search for letters or words
#include <boost/progress.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp> // seconds since January 1st of 1970
#include <unistd.h>

#include <math.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>


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

ModifiedBenchmarkExecutor::ModifiedBenchmarkExecutor(const std::string& robot_description_param)
{
  pss_ = NULL;
  psws_ = NULL;
  rs_ = NULL;
  cs_ = NULL;
  tcs_ = NULL;
  psm_ = new planning_scene_monitor::PlanningSceneMonitor(robot_description_param);
  planning_scene_ = psm_->getPlanningScene(); // pointer

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
  delete psm_;
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
  if (planner_interfaces_.size() == 0)
  {
    ROS_ERROR("No planning interfaces configured.  Did you call ModifiedBenchmarkExecutor::initialize?");
    return false;
  }

  std::vector<BenchmarkRequest> queries;
  moveit_msgs::PlanningScene scene_msg;

  if (initializeBenchmarks(opts, scene_msg, queries))
  {
    if (!queriesAndPlannersCompatible(queries, opts.getPlannerConfigurations()))
      return false;

    for (std::size_t i = 0; i < queries.size(); ++i)
    {
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
      for (std::size_t j = 0; j < query_start_fns_.size(); ++j)
        query_start_fns_[j](queries[i].request, planning_scene_);

      ROS_INFO("Benchmarking query '%s' (%lu of %lu)", queries[i].name.c_str(), i + 1, queries.size()); // (Note that queries are benchmarked in disorder)
      ros::WallTime start_time = ros::WallTime::now();
      runBenchmark(queries[i].request, options_.getPlannerConfigurations(), options_.getNumRuns(),
      		   options_.getMetricChoice());
      double duration = (ros::WallTime::now() - start_time).toSec();
      
      ROS_INFO("Benchmark took '%f' seconds.", duration);

      for (std::size_t j = 0; j < query_end_fns_.size(); ++j)
        query_end_fns_[j](queries[i].request, planning_scene_);

      writeOutput(queries[i], boost::posix_time::to_iso_extended_string(start_time.toBoost()), duration);
    }

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

bool ModifiedBenchmarkExecutor::initializeBenchmarks(const ModifiedBenchmarkOptions& opts, moveit_msgs::PlanningScene& scene_msg,
                                             std::vector<BenchmarkRequest>& requests)
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

  ROS_INFO("Benchmark loaded %lu starts, %lu goals, %lu path constraints, %lu trajectory constraints, and %lu queries",
           start_states.size(), goal_constraints.size(), path_constraints.size(), traj_constraints.size(),
           queries.size());

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
    requests.insert(requests.end(), request_combos.begin(), request_combos.end());
  }

  // 2) Existing queries are treated like goal constraints.
  //    Create all combos of query, start states, and path constraints
  for (std::size_t i = 0; i < queries.size(); ++i)
  {
    // Common benchmark request properties
    BenchmarkRequest brequest;
    brequest.name = queries[i].name;
    brequest.request = queries[i].request;
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
    createRequestCombinations(brequest, start_states, path_constraints, request_combos);
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
    requests.insert(requests.end(), request_combos.begin(), request_combos.end());
  }

  options_ = opts;
  return true;
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
    ROS_ERROR("Error loading motion planning queries: %s", ex.what());
    return false;
  }

  if (query_names.empty())
  {
    ROS_ERROR("Scene '%s' has no associated queries", scene_name.c_str());
    return false;
  }

  for (std::size_t i = 0; i < query_names.size(); ++i)
  {
    moveit_warehouse::MotionPlanRequestWithMetadata planning_query;
    try
    {
      pss_->getPlanningQuery(planning_query, scene_name, query_names[i]);
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("Error loading motion planning query '%s': %s", query_names[i].c_str(), ex.what());
      continue;
    }

    BenchmarkRequest query;
    query.name = query_names[i];
    query.request = static_cast<moveit_msgs::MotionPlanRequest>(*planning_query);
    queries.push_back(query);
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

////static	      getRobotActuatedJoints();

void ModifiedBenchmarkExecutor::runBenchmark(moveit_msgs::MotionPlanRequest request,
                                     const std::map<
                                     std::string, std::vector<std::string> >& planners, 
                                     int runs, 
                                     const std::string& metricChoice)
{
  benchmark_data_.clear();

  double countdown = request.allowed_planning_time;
  ROS_INFO("I ensure that countdown is in seconds -- countdown = '%lf'", countdown);

  unsigned int num_planners = 0;
  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end();
       ++it)
    num_planners += it->second.size();

  // Iterate through all planner plugins
  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end();
       ++it)
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

      planning_interface::PlanningContextPtr context =
          planner_interfaces_[it->first]->getPlanningContext(planning_scene_, request);
      for (int j = 0; j < runs; ++j)
      {
        // Pre-run events
        for (std::size_t k = 0; k < pre_event_fns_.size(); ++k)
          pre_event_fns_[k](request);

      	// Modification of the initial code : addition of a while countdown not exceeded -- tweak.
      	planning_interface::MotionPlanDetailedResponse mp_res;
      	planning_interface::MotionPlanDetailedResponse mp_res_before_exceeding;
      	double total_time = 0.;
      	bool solved = true; //In case the last mp_res before exceed is not solved but a previous is
      	int solved_proof = 0;
      	bool finally_solved = false;
      	mp_res_before_exceeding = mp_res; // In case the first iteration isn't kept bc too long
      	
      	// https://www.learncpp.com/cpp-tutorial/78-function-pointers/
      	double (*qualityFcnPtr)(const robot_trajectory::RobotTrajectory&); // pointer which points to the chosen metric function during all the following, even though one could alternate between metrics...
      	std::string metric1 ("energy");
      	std::string metric2 ("relevancy");
      	if (metricChoice.compare(metric1)==0)
      	{
      	  qualityFcnPtr = &evaluate_plan;
      	  ROS_INFO("The chosen metric, over which optimization will be done, is set on : '%s'. Currently you can change it by acting on iplanr_description/benchmark_configs/scene_ground_with_boxes.yaml", metric1.c_str());
      	  goto jmp; // a hack to break out of if statements
      	} else if (metricChoice.compare(metric2)==0)
      	{
      	  qualityFcnPtr = &evaluate_plan_cart;
      	  ROS_INFO("The chosen metric, over which optimization will be done, is set on : '%s'. Currently you can change it by acting on iplanr_description/benchmark_configs/scene_ground_with_boxes.yaml", metric2.c_str());
      	  goto jmp;
      	} else
      	{
      	  ROS_ERROR("In iplanr_description/benchmark_configs/scene_ground_with_boxes.yaml, you did not set any metric over which do the optimization process. In parameters list, please add metric_choice: relevancy OR energy");
      	}
      	jmp:
      	double planQuality, previousPlanQuality = planQuality;
      	
      	// Offline acquisition of the planner's parameters from the server in order to tweak them
      	XmlRpc::XmlRpcValue parametersSet = getPlannerParameters(planners.begin()->second[0]); // planners of type {plugin1_name: ["planner1_name", "planner2_name"], plugin2_name: ["planner1_name]}
      	
      	// Solve problem, once, as before
      	ros::WallTime start = ros::WallTime::now();
      	solved = context->solve(mp_res); //github.com/ros-planning/moveit/issues/1230
      	if (solved)
      	{
      	  solved_proof +=1;
      	  planQuality = (*qualityFcnPtr)(*mp_res.trajectory_[0]); // HOW/WHY can it exists several found trajectories ? (why do I have to retrieve only the first [0] of them?)
  	  ROS_INFO("Current quality = %lf out of 1", planQuality); // TO BE REMOVED
      	}
      	total_time += (ros::WallTime::now() - start).toSec();
      	
      	// and as many times as the countdown allows it,
      	while (total_time < countdown)
      	{
      	  ros::WallTime start = ros::WallTime::now();
      	  if (solved)
      	  { //(We save the previous iteration only if it solved the pb and allocated time remains.)
      	    mp_res_before_exceeding = mp_res; // addition to the first iteration above
      	    previousPlanQuality = planQuality;
      	  }
      	  // while tweaking the planner's parameters,
      	  // ...
      	  
      	  solved = context->solve(mp_res);
      	  if (solved)
      	  {
      	    solved_proof +=1;
      	    // and while comparing the qualities as well to see if these tweaks lead to improvment.
      	    planQuality = (*qualityFcnPtr)(*mp_res.trajectory_[0]); // HOW/WHY can it exists several found trajectories ? (why do I have to retrieve only the first [0] of them?)
      	    if (planQuality <= previousPlanQuality) //switch back to the previous solution
      	    {
      	      mp_res = mp_res_before_exceeding;
      	      ROS_INFO("Current quality = %lf out of 1", planQuality); // TO BE REMOVED
      	    }
      	  }
      	  total_time += (ros::WallTime::now() - start).toSec();
      	}
      	
      	if (solved_proof > 0)
      	{
      	  finally_solved = true;
      	  ROS_INFO("It exists some iteration which has managed to solve the problem. And the one which best solves it, is stored");
      	}

        // Post-run events
        for (std::size_t k = 0; k < post_event_fns_.size(); ++k)
          post_event_fns_[k](request, mp_res_before_exceeding, planner_data[j]);
          
        // Collect data
        start = ros::WallTime::now();
        collectMetrics(planner_data[j], mp_res_before_exceeding, finally_solved, total_time);
        double metrics_time = (ros::WallTime::now() - start).toSec();
        ROS_INFO("Spent %lf seconds collecting metrics", metrics_time);
      }

      // Planner completion events
      for (std::size_t j = 0; j < planner_completion_fns_.size(); ++j)
        planner_completion_fns_[j](request, planner_data);

      benchmark_data_.push_back(planner_data);
    }
  }
}

XmlRpc::XmlRpcValue ModifiedBenchmarkExecutor::getPlannerParameters(const std::string& planner) // the output type is for debug purpose, I want in the end to return only one planner's set of parameters, and in a struct or array type!!
{
  //XmlRpc::XmlRpcValue planner_parameters_XmlRpc;
  int offset = 2; // just to test
  int* offsetPtr = &offset;
  if (ros::param::get("planner_configs/"+planner, *this)) // http://docs.ros.org/kinetic/api/roscpp/html/namespaceros_1_1param.html#a8946be052ed53e5e243dbd0c9bb23b8a
  // TODO && check that, apart from the type parameter, it exists parameters to tweak (PRMstarkConfigDefault creates an exception, see ompl_planning.yaml)
  {
    return this->XmlRpcValue::structFromXml("string test", offsetPtr);
  }
  else
    ROS_WARN("No planner_configs/'%s' found on param server. Type 'rosparam list' in the console to see the paths. The optimization process continue, though it won't smartly tweak its parameters at restarts!!", planner.c_str());
}

////const 		& getRobotActuatedJoints() // I use a hack, which assumes that have any of these: joint and/or velocity and/or acceleration limits, i.e that the topic /robot_description_planning/joint_limits/ exists. Currently this is the only one available which shows the robot joints //TODO Laterly read in somewhere stable, like the .urdf. //TODO Find where to read the joint which stands as end effector (where the ball marker is on, in RViz)
////{
////  XmlRpc::XmlRpcValue robot_joints_XmlRpc;
////  if (ros::param::get("robot_description_planning/joint_limits", planner_parameters_XmlRpc)) 
////  {
////    return robot_joints_XmlRpc.structFromXml()
////  }
////  else
////    ROS_WARN("No robot_description_planning/joint_limits found on param server. Type 'rosparam list' in the console to see the paths. The optimization process continue, though it won't smartly tweak its parameters at restarts!!");
////}

double ModifiedBenchmarkExecutor::evaluate_plan(const robot_trajectory::RobotTrajectory& p) // kindof energy consumption
{
  int num_of_joints = p.getWayPoint(0).getVariableCount();
  const double pi = boost::math::constants::pi<double>();

  // Joints near the shoulder consume more than those near the EE
  std::vector<int> weights(num_of_joints, 0);
  for(int k = 0; k<num_of_joints; k++){
    weights[k] = num_of_joints - k;
  }
  
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
