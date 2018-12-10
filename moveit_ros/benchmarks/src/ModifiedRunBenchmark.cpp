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

#include <ros/ros.h>
#include <string>

#include <moveit/benchmarks/ModifiedBenchmarkOptions.h>
#include <moveit/benchmarks/ModifiedBenchmarkExecutor.h>

//#include <cstdlib> //for random numbers (to decide in how many dim make a move towards neighbour)
//#include <ctime>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "modified_moveit_run_benchmark");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Read benchmark options from param server
  moveit_ros_benchmarks::ModifiedBenchmarkOptions opts(ros::this_node::getName());
  
  // Setup benchmark server
  moveit_ros_benchmarks::ModifiedBenchmarkExecutor server;

  std::vector<std::string> plugins;
  opts.getPlannerPluginList(plugins);
  server.initialize(plugins);

  //Iterations over countdown alter it only by +-1sec and summon this node, but this node, even with a low countdown, takes (I believe) more than one second (bc of the multiples runs and queries) to return its agreement to be re-summoned again to the countdown loop. So, for the uniform random dim moves (see in Executor), initialization of a seed here should not lead to any repetition in sequences of random numbers
  std::srand(std::time(NULL));

  // Running benchmarks
  if (!server.runBenchmarks(opts))
    ROS_ERROR("Failed to run all benchmarks");
}
