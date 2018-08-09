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

#include <moveit/benchmarks/BenchmarkOptions.h>
//#include <moveit/benchmarks/BenchmarkExecutor.h>
#include <moveit/benchmarks/BenchmarkExecutor_hill_climb.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_run_benchmark");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Read benchmark options from param server
  moveit_ros_benchmarks::BenchmarkOptions opts(ros::this_node::getName());
  // Setup benchmark server
  moveit_ros_benchmarks::BenchmarkExecutor_hill_climb server;
  
  double v_up = 10.0;
  double r_up;
  double v_down = 0.0;
  double r_down;
  
  std::vector<std::string> plugins;
  opts.getPlannerPluginList(plugins);
  
  server.initialize(plugins, v_up);
  r_up = server.runBenchmarks(opts, v_up);

  server.initialize(plugins, v_down);
  r_down = server.runBenchmarks(opts, v_down);
  
  for(int i = 0; i<10; ++i){
    std::cout << "R_UP : " << r_up << " and R_DOWN : " << r_down << "\n\n";
    if(r_up < r_down)
    {
      v_down = (v_up+v_down)/2;
      server.initialize(plugins, v_down);
      r_down = server.runBenchmarks(opts, v_down);
    }
    else
    {
      v_up = (v_up+v_down)/2;
      server.initialize(plugins, v_up);
      r_up = server.runBenchmarks(opts, v_up);
    }
  }
}
