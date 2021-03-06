/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <nodelet/loader.h>
#include <ros/ros.h>
#include "nodelet/nodelet.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "concatenate_data_node");
  ros::NodeHandle private_nh("~");

  nodelet::Loader nodelet;
  /**
 typedef std::map<std::string, std::string> M_string;
 typedef std::vector<std::string> V_string;
   * 
   *
   */
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "pointcloud_preprocessor/concatenate_data_nodelet", remap, nargv);

  ros::spin();
  return 0;
}
