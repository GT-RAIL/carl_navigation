/*!
 * \file rail_lab_location_server.cpp
 * \brief Action server for high-level movement commands.
 *
 * rail_lab_location_server creates an action server with pre-defined goal locations for CARL navigation.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date September 19, 2014
 */
#include <carl_navigation/rail_lab_location_server.hpp>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <sstream>

using namespace std;

rail_lab_location_server::rail_lab_location_server() :
    move_base_("move_base", true)
{
  // private node handle
  ros::NodeHandle private_nh("~");

  // grab the config file
  stringstream ss;
  ss << ros::package::getPath("carl_navigation") << "/config/rail_lab_locations.yaml";
  string file;
  private_nh.param("locations_config", file, ss.str());

  // parse the configuration file
  YAML::Node config = YAML::LoadFile(file);
  for (size_t i = 0; i < config.size(); i++)
  {
    // load the ID and name
    int id = config[i]["id"].as<int>();
    string name = config[i]["name"].as<std::string>();

    // load position information
    geometry_msgs::Pose pose;
    const YAML::Node &position = config[i]["position"];
    pose.position.x = position[0].as<float>();
    pose.position.y = position[1].as<float>();
    pose.position.z = position[2].as<float>();

    // load orientation information
    const YAML::Node &orientation = config[i]["orientation"];
    pose.orientation.x = orientation[0].as<float>();
    pose.orientation.y = orientation[1].as<float>();
    pose.orientation.z = orientation[2].as<float>();
    pose.orientation.w = orientation[3].as<float>();

    // store the location
    locations_.push_back(location(id, name, pose));
  }

  ROS_INFO("Connection to CARL low-level navigation...");
  // wait for the action server to start TODO
  //move_base_.waitForServer();

  ROS_INFO("CARL High Level Navigation Initialized with %d Locations", (int) locations_.size());
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rail_lab_location_server");

  // initialize the server
  rail_lab_location_server server;

  // continue until a ctrl-c has occurred
  ros::spin();
}
