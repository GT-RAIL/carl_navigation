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
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <sstream>

using namespace std;

rail_lab_location_server::rail_lab_location_server() :
    move_base_("move_base", true), as_(node_, "move_carl", boost::bind(&rail_lab_location_server::execute, this, _1),
                                       false)
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
  // wait for the action server to start
  move_base_.waitForServer();

  // start the action server
  as_.start();

  ROS_INFO("CARL High Level Navigation Initialized with %d Locations", (int ) locations_.size());
}

void rail_lab_location_server::execute(const carl_navigation::MoveCarlGoalConstPtr &goal)
{
  // search for the location
  int location_id = goal->location;
  int index = -1;
  for (uint i = 0; i < locations_.size(); i++)
  {
    if (locations_.at(i).get_id() == location_id)
    {
      index = i;
      break;
    }
  }

  // attempt to run the navigation
  bool success = false;
  if (index >= 0)
  {
    ROS_INFO("Attempting to move CARL to '%s'", locations_[index].get_name().c_str());

    // setup the goal
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose = locations_[index].get_pose();

    // send the goal
    move_base_.sendGoal(goal, boost::bind(&rail_lab_location_server::done, this, _1, _2),
                        boost::bind(&rail_lab_location_server::active, this),
                        boost::bind(&rail_lab_location_server::feedback, this, _1));

    while (!move_base_.getState().isDone())
    {
      // check if we should cancel
      if (as_.isPreemptRequested() || as_.isNewGoalAvailable() || !ros::ok())
      {
        ROS_INFO("Canceling move CARL goals...");
        move_base_.cancelAllGoals();
        as_.setPreempted();
      }
    }
  }
  else
  {
    ROS_INFO("Unknown location ID.");
    as_.setAborted(result_);
  }
}

void rail_lab_location_server::done(const actionlib::SimpleClientGoalState& state,
                                    const move_base_msgs::MoveBaseResultConstPtr &result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Move CARL Succeeded");
    as_.setSucceeded(result_);
  }
  else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    ROS_INFO("Low-level navigation canceled.");
    as_.setPreempted(result_);
  }
  else
  {
    ROS_INFO("Move CARL Failed");
    as_.setAborted(result_);
  }
}

void rail_lab_location_server::active()
{
  ROS_INFO("Low-level navigation goal activated.");
}

void rail_lab_location_server::feedback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
  // forward the feedback
  feedback_.base_position = feedback->base_position;
  as_.publishFeedback(feedback_);
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
