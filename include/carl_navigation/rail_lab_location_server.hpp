/*!
 * \file rail_lab_location_server.hpp
 * \brief Action server for high-level movement commands.
 *
 * rail_lab_location_server creates an action server with pre-defined goal locations for CARL navigation.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date September 19, 2014
 */

#ifndef RAIL_LAB_LOCATION_SERVER_HPP_
#define RAIL_LAB_LOCATION_SERVER_HPP_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <carl_navigation/location.hpp>
#include <carl_navigation/MoveCarlAction.h>
#include <vector>

/*!
 * \class rail_lab_location_server.
 * \brief Action server for high-level movement commands.
 *
 * The rail_lab_location_server handles the creation of the ROS action servers.
 */
class rail_lab_location_server
{
public:
  /*!
   * \brief Creates a rail_lab_location_server.
   *
   * Creates a rail_lab_location_server object that creates the necessary action servers for high level navigation.
   */
  rail_lab_location_server();

private:
  ros::NodeHandle node_; /*!< a handle for this ROS node */
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_; /*!< move_base action client */
  actionlib::SimpleActionServer<carl_navigation::MoveCarlAction> as_; /*!< main action server */
  std::vector<location> locations_; /*!< pre-defined locations */
};

/*!
 * Creates and runs the rail_lab_location_server node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
