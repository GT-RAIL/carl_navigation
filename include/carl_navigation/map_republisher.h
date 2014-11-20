/**
 * \file map_republisher.h
 * \brief Combines a static map with obstacle information.
 *
 * Adds obstacles to a static map and republishes the map for localization purposes.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date October 24, 2014
 */

#include <carl_navigation/BlockedCells.h>
#include <nav_msgs/GetMap.h>
#include <ros/ros.h>

class MapRepublisher
{
public:
  /**
   * Constructor
   */
  MapRepublisher();

  /**
   * Republish the map or the map metadata if the number of subscribers changes
   */
  void publishMap();

private:
  //ROS publishers, subscribers, and action servers
  ros::NodeHandle n;

  ros::Subscriber mapSubscriber;
  ros::Subscriber obstacleGridSubscriber;

  ros::Publisher mapPublisher;
  ros::Publisher mapMetaDataPublisher;

  ros::ServiceServer getMapServer;

  int mapSubscribers; //number of subscribers for the map topic
  int metaDataSubscribers; //number of subscribers for the map metadata topic

  //Map
  nav_msgs::OccupancyGrid combinedMap; //combined static map and obstacles
  nav_msgs::OccupancyGrid staticMap; //static map forming the base of the combined map
  std::vector<geometry_msgs::Point> obstacleCells; //obstacle information to be added to the map
  std::vector<geometry_msgs::Point> obstacleEdges; //edge of obstacles to be added to the map
  bool staticMapInitialized; //true if the static map has been read successfully

  /**
   * Map service callback
   * @param req service request
   * @param res service response
   * @return true on success
   */
  bool mapServiceCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);

  /**
   * Callback for static map, will republish the map with obstacle data added if obstacle data
   * has been previously received
   * @param map static map from map_server
   */
  void staticMapCallback(const nav_msgs::OccupancyGrid& map);

  /**
   * Callback for the obstacle grid, will republish the map with obstacle data added to the static map
   * @param grid obstacles found by an external vision system
   */
  void obstacleCallback(const carl_navigation::BlockedCells& grid);
};
