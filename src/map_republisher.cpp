/**
* \file map_republisher.cpp
* \brief Combines a static map with obstacle information.
*
* Adds obstacles to a static map and republishes the map for localization purposes.
*
* \author David Kent, WPI - davidkent@wpi.edu
* \date October 24, 2014
*/

#include "carl_navigation/map_republisher.h"

MapRepublisher::MapRepublisher()
{
  mapSubscriber = n.subscribe("map_server/map", 1, &MapRepublisher::staticMapCallback, this);
  obstacleGridSubscriber = n.subscribe("furniture_layer/obstacle_grid", 1,
                                       &MapRepublisher::obstacleCallback, this);

  mapPublisher = n.advertise<nav_msgs::OccupancyGrid>("map", 1);
  mapMetaDataPublisher = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1);

  getMapServer = n.advertiseService("static_map", &MapRepublisher::mapServiceCallback, this);

  obstacleCells.clear();
  staticMapInitialized = false;
  mapSubscribers = 0;
  metaDataSubscribers = 0;
}

bool MapRepublisher::mapServiceCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
  res.map = combinedMap;
  return true;
}

void MapRepublisher::staticMapCallback(const nav_msgs::OccupancyGrid& map)
{
  staticMap = map;
  combinedMap = staticMap;
  staticMapInitialized = true;
  for (unsigned int i = 0; i < obstacleCells.size(); i++)
  {
    combinedMap.data[obstacleCells[i].y * staticMap.info.width + obstacleCells[i].x] = -1;
  }
  for (unsigned int i = 0; i < obstacleEdges.size(); i ++)
  {
    combinedMap.data[obstacleEdges[i].y * staticMap.info.width + obstacleEdges[i].x] = 100;
  }
  mapPublisher.publish(combinedMap);
  mapMetaDataPublisher.publish(combinedMap.info);
}

void MapRepublisher::obstacleCallback(const carl_navigation::BlockedCells& grid)
{
  obstacleCells = grid.blockedCells;
  obstacleEdges = grid.edgeCells;
  if (staticMapInitialized)
  {
    combinedMap = staticMap;
    for (unsigned int i = 0; i < obstacleCells.size(); i++)
    {
      combinedMap.data[obstacleCells[i].y * staticMap.info.width + obstacleCells[i].x] = -1;
    }
    for (unsigned int i = 0; i < obstacleEdges.size(); i ++)
    {
      combinedMap.data[obstacleEdges[i].y * staticMap.info.width + obstacleEdges[i].x] = 100;
    }
    mapPublisher.publish(combinedMap);
    mapMetaDataPublisher.publish(combinedMap.info);
  }
}

void MapRepublisher::publishMap()
{
  if (staticMapInitialized)
  {
    int subscribers = mapPublisher.getNumSubscribers();
    if (subscribers != mapSubscribers)
    {
      if (subscribers > mapSubscribers)
      {
        ROS_INFO("New subscriber detected for /map, publishing map...");
        mapPublisher.publish(combinedMap);
      }
      mapSubscribers = subscribers;
    }

    subscribers = mapMetaDataPublisher.getNumSubscribers();
    if (subscribers != metaDataSubscribers)
    {
      if (subscribers > metaDataSubscribers)
      {
        mapMetaDataPublisher.publish(combinedMap.info);
      }
      metaDataSubscribers = subscribers;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_republisher");

  MapRepublisher mr;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    mr.publishMap();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
