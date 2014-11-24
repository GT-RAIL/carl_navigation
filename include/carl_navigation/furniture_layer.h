/*!
 * \file furniture_layer.h
 * \brief Costmap layer for representing moveable furniture.
 *
 * Furniture includes both a navigation footprint for use in path planning, and a localization footprint that represents
 * what will be seen on CARL's laser scan.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date October 22, 2014
 */

#ifndef FURNITURE_LAYER_H_
#define FURNITURE_LAYER_H_
#include <carl_navigation/BlockedCells.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <rail_ceiling/GetAllObstacles.h>
#include <rail_ceiling/Obstacles.h>
#include <tf/transform_listener.h>

namespace furniture_layer_namespace
{

class FurnitureLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  FurnitureLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  /**
  * \brief update furniture polygons for use in localization and navigation/path planning
  *
  * @param obs list of obstacle polygons with associated ids
  */
  void updateFurnitureCallback(const rail_ceiling::Obstacles::ConstPtr &obs);

  /**
  * \brief update map with initial positions of furniture obstacles
  *
  * Any updates from a furniture tracking node that occur before navigation is started will also be received using
  * this function.
  */
  void getInitialObstacles();

  ros::NodeHandle n;

  ros::Subscriber obstacleSubscriber;
  ros::Publisher localizationGridPublisher;
  ros::Publisher localObstaclesPublisher;

  ros::ServiceClient initialObstaclesClient;
  
  bool updateReceived; //flag for when a furniture position update is received
  std::vector<rail_ceiling::Obstacle> navigationObstacles; //obstacle list for the navigation map
  std::vector<rail_ceiling::Obstacle> localizationObstacles; //obstacle list for the localization map

  //stored map bounds
  double prevMaxX;
  double prevMaxY;
  double prevMinX;
  double prevMinY;
};


class FurnitureLayerLocal : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  FurnitureLayerLocal();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  /**
  * \brief update obstacle cells for filling in the local map
  *
  * @param obs list of obstacle points in the /odom frame
  */
  void updateObstaclePointsCallback(const carl_navigation::BlockedCells::ConstPtr &obs);

  ros::NodeHandle n;

  ros::Subscriber obstaclePointsSubscriber;

  std::vector<geometry_msgs::Point> obstaclePoints; //obstacle list for the navigation map
  std::vector<geometry_msgs::Point> transformedPoints; //obstacle points transformed to the odom frame

  tf::TransformListener tfListener;

  double mark_x, mark_y;

  //stored map bounds
  double prevMaxX;
  double prevMaxY;
  double prevMinX;
  double prevMinY;
};
}
#endif
