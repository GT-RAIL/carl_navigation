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
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <carl_navigation/Obstacles.h>
#include <nav_msgs/OccupancyGrid.h>
#include <carl_navigation/BlockedCells.h>

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
  void updateFurnitureCallback(const carl_navigation::Obstacles::ConstPtr &obs);
  
  ros::Subscriber obstacleSubscriber;
  ros::Publisher localizationGridPublisher;
  
  bool updateReceived;
  std::vector<carl_navigation::Obstacle> navigationObstacles; //obstacle list for the navigation map
  std::vector<carl_navigation::Obstacle> localizationObstacles; //obstacle list for the localization map

  //stored map bounds
  double prevMaxX;
  double prevMaxY;
  double prevMinX;
  double prevMinY;
};
}
#endif
