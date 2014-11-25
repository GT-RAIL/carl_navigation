/*!
 * \file furniture_layer.cpp
 * \brief Costmap layer for representing moveable furniture.
 *
 * Furniture includes both a navigation footprint for use in path planning, and a localization footprint that represents
 * what will be seen on CARL's laser scan.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date October 22, 2014
 */

#include <carl_navigation/furniture_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(furniture_layer_namespace::FurnitureLayer, costmap_2d::Layer)
PLUGINLIB_EXPORT_CLASS(furniture_layer_namespace::FurnitureLayerLocal, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

using namespace std;

namespace furniture_layer_namespace
{

FurnitureLayer::FurnitureLayer() {}

void FurnitureLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &FurnitureLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
 
  updateReceived = false;

  prevMaxX = std::numeric_limits<double>::min();
  prevMaxY = std::numeric_limits<double>::min();
  prevMinX = std::numeric_limits<double>::max();
  prevMinY = std::numeric_limits<double>::max();

  localizationObstacles.clear();
  navigationObstacles.clear();

  localizationGridPublisher = n.advertise<carl_navigation::BlockedCells>("furniture_layer/obstacle_grid", 1);
  localObstaclesPublisher = n.advertise<carl_navigation::BlockedCells>("furniture_layer/local_obstacle_grid", 1);

  initialObstaclesClient = n.serviceClient<rail_ceiling::GetAllObstacles>("furniture_tracker/get_all_poses");
  initialObstaclesClient.waitForExistence();
  this->getInitialObstacles();

  obstacleSubscriber = n.subscribe<rail_ceiling::Obstacles>("furniture_layer/update_obstacles", 1, &FurnitureLayer::updateFurnitureCallback, this);
}

void FurnitureLayer::getInitialObstacles()
{
  rail_ceiling::GetAllObstaclesRequest req;
  rail_ceiling::GetAllObstaclesResponse res;
  if (!initialObstaclesClient.call(req, res))
  {
    ROS_INFO("Failed to call initial obstacle pose client.");
    return;
  }
  if (!res.localizationObstacles.empty())
  {
    //Determine whether the localization obstacle list needs to be expanded to include a previously-unseen obstacle
    int maxID = 0;
    for (unsigned int i = 0; i < res.localizationObstacles.size(); i ++)
    {
      if (res.localizationObstacles[i].id > maxID)
        maxID = res.localizationObstacles[i].id;
    }
    if (maxID >= localizationObstacles.size())
      localizationObstacles.resize(maxID + 1);
    for (unsigned int i = 0; i < res.localizationObstacles.size(); i ++)
    {
      localizationObstacles[res.localizationObstacles[i].id].polygons = res.localizationObstacles[i].polygons;
    }
    updateReceived = true;
  }
  if (!res.navigationObstacles.empty())
  {
    //Determine whether the navigation obstacle list needs to be expanded to include a previously-unseen obstacle
    int maxID = 0;
    for (unsigned int i = 0; i < res.navigationObstacles.size(); i++)
    {
      if (res.navigationObstacles[i].id > maxID)
        maxID = res.navigationObstacles[i].id;
    }
    if (maxID >= navigationObstacles.size())
      navigationObstacles.resize(maxID + 1);
    for (unsigned int i = 0; i < res.navigationObstacles.size(); i++)
    {
      navigationObstacles[res.navigationObstacles[i].id].polygons = res.navigationObstacles[i].polygons;
    }
    updateReceived = true;
  }
}

void FurnitureLayer::updateFurnitureCallback(const rail_ceiling::Obstacles::ConstPtr &obs)
{
  //update navigation obstacles
  if (!obs->navigationObstacles.empty())
  {
    //Determine whether the navigation obstacle list needs to be expanded to include a previously-unseen obstacle
    int maxID = 0;
    for (unsigned int i = 0; i < obs->navigationObstacles.size(); i++)
    {
      if (obs->navigationObstacles[i].id > maxID)
        maxID = obs->navigationObstacles[i].id;
    }

    if (maxID >= navigationObstacles.size())
      navigationObstacles.resize(maxID + 1);

    for (unsigned int i = 0; i < obs->navigationObstacles.size(); i++)
    {
      navigationObstacles[obs->navigationObstacles[i].id].polygons = obs->navigationObstacles[i].polygons;
    }
    updateReceived = true;
  }

  //update localization obstacles
  if (!obs->localizationObstacles.empty())
  {
    //Determine whether the localization obstacle list needs to be expanded to include a previously-unseen obstacle
    int maxID = 0;
    for (unsigned int i = 0; i < obs->localizationObstacles.size(); i ++)
    {
      if (obs->localizationObstacles[i].id > maxID)
        maxID = obs->localizationObstacles[i].id;
    }

    if (maxID >= localizationObstacles.size())
      localizationObstacles.resize(maxID + 1);

    for (unsigned int i = 0; i < obs->localizationObstacles.size(); i ++)
    {
      localizationObstacles[obs->localizationObstacles[i].id].polygons = obs->localizationObstacles[i].polygons;
    }

    updateReceived = true;
  }
}

void FurnitureLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void FurnitureLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void FurnitureLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
{
  if (!enabled_)
    return;

  if (!updateReceived)
    return;

  //clear furniture layer map
  resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());

  //add polygons from the localization obstacle polygon list and adjust bounds based on polygon vertices
  vector<geometry_msgs::Point> filledPoints;
  vector<geometry_msgs::Point> edgePoints;
  vector<geometry_msgs::PointStamped> localObstacles;
  for (unsigned int i = 0; i < localizationObstacles.size(); i ++)
  {
    for (unsigned int j = 0; j < localizationObstacles[i].polygons.size(); j ++)
    {
      if (!localizationObstacles[i].polygons[j].points.empty())
      {
        vector<costmap_2d::MapLocation> vertices;
        vertices.resize(localizationObstacles[i].polygons[j].points.size());
        vector<geometry_msgs::Point> mapPolygon;
        mapPolygon.resize(localizationObstacles[i].polygons[j].points.size());
        for (unsigned int k = 0; k < mapPolygon.size(); k++)
        {
          mapPolygon[k].x = localizationObstacles[i].polygons[j].points[k].x;
          mapPolygon[k].y = localizationObstacles[i].polygons[j].points[k].y;

          worldToMap(localizationObstacles[i].polygons[j].points[k].x, localizationObstacles[i].polygons[j].points[k].y, vertices[k].x, vertices[k].y);

          *min_x = min(*min_x, mapPolygon[k].x);
          *min_y = min(*min_y, mapPolygon[k].y);
          *max_x = max(*max_x, mapPolygon[k].x);
          *max_y = max(*max_y, mapPolygon[k].y);
        }
        //fill polygon, populate polygon outline to send to localization map
        if (!setConvexPolygonCost(mapPolygon, LETHAL_OBSTACLE))
        {
          ROS_INFO("Failed to fill a localization polygon");
        }
        else
        {
          vector<costmap_2d::MapLocation> polygonOutline;
          polygonOutlineCells(vertices, polygonOutline);
          for (unsigned int i = 0; i < polygonOutline.size(); i++)
          {
            geometry_msgs::Point p;
            p.x = polygonOutline[i].x;
            p.y = polygonOutline[i].y;
            edgePoints.push_back(p);
          }

          vector<costmap_2d::MapLocation> filledCells;
          convexFillCells(vertices, filledCells);
          for (unsigned int k = 0; k < filledCells.size(); k++)
          {
            geometry_msgs::Point p;
            p.x = filledCells[k].x;
            p.y = filledCells[k].y;
            filledPoints.push_back(p);
          }
        }
      }
    }
  }

  //add polygons from the navigation obstacle polygon list and adjust bounds based on polygon vertices
  for (unsigned int i = 0; i < navigationObstacles.size(); i ++)
  {
    for (unsigned int j = 0; j < navigationObstacles[i].polygons.size(); j ++)
    {
      if (!navigationObstacles[i].polygons[j].points.empty())
      {
        vector<costmap_2d::MapLocation> vertices;
        vertices.resize(navigationObstacles[i].polygons[j].points.size());
        vector<geometry_msgs::Point> mapPolygon;
        mapPolygon.resize(navigationObstacles[i].polygons[j].points.size());
        for (unsigned int k = 0; k < mapPolygon.size(); k ++)
        {
          mapPolygon[k].x = navigationObstacles[i].polygons[j].points[k].x;
          mapPolygon[k].y = navigationObstacles[i].polygons[j].points[k].y;

          worldToMap(navigationObstacles[i].polygons[j].points[k].x, navigationObstacles[i].polygons[j].points[k].y, vertices[k].x, vertices[k].y);

          *min_x = min(*min_x, mapPolygon[k].x);
          *min_y = min(*min_y, mapPolygon[k].y);
          *max_x = max(*max_x, mapPolygon[k].x);
          *max_y = max(*max_y, mapPolygon[k].y);
        }
        //fill polygon
        if (!setConvexPolygonCost(mapPolygon, LETHAL_OBSTACLE))
        {
          ROS_INFO("Failed to fill a navigation polygon");
        }
        else
        {
          //populate points for local map obstacles
          vector<costmap_2d::MapLocation> filledCells;
          convexFillCells(vertices, filledCells);
          for (unsigned int k = 0; k < filledCells.size(); k++)
          {
            geometry_msgs::PointStamped point;
            point.header.frame_id = "map";
            mapToWorld((int)filledCells[k].x, (int)filledCells[k].y, point.point.x, point.point.y);
            localObstacles.push_back(point);
          }
        }
      }
    }
  }

  prevMinX = min(*min_x, prevMinX);
  prevMinY = min(*min_y, prevMinY);
  prevMaxX = max(*max_x, prevMaxX);
  prevMaxY = max(*max_y, prevMaxY);
  *min_x = prevMinX;
  *min_y = prevMinY;
  *max_x = prevMaxX;
  *max_y = prevMaxY;

  //publish information needed for updating the localization map
  carl_navigation::BlockedCells grid;
  grid.edgeCells = edgePoints;
  grid.blockedCells = filledPoints;
  if (localizationGridPublisher.getNumSubscribers() > 0)
  {
    localizationGridPublisher.publish(grid);

    updateReceived = false;
  }
  else
    ROS_INFO("No subscribers for localizationGridPublisher yet, will republish obstacles shortly...");

  if (localObstaclesPublisher.getNumSubscribers() > 0)
  {
    carl_navigation::BlockedCells localObstaclesMsg;
    localObstaclesMsg.blockedCells.resize(localObstacles.size());

    for (unsigned int i = 0; i < localObstacles.size(); i ++)
    {
      localObstaclesMsg.blockedCells[i] = localObstacles[i].point;
    }

    localObstaclesPublisher.publish(localObstaclesMsg);
  }
  else
  {
    ROS_INFO("No local map subscribing, will republish obstacles shortly...");
    updateReceived = true;
  }

}

void FurnitureLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  //copy relevant information from this layer's costmap to the master costmap
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]);
    }
  }
}



FurnitureLayerLocal::FurnitureLayerLocal() {}

void FurnitureLayerLocal::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &FurnitureLayerLocal::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  prevMaxX = std::numeric_limits<double>::min();
  prevMaxY = std::numeric_limits<double>::min();
  prevMinX = std::numeric_limits<double>::max();
  prevMinY = std::numeric_limits<double>::max();

  obstaclePointsSubscriber = n.subscribe<carl_navigation::BlockedCells>("furniture_layer/local_obstacle_grid", 1, &FurnitureLayerLocal::updateObstaclePointsCallback, this);
}

void FurnitureLayerLocal::updateObstaclePointsCallback(const carl_navigation::BlockedCells::ConstPtr &obs)
{
  //update navigation obstacles
  if (!obs->blockedCells.empty())
  {
    obstaclePoints.clear();
    obstaclePoints = obs->blockedCells;
  }
  transformedPoints.clear();
  transformedPoints.resize(obstaclePoints.size());
}

void FurnitureLayerLocal::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
      master->getOriginX(), master->getOriginY());
}

void FurnitureLayerLocal::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void FurnitureLayerLocal::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
{
  if (!enabled_)
    return;

  resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());

  for (unsigned int i = 0; i < obstaclePoints.size(); i ++)
  {
    geometry_msgs::PointStamped inputPoint;
    inputPoint.header.frame_id = "map";
    inputPoint.point = obstaclePoints[i];
    geometry_msgs::PointStamped transformedPoint;
    tfListener.transformPoint("odom", inputPoint, transformedPoint);
    transformedPoints[i] = transformedPoint.point;
    //add some padding so the inflated portion of the obstacles gets cleared as well
    *min_x = min(*min_x, obstaclePoints[i].x - 1.0);
    *min_y = min(*min_y, obstaclePoints[i].y - 1.0);
    *max_x = max(*max_x, obstaclePoints[i].x + 1.0);
    *max_y = max(*max_y, obstaclePoints[i].y + 1.0);
  }

  *min_x = std::min(std::min(*min_x, mark_x), prevMinX);
  *min_y = std::min(std::min(*min_y, mark_y), prevMinY);
  *max_x = std::max(std::max(*max_x, mark_x), prevMaxX);
  *max_y = std::max(std::max(*max_y, mark_y), prevMaxY);
  prevMinX = *min_x;
  prevMinY = *min_y;
  prevMaxX = *max_x;
  prevMaxY = *max_y;
}


void FurnitureLayerLocal::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  for (unsigned int i = 0; i < transformedPoints.size(); i ++)
  {
    unsigned int mx, my;
    if (master_grid.worldToMap(transformedPoints[i].x, transformedPoints[i].y, mx, my))
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
}

} // end namespace
