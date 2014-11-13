#include <carl_navigation/furniture_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(furniture_layer_namespace::FurnitureLayer, costmap_2d::Layer)

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

  obstacleSubscriber = nh.subscribe<carl_navigation::Obstacles>("update_furniture_layer", 1, &FurnitureLayer::updateFurnitureCallback, this);

  localizationGridPublisher = nh.advertise<carl_navigation::BlockedCells>("furniture_layer/obstacle_grid", 1);

  navigationObstacles.clear();
}

void FurnitureLayer::updateFurnitureCallback(const carl_navigation::Obstacles::ConstPtr &obs)
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
  localizationGridPublisher.publish(grid);

  updateReceived = false;
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

} // end namespace
