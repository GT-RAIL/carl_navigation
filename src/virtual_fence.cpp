/*!
 * \virtual_fence.cpp
 * \brief Cancels navigation goals placed outside of a bounded region
 *
 * If a navigation goal is placed outside of a defined bounded region, the move base navigation planner will be cancelled immediately.
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date July 21, 2014
 */

#include <carl_navigation/virtual_fence.h>

using namespace std;

virtual_fence::virtual_fence()
{
  //a private handle for this ROS node
  ros::NodeHandle node("~");

  // create the ROS topics
  nav_goal_in = node.subscribe < geometry_msgs::PoseStamped
      > ("/nav_goal", 10, &virtual_fence::check_within_bounds, this);
  fence_polygon_out = node.advertise < geometry_msgs::PolygonStamped > ("/fence_polygon", 1, true);

  // Connect to the move_base action server
  actionClient = new ActionClient("move_base", true); // create a thread to handle subscriptions.

  //read in parameters defining the region
  node.param<double>("fence_x", fence_x, 0.0);
  node.param<double>("fence_y", fence_y, 0.0);
  node.param<double>("fence_width", fence_width, 2.0);
  node.param<double>("fence_height", fence_height, 2.0);

  /*  ROS_INFO("Virtual Fence Started with x = %f, y = %f, width = %f, height = %f", fence_x, fence_y, fence_width,
      fence_height);*/

  //publish the fence as a polygon
  geometry_msgs::PolygonStamped fence_polygon;
  fence_polygon.header.frame_id = "map";

  geometry_msgs::Point32* point = new geometry_msgs::Point32();
  point->x = fence_x;
  point->y = fence_y;
  point->z = 0;
  fence_polygon.polygon.points.push_back(*point);

  point = new geometry_msgs::Point32();
  point->x = fence_x + fence_width;
  point->y = fence_y;
  point->z = 0;
  fence_polygon.polygon.points.push_back(*point);

  point = new geometry_msgs::Point32();
  point->x = fence_x + fence_width;
  point->y = fence_y + fence_height;
  point->z = 0;
  fence_polygon.polygon.points.push_back(*point);

  point = new geometry_msgs::Point32();
  point->x = fence_x;
  point->y = fence_y + fence_height;
  point->z = 0;
  fence_polygon.polygon.points.push_back(*point);

  fence_polygon_out.publish(fence_polygon);
}

void virtual_fence::check_within_bounds(const geometry_msgs::PoseStamped::ConstPtr& nav_goal)
{
  if (nav_goal->pose.position.x < fence_x || nav_goal->pose.position.x > fence_x + fence_width
      || nav_goal->pose.position.y < fence_y || nav_goal->pose.position.y > fence_y + fence_height)
  {
    //Goal placed outside of bounds. Cancel the goal.
    ROS_ERROR("Navigation goal set outside of legal bounding area. Canceling goal.");
    actionClient->waitForServer();
    actionClient->cancelAllGoals();
    ROS_ERROR("Goal cancelled.");
  }
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "virtual_fence");

  // initialize the virtual fence
  virtual_fence fence;

  ros::spin();

  return EXIT_SUCCESS;
}
