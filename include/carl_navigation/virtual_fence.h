/*!
 * \virtual_fence.h
 * \brief Cancels navigation goals placed outside of a bounded region
 *
 * If a navigation goal is placed outside of a defined bounded region, the move base navigation planner will be cancelled immediately.
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date July 21, 2014
 */

#ifndef VIRTUAL_FENCE_H_
#define VIRTUAL_FENCE_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ActionClient;

class virtual_fence
{
public:
  /*!
   * Creates a virtual_fence object which cancels move base goals placed outside a bounded region. ROS nodes, services, and publishers
   * are created and maintained within this object.
   */
  virtual_fence();

private:
  /*!
   * navigation goal received callback function which checks to see if the goal is within the legal bounding area and cancels the goal planner if it isn't
   *
   * \param nav_goal The navigation goal sent to the planner
   */
  void check_within_bounds(const geometry_msgs::PoseStamped::ConstPtr& nav_goal);

  ros::Subscriber nav_goal_in; /*!< the nav_goal_in topic */
  ros::Publisher fence_polygon_out; /*!< the fence_polygon_out topic */
  ActionClient* actionClient; /*!< A handle for the move_base action client thread */

  //Parameters
  double fence_x; /*!< x position of the fence */
  double fence_y; /*!< y position of the fence */
  double fence_width; /*!< width of the fence */
  double fence_height; /*!< height of the fence */
};

/*!
 * Creates and runs the virtual_fence node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif //VIRTUAL_FENCE_H
