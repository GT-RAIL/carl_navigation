/*!
 * \nav_timeout.h
 * \brief Cancels navigation goals placed after an elapsed time period
 *
 * If the robot takes longer than a specified time period to reach a nav goal, the goal will be cancelled
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date August 8, 2014
 */

#ifndef NAV_TIMEOUT_H_
#define NAV_TIMEOUT_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ActionClient;

class nav_timeout
{
public:
  /*!
   * Creates a nav_timeout object which cancels move base goals placed outside a bounded region. ROS nodes, services, and publishers
   * are created and maintained within this object.
   */
  nav_timeout();

private:
  /*!
   * Callback for the timeout timer
   */
  void timeoutCallback(const ros::TimerEvent&);

  /*!
   * Callback for navigation goal
   */
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& nav_goal);

  /*!
   * Callback for nav goal result
   */
  void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& result);

  ros::Subscriber nav_goal_in; /*!< the nav_goal_in topic */
  ros::Subscriber nav_goal_result; /*!< the nav_goal_result topic */
  ActionClient* actionClient; /*!< A handle for the move_base action client thread */
  ros::Timer timeoutTimer; /*< Timer for measuring time since goal recieved */

  //parameters
  double timeout;
};


/*!
 * Creates and runs the nav_timeout node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif //VIRTUAL_FENCE_H
