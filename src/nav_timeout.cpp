/*!
 * \nav_timeout.h
 * \brief Cancels navigation goals placed after an elapsed time period
 *
 * If the robot takes longer than a specified time period to reach a nav goal, the goal will be cancelled
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date August 8, 2014
 */

#include <carl_navigation/nav_timeout.h>

using namespace std;

nav_timeout::nav_timeout()
{
  //a private handle for this ROS node
  ros::NodeHandle node("~");

  // create the ROS topics
  nav_goal_in = node.subscribe < geometry_msgs::PoseStamped > ("nav_goal", 10, &nav_timeout::goalCallback, this);
  nav_goal_result = node.subscribe < move_base_msgs::MoveBaseActionResult
      > ("nav_goal_result", 10, &nav_timeout::goalResultCallback, this);

  // Connect to the move_base action server
  actionClient = new ActionClient("move_base", true); // create a thread to handle subscriptions.

  //read in parameters defining the region
  node.param<double>("timeout", timeout, 60.0);

  //create the timer
  timeoutTimer = node.createTimer(ros::Duration(timeout), &nav_timeout::timeoutCallback, this);
  timeoutTimer.stop();
}

void nav_timeout::timeoutCallback(const ros::TimerEvent&)
{
  timeoutTimer.stop();
  ROS_ERROR("Navigation Goal Timeout");
  actionClient->waitForServer();
  actionClient->cancelAllGoals();
  ROS_ERROR("Goal cancelled");
}

void nav_timeout::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& nav_goal)
{
  timeoutTimer.stop();
  timeoutTimer.start();
}

void nav_timeout::goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& result)
{
  //Either the planner failed to find a path to the goal, the goal was reached, the goal was cancelled, or a new goal was received. Regardless, stop the timer.
  timeoutTimer.stop();
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "nav_timeout");

  // initialize the virtual fence
  nav_timeout navTimeout;

  ros::spin();

  return EXIT_SUCCESS;
}
