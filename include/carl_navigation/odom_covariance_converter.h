/*!
 * \odom_covariance_converter.h
 * \brief Adds covariance matrix to odometry message
 *
 * odom_covariance_converter adds a covariance matrix to odometry messages so they are compatible with robot_pose_efk.
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 16, 2014
 */

#ifndef ODOM_covariance_CONVERTER_H_
#define ODOM_covariance_CONVERTER_H_


#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <boost/assign/list_of.hpp>

class odom_covariance_converter
{
public:
  /*!
   * Creates an odom_covariance_converter object which adds covariance matrices onto Odometry messages. ROS nodes, services, and publishers
   * are created and maintained within this object.
   */
  odom_covariance_converter();

private:
  /*!
   * converter callback function.
   *
   * \param odom the message for the odom topic
   */
  void convert_cback(const nav_msgs::Odometry::ConstPtr& odom);

  ros::Subscriber odom_in; /*!< the odom_in topic */
  ros::Publisher odom_out; /*!< the odom_out topic */

  //Variables to store covariance parameters
  double cov_x;
  double cov_y;
  double cov_z;
  double cov_rx;
  double cov_ry;
  double cov_rz;
};

/*!
 * Creates and runs the odom_covariance_converter node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
