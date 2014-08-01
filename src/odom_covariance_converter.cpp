/*!
 * \odom_covariance_converter.cpp
 * \brief Adds covariance matrix to odometry message
 *
 * odom_covariance_converter adds a covariance matrix to odometry messages so they are compatible with robot_pose_efk.
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 16, 2014
 */

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <carl_navigation/odom_covariance_converter.h>
#include <boost/assign/list_of.hpp>

using namespace std;

odom_covariance_converter::odom_covariance_converter()
{
  //a private handle for this ROS node
  ros::NodeHandle node("~");

  // create the ROS topics
  odom_in = node.subscribe<nav_msgs::Odometry>("odom_in", 10, &odom_covariance_converter::convert_cback, this);
  odom_out = node.advertise<nav_msgs::Odometry>("odom_out", 10);

  // read in covariance parameters
  const double defaultCov = 1e9;
  node.param<double>("cov_x", cov_x, defaultCov);
  node.param<double>("cov_y", cov_y, defaultCov);
  node.param<double>("cov_z", cov_z,defaultCov);
  node.param<double>("cov_rx", cov_rx, defaultCov);
  node.param<double>("cov_ry", cov_ry, defaultCov);
  node.param<double>("cov_rz", cov_rz, defaultCov);

  ROS_INFO("Odometry covariance Converter Started");

  ROS_INFO("Covarience parameters [%f, %f, %f, %f, %f, %f]", cov_x, cov_y, cov_z, cov_rx, cov_ry, cov_rz);
}

void odom_covariance_converter::convert_cback(const nav_msgs::Odometry::ConstPtr& odom)
{
  nav_msgs::Odometry odometry = *odom;
  odometry.pose.covariance =  boost::assign::list_of (cov_x)  (0)    (0)    (0)     (0)     (0)
                                                       (0)  (cov_y)  (0)    (0)     (0)     (0)
                                                       (0)    (0)  (cov_z)  (0)     (0)     (0)
                                                       (0)    (0)    (0)  (cov_rx)  (0)     (0)
                                                       (0)    (0)    (0)    (0)   (cov_ry)  (0)
                                                       (0)    (0)    (0)    (0)     (0)   (cov_rz);
  odom_out.publish(odometry);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "odom_covariance_converter");

  // initialize the converter
  odom_covariance_converter converter;

  ros::spin();

  return EXIT_SUCCESS;
}
