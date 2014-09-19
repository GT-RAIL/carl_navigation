/*!
 * \file location.hpp
 * \brief Location information.
 *
 * A location has a pose, name, and ID.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date September 19, 2014
 */

#ifndef LOCATION_HPP_
#define LOCATION_HPP_

#include <geometry_msgs/Pose.h>

/*!
 * \class location
 * \brief Location information.
 *
 * A location has a pose, name, and ID.
 */
class location
{
public:
  /*!
   * \brief Creates a location.
   *
   * Creates a location with the given information.
   *
   * @param id The location ID.
   * @param name The location name.
   * @param pose The location position.
   */
  location(int id, std::string name, geometry_msgs::Pose pose);

  /**
   * Get the location ID.
   *
   * @return The location ID.
   */
  int get_id();

  /**
   * Get the location name.
   *
   * @return The location name.
   */
  std::string &get_name();

  /**
   * Get the location pose.
   *
   * @return The location pose.
   */
  geometry_msgs::Pose &get_pose();

private:
  int id_; /*! The location ID */
  std::string name_; /*! The location name */
  geometry_msgs::Pose pose_; /*! The location pose */
};

#endif
