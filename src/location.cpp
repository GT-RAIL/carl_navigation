/*!
 * \file location.cpp
 * \brief Location information.
 *
 * A location has a pose, name, and ID.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date September 19, 2014
 */

#include <carl_navigation/location.hpp>

location::location(int id, std::string name, geometry_msgs::Pose pose) :
    name_(name), pose_(pose)
{
  this->id_ = id;
}

int location::get_id()
{
  return this->id_;
}

std::string &location::get_name()
{
  return this->name_;
}

geometry_msgs::Pose &location::get_pose()
{
  return this->pose_;
}
