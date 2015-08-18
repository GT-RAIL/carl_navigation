^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package carl_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.12 (2015-08-18)
-------------------
* reverted changelog
* changelog updated
* Added some locations
* Contributors: David Kent, Russell Toris

0.0.11 (2015-05-06)
-------------------
* pretty world rviz
* pretty world rviz
* Contributors: Russell Toris

0.0.10 (2015-03-27)
-------------------
* pretty rviz view added
* Contributors: Russell Toris

0.0.9 (2014-12-18)
------------------
* rviz update
* launches ilab URDF
* Update .travis.yml
* Update .travis.yml
* Update .travis.yml
* Contributors: Russell Toris

0.0.8 (2014-12-02)
------------------
* Update .travis.yml
* travis now pull source
* topic name update
* propagated furniture obstacle information to local costmap as well as the global costmap
* message generation dependency
* Fixed a race condition on geting the initial furniture positions
* Adjustments to inflation amount and publisher/subscriber initialization order for the furniture_layer
* Organization/Documentation
* Navigation can now be launched to use layered costmaps for localization and navigation based on fixed furniture positions, marker-tracked furniture positions, or the original non-layered version.
* Documentation and cleanup
* Layered costmap implementation for dynamic furniture tracking
* Contributors: David Kent, Russell Toris

0.0.7 (2014-10-23)
------------------
* navigation marker fix
* Contributors: Russell Toris

0.0.6 (2014-10-03)
------------------
* no more checks
* error in pose
* tuning for rail locations
* tuning for lab locations
* tuning for lab locations
* tuning for lab locations
* Contributors: Russell Toris

0.0.5 (2014-09-23)
------------------
* parser for yaml file
* fstream included
* checks made for yaml-cpp version during compile
* Contributors: Russell Toris

0.0.4 (2014-09-22)
------------------
* added jaco_description support
* Merge branch 'develop' of github.com:WPI-RAIL/carl_navigation into develop
* travis only builds carl_description instead of entire robot
* rviz config update for new interactive marker server
* Contributors: Russell Toris, dekent

0.0.3 (2014-09-19)
------------------
* final version of location server
* build fix
* travis fix test
* message generation added
* started carl navigation
* footprint updated
* Contributors: Russell Toris

0.0.2 (2014-09-04)
------------------
* robot_pose_publisher now launched with AMCL
* Contributors: Russell Toris

0.0.1 (2014-08-27)
------------------
* fixed launch file location
* navigation launch file added
* amcl params moved to yaml
* gmapping params moved to yaml
* nav rviz config
* params updated
* navigation started
* travis fix
* gmapping parameters
* removed old code
* cleanup of readmes and such
* Merge pull request #3 from Spkordell/develop
  Get the localization map only once by making a service call
* Fixed merge conflicts
* Created navigation timeout node
* Created navigation timeout node
* Fixed some nav goals failing to cancel.
* Get the localization map only once by making a service call
* Cleaned rviz configuration
* Merge pull request #2 from Spkordell/develop
  Navigation Tuning
* Tuning
* Tuning
* Tuning
* Tuning
* Tuning
* Tuning
* tuning
* Aligned navigation boundary to new map
* Merge pull request #1 from Spkordell/develop
  Moved carl_navigation from carl_bot package to its own package
* Merge branch 'develop' of https://github.com/Spkordell/carl_navigation into develop
* Removed visual odometry from gmapping
* Tuning
* Moved carl_navigation from carl_bot package to its own package
* Initial commit
* Contributors: Russell Toris, Steven Kordell, spkordell
