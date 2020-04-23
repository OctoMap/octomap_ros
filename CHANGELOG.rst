^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package octomap_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2020-04-23)
------------------
* Added archive destination for static libraries (`#10 <https://github.com/OctoMap/octomap_ros/issues/10>`_)
* Fix catkin_lint issues and cmake policy CMP0038 (`#9 <https://github.com/OctoMap/octomap_ros/issues/9>`_)
* Update maintainer emails
* Contributors: Andrea Ponza, Armin Hornung, Sebastian Kasperski, Wolfgang Merkt

0.4.0 (2014-04-17)
------------------
* Dropped PCL support in favor of sensor_msgs::PointCloud2.

0.3.1 (2014-04-16)
------------------
* Fixing `#3 <https://github.com/OctoMap/octomap_ros/issues/3>`_ (explicit instantiation in conversions.cpp)

0.3.0 (2013-05-03)
------------------
* Removed deprecated OctomapROS class

0.2.6 (2013-01-05)
------------------
* fixes for cmake / catkin

0.2.5 (2012-12-08)
------------------
* catkin fixes

0.2.4 (2012-11-11)
------------------
* adding include installs
* catkin fixes
* fixing octomap_msgs dependency

0.2.3 (2012-10-29)
------------------
* catkinizing octomap_ros

0.2.2 (2012-10-01)
------------------
* catkinizing octomap_ros

0.2.1 (2012-09-04)
------------------

0.2.0 (2012-08-24)
------------------
* Further renaming, improved compat. with groovy (server won't compile yet though)
* Updated octomap_ros to new-style stack.xml
* manifest / doxygen for octomap_ros & octomap_msgs
* deprecated OctomapROS in octomap_ros => directly use octomap lib and conversions.h

