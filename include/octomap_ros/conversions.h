/**
 * OctoMap ROS integration
 *
 * @author A. Hornung, University of Freiburg, Copyright (C) 2011-2012.
 * @see http://www.ros.org/wiki/octomap_ros
 * License: BSD
 */

/*
 * Copyright (c) 2011, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OCTOMAP_ROS_CONVERSIONS_H
#define OCTOMAP_ROS_CONVERSIONS_H

#include <octomap/octomap.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>

namespace octomap {
  /**
   * @brief Conversion from octomap::point3d_list (e.g. all occupied nodes from getOccupied()) to
   * sensor_msgs::PointCloud2
   *
   * @param points
   * @param cloud
   */
  void pointsOctomapToPointCloud2(const point3d_list& points, sensor_msgs::PointCloud2& cloud);

  /**
   * @brief Conversion from a sensor_msgs::PointCLoud2 to octomap::Pointcloud, used internally in OctoMap
   *
   * @param cloud
   * @param octomapCloud
   */
  void pointCloud2ToOctomap(const sensor_msgs::PointCloud2& cloud, Pointcloud& octomapCloud);

  /// Conversion from octomap::point3d to geometry_msgs::Point
  static inline geometry_msgs::Point pointOctomapToMsg(const point3d& octomapPt){
    geometry_msgs::Point pt;
    pt.x = octomapPt.x();
    pt.y = octomapPt.y();
    pt.z = octomapPt.z();

    return pt;
  }

  /// Conversion from geometry_msgs::Point to octomap::point3d
  static inline octomap::point3d pointMsgToOctomap(const geometry_msgs::Point& ptMsg){
    return octomap::point3d(ptMsg.x, ptMsg.y, ptMsg.z);
  }

  /// Conversion from octomap::point3d to tf::Point
  static inline tf::Point pointOctomapToTf(const point3d& octomapPt){
    return tf::Point(octomapPt.x(), octomapPt.y(), octomapPt.z());
  }

  /// Conversion from tf::Point to octomap::point3d
  static inline octomap::point3d pointTfToOctomap(const tf::Point& ptTf){
    return point3d(ptTf.x(), ptTf.y(), ptTf.z());
  }

  /// Conversion from octomap Quaternion to tf::Quaternion
  static inline tf::Quaternion quaternionOctomapToTf(const octomath::Quaternion& octomapQ){
    return tf::Quaternion(octomapQ.x(), octomapQ.y(), octomapQ.z(), octomapQ.u());
  }

  /// Conversion from tf::Quaternion to octomap Quaternion
  static inline octomath::Quaternion quaternionTfToOctomap(const tf::Quaternion& qTf){
    return octomath::Quaternion(qTf.w(), qTf.x(), qTf.y(), qTf.z());
  }

  /// Conversion from octomap::pose6f to tf::Pose
  static inline tf::Pose poseOctomapToTf(const octomap::pose6d& octomapPose){
    return tf::Pose(quaternionOctomapToTf(octomapPose.rot()), pointOctomapToTf(octomapPose.trans()));
  }

  /// Conversion from tf::Pose to octomap::pose6d
  static inline octomap::pose6d poseTfToOctomap(const tf::Pose& poseTf){
    return octomap::pose6d(pointTfToOctomap(poseTf.getOrigin()), quaternionTfToOctomap(poseTf.getRotation()));
  }





}


#endif

