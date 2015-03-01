//=================================================================================================
// Copyright (c) 2013, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef POINT_CLOUD_VISUALIZATION_H__
#define POINT_CLOUD_VISUALIZATION_H__

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>

namespace vigir_worldmodel{

/**
 * Contains point cloud information
 */
  class PointCloudVisualization
  {
  public:
    PointCloudVisualization(ros::NodeHandle& nh)
    {
      pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("pointcloud_vis", 1, false);
    }

    template <class PointT>
    void publishVis(const pcl::PointCloud<PointT>& cloud)
    {
      pcl::toROSMsg(cloud, pc_ros_);

      pc_pub_.publish(pc_ros_);
    }

    bool hasSubscribers() const{
      return (pc_pub_.getNumSubscribers() > 0);
    }

  protected:
    ros::Publisher pc_pub_;

    sensor_msgs::PointCloud2 pc_ros_;

  };

}

#endif
