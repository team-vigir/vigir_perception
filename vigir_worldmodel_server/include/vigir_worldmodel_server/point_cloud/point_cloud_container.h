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

#ifndef POINT_CLOUD_CONTAINER_H__
#define POINT_CLOUD_CONTAINER_H__

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <ros/time.h>

namespace vigir_worldmodel{

/**
 * Contains point cloud information along with snapshots of tf transforms for the cloud
 * timestamp. By storing these with the cloud, it can be transformed to all stored
 * coordinate systems at any time.
 */
  template <typename PointT>
  class PointCloudContainer
  {
  public:
    PointCloudContainer(boost::shared_ptr<pcl::PointCloud<PointT> > pc, const std::vector<tf::StampedTransform>& transforms)
    : pc(pc)
    , transforms(transforms)
    {}

    /**
     * Needed for instantiating temporary comparison container
     */
    PointCloudContainer()
    {}


    const tf::StampedTransform& getTransform(size_t frame_id_idx) const { return transforms[frame_id_idx]; };
    const pcl::PointCloud<PointT>& getPointcloud() const { return *pc; };
    //const boost::shared_ptr<pcl::PointCloud<PointT> >& getPointcloudPtr() const { return pc; };

    const uint64_t& getStamp() const { return pc->header.stamp; };
    void setStamp(const ros::Time& time) { pc->header.stamp = time.toNSec()/ 1e3; };


  protected:
    //pcl::PointCloud<PointT> pc;
    boost::shared_ptr<pcl::PointCloud<PointT> > pc;
    std::vector<tf::StampedTransform> transforms;
    //ros::Time stamp;
  };

}

#endif
