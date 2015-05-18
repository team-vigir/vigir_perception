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

#ifndef POINTCLOUD_SUBSCRIPTION_ADAPTER_H__
#define POINTCLOUD_SUBSCRIPTION_ADAPTER_H__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


namespace vigir_worldmodel{

template <typename PointT>
class PointCloudSubscriptionAdapter{
public:

    PointCloudSubscriptionAdapter(boost::shared_ptr<PointCloudAggregator<PointT> > point_cloud_aggregator,
                                  const std::string topic_name,
                                  int queue_size = 40)
        : point_cloud_aggregator_(point_cloud_aggregator)
    {
        subscriber_nh_ = ros::NodeHandle("");
        subscriber_nh_.setCallbackQueue(&subscriber_queue_);
        subscriber_spinner_.reset(new ros::AsyncSpinner(1, &subscriber_queue_));
        subscriber_spinner_->start();

        point_cloud_sub_ = subscriber_nh_.subscribe(topic_name, queue_size, &PointCloudSubscriptionAdapter::cloudCallback, this);
    }

    ~PointCloudSubscriptionAdapter()
    {
        subscriber_spinner_->stop();
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& stereo_cloud)
    {
        //@TODO: Switch to new PCL conversion functions
        /*
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*stereo_cloud, pcl_pc);

        pcl::PointCloud<pcl::PointXYZ> cloud;

        pcl::fromPCLPointCloud2(pcl_pc, cloud);
        */

      if (!point_cloud_aggregator_->hasDataWithTimestamp(stereo_cloud->header.stamp))
      {
        boost::shared_ptr<pcl::PointCloud<PointT> > pc_ (new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(*stereo_cloud, *pc_);
        point_cloud_aggregator_->addCloud(pc_);
      }
    }


protected:

    boost::shared_ptr<PointCloudAggregator<PointT> > point_cloud_aggregator_;

    ros::NodeHandle subscriber_nh_;
    ros::CallbackQueue subscriber_queue_;
    boost::shared_ptr<ros::AsyncSpinner> subscriber_spinner_;

    ros::Subscriber point_cloud_sub_;

};

}

#endif
