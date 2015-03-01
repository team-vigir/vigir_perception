//=================================================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
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

#ifndef VIGIR_FILTERED_LOCALIZED_SCAN_CONVERTER_ROS_H_
#define VIGIR_FILTERED_LOCALIZED_SCAN_CONVERTER_ROS_H_

#include <ros/ros.h>
#include <vigir_filtered_localized_scan_utils/filtered_localized_scan_converter.h>

namespace vigir_filtered_localized_scan_utils
{

/**
 * @brief The FilteredLocalizedScanConversionRos class provides
 * a ROS(topic) interface for converting FilteredLocalizedLaserScan
 * messages to point cloud representations.
 */
class FilteredLocalizedScanConversionRos
{
public:
  FilteredLocalizedScanConversionRos()
  {
    ros::NodeHandle pnh("~");

    cloud_pub_              = pnh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1, false);
    cloud_self_filtered_pub_= pnh.advertise<sensor_msgs::PointCloud2>("cloud_self_filtered_out", 1, false);
    scan_sub_ = pnh.subscribe("scan", 1, &FilteredLocalizedScanConversionRos::scanCallback, this);

  }

  void scanCallback(const vigir_perception_msgs::FilteredLocalizedLaserScan& scan_in)
  {
    if (converter.convertScanToClouds(scan_in, cloud_out_, cloud_self_filtered_out))
    {
      cloud_pub_.publish(cloud_out_);
      cloud_self_filtered_pub_.publish(cloud_self_filtered_out);
    }else{
      ROS_WARN("Could not convert scan to cloud, skipping.");
    }
  }


private:
  FilteredLocalizedScanConversion converter;
  //sensor_msgs::LaserScan scan_;
  //laser_geometry::LaserProjection laser_proj_;
  ros::Subscriber scan_sub_;
  ros::Publisher cloud_pub_;
  ros::Publisher cloud_self_filtered_pub_;

  sensor_msgs::PointCloud2 cloud_out_;
  sensor_msgs::PointCloud2 cloud_self_filtered_out;

};

}
#endif
