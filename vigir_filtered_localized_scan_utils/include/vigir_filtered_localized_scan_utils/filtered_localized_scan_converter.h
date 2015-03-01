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

#ifndef VIGIR_FILTERED_LOCALIZED_SCAN_CONVERTER_H_
#define VIGIR_FILTERED_LOCALIZED_SCAN_CONVERTER_H_


#include <laser_geometry/laser_geometry.h>

#include <vigir_perception_msgs/FilteredLocalizedLaserScan.h>
#include <vigir_perception_msgs/filtered_scan_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

namespace vigir_filtered_localized_scan_utils
{

class FilteredLocalizedScanConversion
{
public:
  bool convertScanToClouds(const vigir_perception_msgs::FilteredLocalizedLaserScan& scan_in,
                           sensor_msgs::PointCloud2& cloud_out,
                           sensor_msgs::PointCloud2& cloud_self_filtered_out)
  {
    vigir_perception_msgs::convertFilteredToLaserScan(scan_in,
                                                      vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_PREPROCESSED,
                                                      scan_);


    tf::StampedTransform tf_start;
    tf::StampedTransform tf_end;

    //tf_transformer_.clear();
    tf::transformMsgToTF(scan_in.transform_first_ray, tf_start);
    tf::transformMsgToTF(scan_in.transform_last_ray, tf_end);

    tf_start.frame_id_ = scan_in.header.frame_id;
    tf_start.child_frame_id_ = scan_.header.frame_id;
    tf_start.stamp_ = scan_.header.stamp;

    tf_end.frame_id_ = tf_start.frame_id_;
    tf_end.child_frame_id_ = tf_start.frame_id_;
    tf_end.stamp_ = scan_.header.stamp + ros::Duration().fromSec((scan_.ranges.size() -1)*scan_.time_increment) ;

    tf_transformer_.setTransform(tf_start);
    tf_transformer_.setTransform(tf_end);

    laser_proj_.transformLaserScanToPointCloud(tf_start.frame_id_,
                                              scan_,
                                              cloud_out,
                                              tf_transformer_,
                                              scan_.range_max,
                                              laser_geometry::channel_option::Intensity);

    scan_.ranges = scan_in.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_SELF_FILTERED].echoes;

    laser_proj_.transformLaserScanToPointCloud(tf_start.frame_id_,
                                              scan_,
                                              cloud_self_filtered_out,
                                              tf_transformer_,
                                              scan_.range_max,
                                              laser_geometry::channel_option::Intensity);

    return true;
  }




private:
  sensor_msgs::LaserScan scan_;
  laser_geometry::LaserProjection laser_proj_;
  tf::Transformer tf_transformer_;
};

}
#endif
