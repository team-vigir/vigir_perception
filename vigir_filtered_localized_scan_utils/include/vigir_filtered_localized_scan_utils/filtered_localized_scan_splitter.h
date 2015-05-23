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

#ifndef VIGIR_FILTERED_LOCALIZED_SCAN_SPLITTER_H_
#define VIGIR_FILTERED_LOCALIZED_SCAN_SPLITTER_H_


#include <laser_geometry/laser_geometry.h>

#include <vigir_perception_msgs/FilteredLocalizedLaserScan.h>
#include <vigir_perception_msgs/filtered_scan_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

namespace vigir_filtered_localized_scan_utils
{

/**
 * @brief The FilteredLocalizedScanSplitter class provides
 * conversion functionality for converting a
 * FilteredLocalizedLaserScan messages to point cloud
 * representations.
 */
class FilteredLocalizedScanSplitter
{
public:
  bool splitScan(const vigir_perception_msgs::FilteredLocalizedLaserScan& scan_in,
                           std::vector<vigir_perception_msgs::FilteredLocalizedLaserScan::Ptr>& out_scans,
                           size_t num_scans = 3)
  {
    if (num_scans == 0)
      return false;


    tf_transformer_.clear();

    tf::StampedTransform tf_start;
    tf::StampedTransform tf_end;

    tf::transformMsgToTF(scan_in.transform_first_ray, tf_start);
    tf::transformMsgToTF(scan_in.transform_last_ray, tf_end);

    tf_start.frame_id_ = scan_in.header.frame_id;
    tf_start.child_frame_id_ = scan_in.processed_scan.header.frame_id;
    tf_start.stamp_ = scan_in.header.stamp;

    tf_end.frame_id_ = tf_start.frame_id_;
    tf_end.child_frame_id_ = tf_start.child_frame_id_;
    tf_end.stamp_ = scan_in.processed_scan.header.stamp + ros::Duration().fromSec((scan_in.processed_scan.ranges[0].echoes.size() -1)*scan_in.processed_scan.time_increment) ;

    tf_transformer_.setTransform(tf_start);
    tf_transformer_.setTransform(tf_end);

    size_t size = scan_in.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_PREPROCESSED].echoes.size();

    size_t size_per_scan = size / num_scans;

    for (size_t i = 0; i < num_scans; ++i){

      vigir_perception_msgs::FilteredLocalizedLaserScan::Ptr scan_out(new vigir_perception_msgs::FilteredLocalizedLaserScan());

      tf::StampedTransform tf_start_split;
      tf::StampedTransform tf_end_split;

      ros::Time start_time;
      ros::Time end_time;

      // Make sure tf splitting works first, otherwise we're done
      try{
        start_time = scan_in.processed_scan.header.stamp + ros::Duration().fromSec(static_cast<float>(size_per_scan*i)*scan_in.processed_scan.time_increment);
        tf_transformer_.lookupTransform(scan_in.header.frame_id, scan_in.processed_scan.header.frame_id, start_time, tf_start_split);

        end_time = start_time + ros::Duration().fromSec((size_per_scan -1)*scan_in.processed_scan.time_increment);
        tf_transformer_.lookupTransform(scan_in.header.frame_id, scan_in.processed_scan.header.frame_id, end_time, tf_end_split);
      }catch(...){
        ROS_ERROR_STREAM("Caught exception while retrieving tf scan transforms in FilteredLocalizedScanSplitter");
        return false;
      }

      // After tf done, first fill everything that just needs trivial copying.
      scan_out->header.frame_id = scan_in.header.frame_id;
      scan_out->processed_scan.header.frame_id = scan_in.processed_scan.header.frame_id;

      scan_out->processed_scan.angle_increment = scan_in.processed_scan.angle_increment;
      scan_out->processed_scan.time_increment = scan_in.processed_scan.time_increment;
      scan_out->processed_scan.scan_time = scan_in.processed_scan.scan_time;
      scan_out->processed_scan.range_min = scan_in.processed_scan.range_min;
      scan_out->processed_scan.range_max = scan_in.processed_scan.range_max;

      // Fill data that has to be changed for split scans
      tf::transformTFToMsg(tf_start_split, scan_out->transform_first_ray);
      tf::transformTFToMsg(tf_end_split, scan_out->transform_last_ray);

      scan_out->processed_scan.angle_min = scan_in.processed_scan.angle_min + (scan_in.processed_scan.angle_increment*static_cast<float>(size_per_scan*i));
      scan_out->processed_scan.angle_max = scan_out->processed_scan.angle_min + (scan_in.processed_scan.angle_increment*static_cast<float>(size_per_scan-1));

      scan_out->header.stamp = start_time;
      scan_out->processed_scan.header.stamp = start_time;

      scan_out->processed_scan.ranges.resize(2);
      scan_out->processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_PREPROCESSED].echoes.resize(size_per_scan);
      scan_out->processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_SELF_FILTERED].echoes.resize(size_per_scan);

      bool use_intensity = (scan_in.processed_scan.intensities.size() > 0);

      if (use_intensity){
        scan_out->processed_scan.intensities.resize(1);

        scan_out->processed_scan.intensities[0].echoes.resize(size_per_scan);
      }

      std::vector<float>& points_out_preproc = scan_out->processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_PREPROCESSED].echoes;
      std::vector<float>& points_out_self_filtered = scan_out->processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_SELF_FILTERED].echoes;

      const std::vector<float>& points_in_preproc = scan_in.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_PREPROCESSED].echoes;
      const std::vector<float>& points_in_self_filtered = scan_in.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_SELF_FILTERED].echoes;

      for (size_t j = 0; j < size_per_scan; ++j){

        points_out_preproc[j] = points_in_preproc[i*size_per_scan + j];
        points_out_self_filtered[j] = points_in_self_filtered[i*size_per_scan + j];

        if (use_intensity)
          scan_out->processed_scan.intensities[0].echoes[j] = scan_in.processed_scan.intensities[0].echoes[i*size_per_scan + j];
      }

      out_scans.push_back(scan_out);

    }
    return true;
  }

private:

  tf::Transformer tf_transformer_;
};

}
#endif
