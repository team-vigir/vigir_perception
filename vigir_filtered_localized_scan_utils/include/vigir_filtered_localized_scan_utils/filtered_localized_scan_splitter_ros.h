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
#include <vigir_filtered_localized_scan_utils/filtered_localized_scan_splitter.h>

namespace vigir_filtered_localized_scan_utils
{

/**
 * @brief The FilteredLocalizedScanSplitterRos class provides
 * a ROS(topic) interface for converting FilteredLocalizedLaserScan
 * messages to point cloud representations.
 */
class FilteredLocalizedScanSplitterRos
{
public:
  FilteredLocalizedScanSplitterRos()
  {
    ros::NodeHandle pnh("~");

    pnh.param("scan_sub_queue_size", p_scan_queue_size_, 1);
    pnh.param("split_into_num_scans", p_scan_split_number_, 3);

    ROS_INFO("FilteredLocalizedScanSplitter using incoming queue size %d", p_scan_queue_size_);

    scan_pub_ = pnh.advertise<vigir_perception_msgs::FilteredLocalizedLaserScan>("scan_out", 100, false);
    scan_sub_ = pnh.subscribe("scan", p_scan_queue_size_, &FilteredLocalizedScanSplitterRos::scanCallback, this);
  }

  void scanCallback(const vigir_perception_msgs::FilteredLocalizedLaserScan& scan_in)
  {
    std::vector<vigir_perception_msgs::FilteredLocalizedLaserScan::Ptr> out_scans;



    if (converter.splitScan(scan_in,
                            out_scans,
                            p_scan_split_number_))
    {
      if (out_scans.size() != p_scan_split_number_){
        ROS_ERROR("Split scan vector size %d != desired split scan size %d", (int)out_scans.size(), (int)p_scan_split_number_);
        return;
      }else if (out_scans.size() == 0){
        ROS_ERROR("Split scan vector size is zero!");
        return;
      }

      //Publish the split scans on single topic
      for (size_t i = 0; i < out_scans.size(); ++i){
        scan_pub_.publish(out_scans[i]);
      }

    }else{
      ROS_WARN("Could not split scan, skipping.");
    }
  }


private:
  FilteredLocalizedScanSplitter converter;

  ros::Subscriber scan_sub_;
  ros::Publisher scan_pub_;

  int p_scan_queue_size_;
  int p_scan_split_number_;
  bool p_fill_in_intensity_if_not_available_;

};

}
#endif
