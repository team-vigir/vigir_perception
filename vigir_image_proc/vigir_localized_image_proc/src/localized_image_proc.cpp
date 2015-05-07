//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
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

#include <boost/make_shared.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <vigir_localized_image_proc/localized_image_proc.h>

namespace vigir_image_proc {

using namespace cv_bridge; // CvImage, toCvShare


LocalizedImageProc::LocalizedImageProc(boost::shared_ptr<tf::Transformer> transformer, const std::string target_frame)
  : transformer_(transformer)
  , target_frame_(target_frame)
{}


bool LocalizedImageProc::processImage(
                      const sensor_msgs::ImageConstPtr& image_msg,
                      const sensor_msgs::CameraInfoConstPtr& info_msg,
                      vigir_perception_msgs::LocalizedImagePtr& localized_image_msg_out)
{

  if (!transformer_.get()){
    ROS_ERROR("Transformer in localized image proc is Null!");
    return false;
  }

  tf::StampedTransform transform_camera_to_world;

  try
  {
    if(transformer_->waitForTransform(target_frame_, image_msg->header.frame_id, image_msg->header.stamp, ros::Duration(0.5)))
    {
      transformer_->lookupTransform(target_frame_, image_msg->header.frame_id, image_msg->header.stamp, transform_camera_to_world);
    }
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << "; quitting callback");
    return false;
  }

  localized_image_msg_out.reset(new vigir_perception_msgs::LocalizedImage());

  // Set transform for outgoing message
  tf::transformTFToMsg(transform_camera_to_world, localized_image_msg_out->camera_pose_world_frame);

  //Set image message
  localized_image_msg_out->image = *image_msg;

  //Set camera info
  localized_image_msg_out->camera_info = *info_msg;

  return true;

}
} // namespace vigir_image_proc

