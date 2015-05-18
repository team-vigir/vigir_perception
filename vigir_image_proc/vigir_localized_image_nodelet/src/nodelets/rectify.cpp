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

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <vigir_localized_image_proc/localized_image_proc.h>

#include <sensor_msgs/JointState.h>

namespace vigir_image_proc {

class LocalizedImageProvider : public nodelet::Nodelet
{
public:
  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

protected:

  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_in_;//, it_out_;
  image_transport::CameraSubscriber sub_;

  boost::mutex connect_mutex_;
  image_transport::CameraPublisher pub_still_;

  ros::Publisher localized_image_pub_;


  int queue_size_;
  std::string input_topic_in_camera_namespace_;

  boost::shared_ptr<tf::TransformListener> tf_listener_;

  boost::shared_ptr<vigir_image_proc::LocalizedImageProc> localized_image_proc_;


  //vigir_image_proc::CropDecimate::CropDecimateConfig crop_decimate_config_;
  bool crop_decimate_configured_;

  sensor_msgs::ImageConstPtr last_image_msg_;
  sensor_msgs::CameraInfoConstPtr last_info_msg_;
  //flor_perception_msgs::DownSampledImageRequestConstPtr last_request_;

  ros::Timer image_publish_timer_;


};

void LocalizedImageProvider::onInit()
{
  crop_decimate_configured_ = false;


  tf_listener_.reset(new tf::TransformListener());

  localized_image_proc_.reset(new vigir_image_proc::LocalizedImageProc(tf_listener_));



  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  ros::NodeHandle nh_in (nh, "camera");
  ros::NodeHandle nh_out(nh, "camera_out");
  //ros::NodeHandle nh_out_still(nh, "camera_out_still");
  //ros::NodeHandle nh_out_video(nh, "camera_out_video");
  it_in_ .reset(new image_transport::ImageTransport(nh_in));
  //it_out_.reset(new image_transport::ImageTransport(nh_out));
  //it_out_video_.reset(new image_transport::ImageTransport(nh_out_video));

  // Read parameters
  //private_nh.param("queue_size", queue_size_, 5);
  //private_nh.param("max_video_framerate", max_video_framerate_, 100.0);
  private_nh.param("input_topic_in_camera_namespace", input_topic_in_camera_namespace_, std::string("image_rect"));
  ROS_INFO("Subscribing to topic %s in camera namespace", input_topic_in_camera_namespace_.c_str());

  // Monitor whether anyone is subscribed to the output
  //image_transport::SubscriberStatusCallback connect_cb = boost::bind(&LocalizedImageNodelet::connectCb, this);
  //ros::SubscriberStatusCallback connect_cb_info = boost::bind(&LocalizedImageNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_

  ROS_INFO("LOCK AND DO CAMERA-Y STUFF");
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  localized_image_pub_ = nh_out.advertise<vigir_perception_msgs::LocalizedImage>("image_raw", 5);

  //pub_still_ = it_out_->advertiseCamera("image_raw",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
  //pub_video_ = it_out_video_->advertiseCamera("image_raw",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);

  //image_req_sub_ = nh_out.subscribe("image_request",1, &CropDecimateNodelet::imageRequestCb, this);
  //ROS_INFO("DONE CREATING ADVERTISER AND SUBSRIBING TO IMAGE REQUEST SUB");

  // For the moment, directly subscribe
  image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
  sub_ = it_in_->subscribeCamera(input_topic_in_camera_namespace_, queue_size_, &LocalizedImageProvider::imageCb, this, hints);

}

void LocalizedImageProvider::connectCb()
{

}

void LocalizedImageProvider::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                             const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  last_image_msg_ = image_msg;
  last_info_msg_ = info_msg;

  if (localized_image_pub_.getNumSubscribers() > 0){

    vigir_perception_msgs::LocalizedImagePtr localized_image_out;


     if (!localized_image_proc_->processImage(image_msg,
                                        info_msg,
                                        localized_image_out))
     {
       ROS_ERROR("Failed to generate localized image!");
       return;
     }

     localized_image_pub_.publish(localized_image_out);

  }

  /*
  // If we didn't get a request yet, do nothing
  if (!last_request_){
    return;
  }

  //Free run (direct republish) if in ALL mode
  if (last_request_->mode == flor_perception_msgs::DownSampledImageRequest::ALL){
    this->publishCroppedImage(true);
  }
  */
}


} //namespace

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( vigir_image_proc::LocalizedImageProvider, nodelet::Nodelet)

