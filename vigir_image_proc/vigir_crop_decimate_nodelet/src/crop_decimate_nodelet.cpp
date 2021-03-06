/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of TU Darmstadt, Team ViGIR, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <image_transport/image_transport.h>


#include <boost/thread.hpp>

#include <vigir_crop_decimate/crop_decimate.h>

#include <vigir_perception_msgs/DownSampledImageRequest.h>

namespace vigir_image_proc{


  class CropDecimateNodelet : public nodelet::Nodelet
  {

  public:
    virtual void onInit();

    void connectCb();

    virtual void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg);

    virtual void imageRequestCb(const vigir_perception_msgs::DownSampledImageRequestConstPtr& image_request_msg);

    void publishTimerCb(const ros::TimerEvent& event);

    void publishCroppedImage(bool pub_as_video);

  protected:

    // ROS communication
    boost::shared_ptr<image_transport::ImageTransport> it_in_, it_out_still_, it_out_video_;
    image_transport::CameraSubscriber sub_;

    boost::mutex connect_mutex_;
    image_transport::CameraPublisher pub_still_;
    image_transport::CameraPublisher pub_video_;

    ros::Subscriber image_req_sub_;

    int queue_size_;
    double max_video_framerate_;
    std::string input_topic_in_camera_namespace_;

    CropDecimate crop_decimate_;
    vigir_image_proc::CropDecimate::CropDecimateConfig crop_decimate_config_;
    bool crop_decimate_configured_;

    sensor_msgs::ImageConstPtr last_image_msg_;
    sensor_msgs::CameraInfoConstPtr last_info_msg_;
    vigir_perception_msgs::DownSampledImageRequestConstPtr last_request_;

    ros::Timer image_publish_timer_;

  };


  void CropDecimateNodelet::onInit()
  {
    crop_decimate_configured_ = false;

    ros::NodeHandle& nh         = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    ros::NodeHandle nh_in (nh, "camera");
    ros::NodeHandle nh_out(nh, "camera_out");
    ros::NodeHandle nh_out_still(nh, "camera_out_still");
    ros::NodeHandle nh_out_video(nh, "camera_out_video");
    it_in_ .reset(new image_transport::ImageTransport(nh_in));
    it_out_still_.reset(new image_transport::ImageTransport(nh_out_still));
    it_out_video_.reset(new image_transport::ImageTransport(nh_out_video));

    // Read parameters
    private_nh.param("queue_size", queue_size_, 5);
    private_nh.param("max_video_framerate", max_video_framerate_, 100.0);
    private_nh.param("input_topic_in_camera_namespace", input_topic_in_camera_namespace_, std::string("image_raw"));
    ROS_INFO("Subscribing to topic %s in camera namespace", input_topic_in_camera_namespace_.c_str());

    // Monitor whether anyone is subscribed to the output
    image_transport::SubscriberStatusCallback connect_cb = boost::bind(&CropDecimateNodelet::connectCb, this);
    ros::SubscriberStatusCallback connect_cb_info = boost::bind(&CropDecimateNodelet::connectCb, this);
    // Make sure we don't enter connectCb() between advertising and assigning to pub_

    ROS_INFO("LOCK AND DO CAMERA-Y STUFF");
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
   
    pub_still_ = it_out_still_->advertiseCamera("image_raw",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
    pub_video_ = it_out_video_->advertiseCamera("image_raw",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);

    image_req_sub_ = nh_out.subscribe("image_request",1, &CropDecimateNodelet::imageRequestCb, this);
    ROS_INFO("DONE CREATING ADVERTISER AND SUBSRIBING TO IMAGE REQUEST SUB");
  }

  // Handles (un)subscribing when clients (un)subscribe
  void CropDecimateNodelet::connectCb()
  {
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    if ( (pub_still_.getNumSubscribers() == 0) && (pub_video_.getNumSubscribers() == 0) )
      sub_.shutdown();
    else if (!sub_)
    {
      //ROS_INFO("SANDIA TRYING TO SUBSCRIBE TO CAMERA")    ;
      image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
      sub_ = it_in_->subscribeCamera(input_topic_in_camera_namespace_, queue_size_, &CropDecimateNodelet::imageCb, this, hints);
      //ROS_INFO("SANDIA SUBSCRIBED");
    }
  }

  void CropDecimateNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
  const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    last_image_msg_ = image_msg;
    last_info_msg_ = info_msg;

    // If we didn't get a request yet, do nothing
    if (!last_request_){
      return;
    }

    //Free run (direct republish) if in ALL mode
    if (last_request_->mode == vigir_perception_msgs::DownSampledImageRequest::ALL){
      this->publishCroppedImage(true);
    }
  }

  void CropDecimateNodelet::imageRequestCb(const vigir_perception_msgs::DownSampledImageRequestConstPtr& image_request_msg)
  {

    ROS_INFO("Image requested");

    last_request_ = image_request_msg;

    crop_decimate_config_.decimation_x = image_request_msg->binning_x;
    crop_decimate_config_.decimation_y = image_request_msg->binning_y;
    crop_decimate_config_.width = image_request_msg->roi.width;
    crop_decimate_config_.height = image_request_msg->roi.height;
    crop_decimate_config_.x_offset = image_request_msg->roi.x_offset;
    crop_decimate_config_.y_offset = image_request_msg->roi.y_offset;

    crop_decimate_configured_ = true;

    if (last_request_->mode == vigir_perception_msgs::DownSampledImageRequest::ONCE){

      this->publishCroppedImage(false);

    }else if (last_request_->mode == vigir_perception_msgs::DownSampledImageRequest::PUBLISH_FREQ){
      this->publishCroppedImage(true);

      ros::NodeHandle& nh = getNodeHandle();
      
      double capped_publish_frequency = std::min( max_video_framerate_,  (double)(last_request_->publish_frequency) );

      if(capped_publish_frequency > 0.0f)
        image_publish_timer_ = nh.createTimer(ros::Duration(1.0/capped_publish_frequency), &CropDecimateNodelet::publishTimerCb, this);

    }else{
      //free run/publish always on receive
      this->publishCroppedImage(true);
    }

  }

  void CropDecimateNodelet::publishTimerCb(const ros::TimerEvent& event)
  {
    //Only actually do something if we're in PUBLISH_FREQ mode.
    if (last_request_->mode != vigir_perception_msgs::DownSampledImageRequest::PUBLISH_FREQ){
      return;
    }

    this->publishCroppedImage(true);
  }

  void CropDecimateNodelet::publishCroppedImage(bool pub_as_video)
  {
    sensor_msgs::ImagePtr image_out;
    sensor_msgs::CameraInfoPtr camera_info_out;

    // Need to make sure we have the last image/info before we try to process it.
    if(last_image_msg_ != NULL && last_info_msg_ != NULL) {
      if (crop_decimate_.processImage(crop_decimate_config_, last_image_msg_, last_info_msg_, image_out, camera_info_out)){

        if (pub_as_video){
          pub_video_.publish(image_out, camera_info_out);
        }else{
          pub_still_.publish(image_out, camera_info_out);
        }
      }
      else
      {
        ROS_INFO( "Could not publish image: could not process" );
      }

    }
    else
    {
      ROS_INFO( "Could not publish image: NULL image or NULL camera info" );
    }
  }

}

PLUGINLIB_DECLARE_CLASS (vigir_crop_decimate_nodelet, CropDecimateNodelet, vigir_image_proc::CropDecimateNodelet, nodelet::Nodelet);



