#include <ros/ros.h>


#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <image_transport/image_transport.h>


#include <boost/thread.hpp>

#include <vigir_crop_decimate/crop_decimate.h>

namespace vigir_image_proc{

class CropDecimateNodelet : public nodelet::Nodelet
{
  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_in_, it_out_;
  image_transport::CameraSubscriber sub_;

  int queue_size_;

  boost::mutex connect_mutex_;
  image_transport::CameraPublisher pub_;

  CropDecimate crop_decimate_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
};

void CropDecimateNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  ros::NodeHandle nh_in (nh, "camera");
  ros::NodeHandle nh_out(nh, "camera_out");
  it_in_ .reset(new image_transport::ImageTransport(nh_in));
  it_out_.reset(new image_transport::ImageTransport(nh_out));

  // Read parameters
  private_nh.param("queue_size", queue_size_, 5);

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&CropDecimateNodelet::connectCb, this);
  ros::SubscriberStatusCallback connect_cb_info = boost::bind(&CropDecimateNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_ = it_out_->advertiseCamera("image_raw",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
}

// Handles (un)subscribing when clients (un)subscribe
void CropDecimateNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0)
    sub_.shutdown();
  else if (!sub_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_ = it_in_->subscribeCamera("image_raw", queue_size_, &CropDecimateNodelet::imageCb, this, hints);
  }
}

void CropDecimateNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  vigir_image_proc::CropDecimate::CropDecimateConfig config;

  config.decimation_x = 1;
  config.decimation_y = 1;
  config.height = 100;
  config.width =100;
  config.x_offset = 100;
  config.y_offset = 100;

  sensor_msgs::ImagePtr image_out;
  sensor_msgs::CameraInfoPtr camera_info_out;

  if (crop_decimate_.processImage(config, image_msg, info_msg, image_out, camera_info_out)){
    pub_.publish(image_out, camera_info_out);
  }
}


}

PLUGINLIB_DECLARE_CLASS (vigir_crop_decimate_nodelet, CropDecimateNodelet, vigir_image_proc::CropDecimateNodelet, nodelet::Nodelet);

