#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "crop_decimate_nodelet.cpp"

namespace vigir_image_proc{

  class CropDecimateRequesterNodelet : public CropDecimateNodelet
  {
  public:
    virtual void onInit();
    void imageRequestCb(const flor_perception_msgs::DownSampledImageRequestConstPtr& image_request_msg);
    void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  void connectCb();

  private:
    std::string cam_request_topic_;
    ros::Publisher request_publisher_;
    bool img_requested;
  }; 

  void CropDecimateRequesterNodelet::onInit()
  { 
    CropDecimateNodelet::onInit();


    //need a check if cam_request_topic was not set  
  
    ros::NodeHandle& nh         = getNodeHandle();
    ros::NodeHandle nh_in (nh, "camera");
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    max_video_framerate_ = 0; // Force no video

    private_nh.param("cam_request_topic", cam_request_topic_, (std::string)"");
    request_publisher_ = nh_in.advertise<std_msgs::Bool>(cam_request_topic_, 5);
    img_requested = false;
  }
  
    // Handles (un)subscribing when clients (un)subscribe
  void CropDecimateRequesterNodelet::connectCb()
  {
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    if ( (pub_still_.getNumSubscribers() == 0) && (pub_video_.getNumSubscribers() == 0) )
      sub_.shutdown();
    else if (!sub_)
    {
      image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
      sub_ = it_in_->subscribeCamera("image_raw", queue_size_, &CropDecimateRequesterNodelet::imageCb, this, hints);
      ROS_INFO("subscribed to camera");
    }
  }
  
    void CropDecimateRequesterNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
  const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    last_image_msg_ = image_msg;
    last_info_msg_ = info_msg;
        
    ROS_INFO( "Image recieved, requested. Calling publish" );

    // If we didn't get a request yet, do nothing
    if (!last_request_){
      return;
    }

    // all images should be published if requested
    if( img_requested ) 
    {
        ROS_INFO( "Image recieved, requested. Calling publish" );
        this->publishCroppedImage(false);
        img_requested = false;
    }

  }

  void CropDecimateRequesterNodelet::imageRequestCb(const flor_perception_msgs::DownSampledImageRequestConstPtr& image_request_msg)
  {
    ROS_INFO("got a requester request");
    last_request_ = image_request_msg;

    crop_decimate_config_.decimation_x = image_request_msg->binning_x;
    crop_decimate_config_.decimation_y = image_request_msg->binning_y;
    crop_decimate_config_.width = image_request_msg->roi.width;
    crop_decimate_config_.height = image_request_msg->roi.height;
    crop_decimate_config_.x_offset = image_request_msg->roi.x_offset;
    crop_decimate_config_.y_offset = image_request_msg->roi.y_offset;
    
    std_msgs::Bool request_msg;
    
    request_msg.data = true;

    ros::Time last_time;
    if(  last_image_msg_ != NULL )
        last_time = last_image_msg_->header.stamp ; 
   
    img_requested = true; 
    request_publisher_.publish(request_msg);
    
//    while( last_image_msg_ == NULL || last_time == last_image_msg_->header.stamp )
//    {
//       if(last_image_msg_ == NULL )
//        ROS_INFO("no image, waiting for new one");
//       else
//        ROS_INFO("waiting for a new image %d.%d vs %d.%d", last_time.sec, last_time.nsec, last_image_msg_->header.stamp.sec, last_image_msg_->header.stamp.nsec); 
//       ros::Duration(0.5).sleep(); // sleep for half a second 
//    }

//    CropDecimateNodelet::imageRequestCb( image_request_msg );
  }


}  

PLUGINLIB_DECLARE_CLASS (vigir_crop_decimate_requester_nodelet, CropDecimateRequesterNodelet, vigir_image_proc::CropDecimateRequesterNodelet, nodelet::Nodelet);
