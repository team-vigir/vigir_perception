#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "crop_decimate_nodelet.cpp"

namespace vigir_image_proc{

  class CropDecimateRequesterNodelet : public CropDecimateNodelet
  {
  public:
    virtual void onInit();
    void imageRequestCb(const flor_perception_msgs::DownSampledImageRequestConstPtr& image_request_msg);

  private:
    std::string cam_request_topic_;
    ros::Publisher request_publisher_;
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

  }

  void CropDecimateRequesterNodelet::imageRequestCb(const flor_perception_msgs::DownSampledImageRequestConstPtr& image_request_msg)
  {
    ROS_INFO("got a requester request");
    last_request_ = image_request_msg;

    std_msgs::Bool request_msg;
    request_msg.data = true;

    ros::Time last_time;
    if(  last_image_msg_ != NULL )
        last_time = last_image_msg_->header.stamp ; 
    
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
