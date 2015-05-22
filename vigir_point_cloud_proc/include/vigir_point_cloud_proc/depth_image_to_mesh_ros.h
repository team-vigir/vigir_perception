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

#ifndef VIGIR_DEPTH_IMAGE_TO_MESH_ROS_H_
#define VIGIR_DEPTH_IMAGE_TO_MESH_ROS_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <shape_msgs/Mesh.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vigir_point_cloud_proc/depth_image_to_mesh.h>
#include <vigir_point_cloud_proc/mesh_conversions.h>
//#include <vigir_filtered_localized_scan_utils/filtered_localized_scan_converter.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_conversions.h>

namespace vigir_point_cloud_proc
{

/**
 * @brief The DepthImageToMeshRos class provides
 * a ROS(topic) interface for converting point clouds
 * messages to mesh representations.
 */
template <typename PointT>
class DepthImageToMeshRos
{
public:
  DepthImageToMeshRos()
  {
    ros::NodeHandle pnh("~");

    pnh.param("cloud_sub_queue_size", p_img_queue_size_, 1);

    ROS_INFO("DepthImageToMeshRos using queue size %d", p_img_queue_size_);

    marker_pub_ = pnh.advertise<visualization_msgs::Marker>("mesh_marker", 1, false);
    shape_pub_  = pnh.advertise<shape_msgs::Mesh>("mesh_shape", 1, false);

    //cloud_sub_ = pnh.subscribe("cloud", p_cloud_queue_size_, &DepthImageToMeshRos::cloudCallback, this);

    it_.reset(new image_transport::ImageTransport(pnh));

    image_transport::TransportHints hints("raw", ros::TransportHints(), pnh);
    sub_depth_ = it_->subscribeCamera("image_rect", p_img_queue_size_, &DepthImageToMeshRos::depthCb, this, hints);

    //cloud_to_mesh_.setVoxelFilterSize(0.025);

  }

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    // Update camera model
    model_.fromCameraInfo(info_msg);

    boost::shared_ptr<pcl::PointCloud<PointT> > cloud;

    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
      convertToPcl<uint16_t>(depth_msg, cloud, model_);
    }
    else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      convertToPcl<float>(depth_msg, cloud, model_);
    }

    depth_image_to_mesh_.setInput(cloud);
    depth_image_to_mesh_.computeMesh();


    if (depth_image_to_mesh_.computeMesh())
    {

      if (marker_pub_.getNumSubscribers() > 0){
        visualization_msgs::Marker mesh_marker;

        meshToMarkerMsg(depth_image_to_mesh_.getMesh() ,mesh_marker);
        marker_pub_.publish(mesh_marker);
      }

      if (shape_pub_.getNumSubscribers() > 0){
        shape_msgs::Mesh shape_mesh;

        meshToShapeMsg(depth_image_to_mesh_.getMesh() ,shape_mesh);
        shape_pub_.publish(shape_mesh);
      }
    }else{
      ROS_WARN("Could not generate mesh for depth image!");
    }


  }

  template<typename T>
  bool convertToPcl(const sensor_msgs::ImageConstPtr& depth_msg,
               boost::shared_ptr<pcl::PointCloud<PointT> > cloud,
               const image_geometry::PinholeCameraModel& model,
               double range_max = 0.0)
  {
    if (!cloud.get()){
      cloud.reset(new pcl::PointCloud<PointT>() );
    }

    cloud->width  = depth_msg->width;
    cloud->height = depth_msg->height;
    cloud->resize(cloud->width * cloud->height);

    float center_x = model.cx();
    float center_y = model.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = depth_image_proc::DepthTraits<T>::toMeters( T(1) );
    float constant_x = unit_scaling / model.fx();
    float constant_y = unit_scaling / model.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    //sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    //sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    //sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
    const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(T);
    for (int v = 0; v < (int)cloud->height; ++v, depth_row += row_step)
    {
      for (int u = 0; u < (int)cloud->width; ++u) //++iter_x, ++iter_y, ++iter_z)
      {
        T depth = depth_row[u];

        PointT& point = (*cloud)(u,v);

        // Missing points denoted by NaNs
        if (!depth_image_proc::DepthTraits<T>::valid(depth))
        {
          if (range_max != 0.0)
          {
            depth = depth_image_proc::DepthTraits<T>::fromMeters(range_max);
          }
          else
          {
            //*iter_x = *iter_y = *iter_z = bad_point;
            point.x = bad_point;
            point.y = bad_point;
            point.z = bad_point;
            continue;
          }
        }

        // Fill in XYZ
        point.x = (u - center_x) * depth * constant_x;
        point.y = (v - center_y) * depth * constant_y;
        point.z = depth_image_proc::DepthTraits<T>::toMeters(depth);
      }
    }


  }

  /*
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
  {
    boost::shared_ptr<pcl::PointCloud<PointT> > pc (new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_in, *pc);

    cloud_to_mesh_.setInput(pc);

    if (cloud_to_mesh_.computeMesh())
    {

      if (marker_pub_.getNumSubscribers() > 0){
        visualization_msgs::Marker mesh_marker;

        meshToMarkerMsg(cloud_to_mesh_.getMesh() ,mesh_marker);
        marker_pub_.publish(mesh_marker);
      }

      if (shape_pub_.getNumSubscribers() > 0){
        shape_msgs::Mesh shape_mesh;

        meshToShapeMsg(cloud_to_mesh_.getMesh() ,shape_mesh);
        shape_pub_.publish(shape_mesh);
      }
    }else{
      ROS_WARN("Could not generate mesh for point cloud!");
    }
  }
  */



private:

  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_depth_;

  image_geometry::PinholeCameraModel model_;

  ros::Publisher marker_pub_;
  ros::Publisher shape_pub_;

  sensor_msgs::PointCloud2 cloud_out_;
  sensor_msgs::PointCloud2 cloud_self_filtered_out;

  int p_img_queue_size_;

  DepthImageToMesh<PointT> depth_image_to_mesh_;

};

}
#endif
