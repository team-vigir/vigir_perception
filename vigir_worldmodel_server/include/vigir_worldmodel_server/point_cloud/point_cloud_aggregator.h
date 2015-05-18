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

#ifndef POINT_CLOUD_AGGREGATOR_H__
#define POINT_CLOUD_AGGREGATOR_H__

#include <vigir_worldmodel_server/point_cloud/point_cloud_container.h>

#include <geometry_msgs/Point.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image_planar.h>

#include <pcl/io/pcd_io.h>

#include <boost/circular_buffer.hpp>

#include <boost/algorithm/string.hpp>

#include <sensor_msgs/CameraInfo.h>

#include <pcl/visualization/range_image_visualizer.h>

namespace vigir_worldmodel{

  //template <typename PointT>
  //bool pointcloud_timestamp_compare (const vigir_worldmodel::PointCloudContainer<PointT>&  i, const vigir_worldmodel::PointCloudContainer<PointT>&  j) { return (i.getStamp()<j.getStamp()); }

  template <typename PointT>
  struct PointcloudStampCompare
  {
    bool operator()(const vigir_worldmodel::PointCloudContainer<PointT>&  i, const vigir_worldmodel::PointCloudContainer<PointT>&  j)
    {
      return (i.getStamp()<j.getStamp());
    }
  };

  /**
 * Stores timestamped point clouds and associated transforms in a
 * ringbuffer. Allows reconstructions of point clouds in different frames
 * as selected tf transforms snapshotted and saved with clouds.
 * The "aggregator frames" parameter should contain a space separated list
 * of the frames of interest. They should be part of the chain from
 * root to sensor frame as only the transform root->sensor frame
 * is looked up using tf, meaning all transform on the chain are available.
 */
  template <typename PointT>
  class PointCloudAggregator
  {
  public:

    PointCloudAggregator(const boost::shared_ptr<tf::TransformListener> tf_listener, int circular_buffer_size = 4000)
    : tf_listener_(tf_listener)
    , max_storage(circular_buffer_size)
    {
      //Need to add a proper pointcloud to stamp comparison object
      boost::shared_ptr<pcl::PointCloud<PointT> > pc (new pcl::PointCloud<PointT>());
      std::vector<tf::StampedTransform> tmp;
      //tmp_comparison_container_ = PointCloudContainer<PointT>(pc, tmp);

      //pointclouds_ = boost::circular_buffer<PointCloudContainer<PointT> >(max_storage);

      ros::NodeHandle pnh("~");

      pnh.param("aggregator_frames", p_aggregator_frames_list_, std::string(""));


      boost::algorithm::split(frames_list_, p_aggregator_frames_list_, boost::is_any_of("\t "));

      size_t number_of_frames = frames_list_.size();

      for (size_t i = 0; i < number_of_frames; ++i){
        frame_to_id_map_[frames_list_[i]] = i;
        id_to_frame_map_[i] = frames_list_[i];
      }

      this->reset();
    }

    void reset()
    {
      boost::mutex::scoped_lock lock(cloud_circular_buffer_mutex_);
      last_insertion_ = ros::Time(0);
      pointclouds_.clear();
    }

    void addCloud(boost::shared_ptr<pcl::PointCloud<PointT> > cloud)
    {
      ros::Time cloud_stamp = pcl_conversions::fromPCL(cloud->header.stamp);

      {
        boost::mutex::scoped_lock lock(cloud_circular_buffer_mutex_);

        if (pointclouds_.find(cloud_stamp) != pointclouds_.end())
          return;
      }

      size_t number_of_frames = frames_list_.size();

      std::vector<tf::StampedTransform> transforms;
      transforms.resize(number_of_frames);

      try {
        tf_listener_->waitForTransform(frames_list_[0], cloud->header.frame_id, cloud_stamp, ros::Duration(0.5));
        for (size_t i = 0; i < number_of_frames; ++i){

          tf_listener_->lookupTransform(frames_list_[i], cloud->header.frame_id, cloud_stamp, transforms[i]);
        }
      } catch(tf::TransformException& ex){
        ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
      }

      boost::mutex::scoped_lock lock(cloud_circular_buffer_mutex_);

      //if (cloud_stamp > last_insertion_){
        //pointclouds_.push_back(PointCloudContainer<PointT> (cloud, transforms));
      pointclouds_.insert(std::pair<ros::Time, PointCloudContainer<PointT> >(cloud_stamp, PointCloudContainer<PointT> (cloud, transforms)) );
      //pointclouds_[cloud_stamp] = PointCloudContainer<PointT> (cloud, transforms);
      last_insertion_ = cloud_stamp;
      //}
      //else{
      //  ROS_WARN("Stamp of point cloud to be inserted older or same as previous one, not inserting.");
      //           //Difference: %ul nanoseconds", cloud->header.stamp-last_insertion_ );
      //}
    }

    bool getAggregateCloud(const boost::shared_ptr<pcl::PointCloud<PointT> >& cloud, const std::string& frame_id, size_t aggregation_size= 500)
    {
      boost::mutex::scoped_lock lock(cloud_circular_buffer_mutex_);

      size_t size = pointclouds_.size();

      int frame_id_idx = getFrameId(frame_id);

      if (size == 0 || frame_id_idx < 0){
        ROS_WARN("Couldn't create aggregate cloud. Number of cached pointclouds: %d Requested frame_id: %s", (int)size, frame_id.c_str());
        return false;
      }

      pcl::PointCloud<PointT> tmp_cloud;

      //typename boost::circular_buffer<PointCloudContainer<PointT> >::iterator it = pointclouds_.end();
      typename std::map<ros::Time, PointCloudContainer<PointT> >::const_iterator it = pointclouds_.end();


      //Access only elements that are actually there
      size_t num_clouds = std::min(size, aggregation_size);
      std::advance(it, -num_clouds);

      cloud->clear();

      for (;it != pointclouds_.end(); ++it){
        Eigen::Matrix4f sensorToWorld;


        pcl_ros::transformAsMatrix(it->second.getTransform(frame_id_idx), sensorToWorld);
        //std::cout << "\nMat:\n" << sensorToWorld << "\nframe_id: " << frame_id << "frame_id idx: " << frame_id_idx << "\n";

        pcl::transformPointCloud(it->second.getPointcloud(), tmp_cloud, sensorToWorld);

        *cloud += tmp_cloud;
      }

      //PointCloudContainer tmp(ros::Time::now());
      //std::binary_search (pointclouds_.begin(), pointclouds_.end(), tmp, pointcloud_timestamp_compare);

      cloud->header = pointclouds_.rbegin()->second.getPointcloud().header;
      cloud->header.frame_id = frame_id;

      return true;
    }

    bool getAggregateCloudBbxFiltered(boost::shared_ptr<pcl::PointCloud<PointT> >& cloud, const std::string& frame_id, const geometry_msgs::Point& bbx_min, const geometry_msgs::Point& bbx_max, double voxel_grid_size, size_t aggregation_size= 500)
    {
      ROS_DEBUG("bbx_min: %f %f %f",bbx_min.x,bbx_min.y,bbx_min.z);
      ROS_DEBUG("bbx_max: %f %f %f",bbx_max.x,bbx_max.y,bbx_max.z);

      if (getAggregateCloud(cloud, frame_id, aggregation_size)){

        crop_box_filter.setInputCloud(cloud);
        crop_box_filter.setMin(Eigen::Vector4f(bbx_min.x,bbx_min.y,bbx_min.z,1));
        crop_box_filter.setMax(Eigen::Vector4f(bbx_max.x,bbx_max.y,bbx_max.z,1));



        boost::shared_ptr<pcl::PointCloud<PointT> > cloud_tmp(new pcl::PointCloud<PointT>());
        crop_box_filter.filter(*cloud_tmp);


        if (voxel_grid_size > 0.000001){
          pcl::VoxelGrid<PointT> vox_grid;
          vox_grid.setInputCloud (cloud_tmp);
          vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
          //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
          const boost::shared_ptr<pcl::PointCloud<PointT> > tempCloud (new pcl::PointCloud<PointT>());
          vox_grid.filter (*tempCloud);
          cloud = tempCloud;
        }else{
          cloud = cloud_tmp;
        }

        return true;
      }else{
        ROS_WARN("Couldn't get aggregate point cloud in bbx cloud generator for frame_id: %s", frame_id.c_str());
        return false;
      }
    }

    /**
     * Return a single cloud from the stored circular buffer. The index variable 0 indicates the most recent cloud
     * and counts up towards older clouds in the buffer.
     */
    bool getSingleCloud(const boost::shared_ptr<pcl::PointCloud<PointT> >& cloud, const std::string& frame_id, tf::Point* origin = 0 , size_t index = 0)
    {
      boost::mutex::scoped_lock lock(cloud_circular_buffer_mutex_);
      size_t size = pointclouds_.size();

      int frame_id_idx = getFrameId(frame_id);

      if (index >= size){
        ROS_WARN("Requested cloud with index %d index, but buffer size is only %d", (int)index, (int)size);
        return false;
      }

      if (frame_id_idx < 0){
        ROS_WARN("Invalid frame_id %s", frame_id.c_str());
        return false;
      }

      //typename boost::circular_buffer<PointCloudContainer<PointT> >::reverse_iterator it = pointclouds_.rbegin();

      typename std::map<ros::Time, PointCloudContainer<PointT> >::reverse_iterator it = pointclouds_.rbegin();

      //it += index;
      std::advance(it, index);

      Eigen::Matrix4f sensorToFrame;
      pcl_ros::transformAsMatrix(it->second.getTransform(frame_id_idx), sensorToFrame);

      pcl::transformPointCloud(it->second.getPointcloud(), *cloud, sensorToFrame);
      cloud->header.frame_id = frame_id;

      if (origin != 0){
        *origin = it->second.getTransform(frame_id_idx).getOrigin();
      }

      return true;
    }

    /**
     * Return a single cloud from the stored circular buffer. The index variable 0 indicates the most recent cloud
     * and counts up towards older clouds in the buffer.
     */
    bool getSingleCloudBbxFiltered(const boost::shared_ptr<pcl::PointCloud<PointT> >& cloud, const std::string& frame_id,  const geometry_msgs::Point& bbx_min, const geometry_msgs::Point& bbx_max, tf::Point* origin = 0 , size_t index = 0)
    {
      if (this->getSingleCloud(cloud, frame_id, origin, index)){

        crop_box_filter.setInputCloud(cloud);
        crop_box_filter.setMin(Eigen::Vector4f(bbx_min.x,bbx_min.y,bbx_min.z,1));
        crop_box_filter.setMax(Eigen::Vector4f(bbx_max.x,bbx_max.y,bbx_max.z,1));

        //ROS_DEBUG("bbx_min: %f %f %f",bbx_min.x,bbx_min.y,bbx_min.z);
        //ROS_DEBUG("bbx_max: %f %f %f",bbx_max.x,bbx_max.y,bbx_max.z);

        pcl::PointCloud<PointT> cloud_tmp;
        crop_box_filter.filter(cloud_tmp);

        *cloud = cloud_tmp;

        return true;
      }else{
        return false;
      }
    }

    /**
     * Returns the point cloud with timestamp higher than that specified by req_stamp (analogous to how lower/upper bound works in stl)
     */
    bool getCloudUpperBound(pcl::PointCloud<PointT>& cloud, const std::string& frame_id, const ros::Time& req_stamp, tf::Point* origin = 0)
    {
      boost::mutex::scoped_lock lock(cloud_circular_buffer_mutex_);
      size_t size = pointclouds_.size();

      int frame_id_idx = getFrameId(frame_id);

      if (size == 0 || frame_id_idx < 0){
        return false;
      }

      //typename boost::circular_buffer<PointCloudContainer <PointT> >::iterator it;
      typename std::map<ros::Time, PointCloudContainer<PointT> >::const_iterator it;

      //If we have zero request timestamp, just return latest. Otherwise, search for first point cloud newer than req_stamp
      if (req_stamp.isZero()){
        it = pointclouds_.end();
        --it;
      }else{
        //tmp_comparison_container_.setStamp(req_stamp);
        /*
        it = std::upper_bound(pointclouds_.begin(),
        pointclouds_.end(),
        tmp_comparison_container_,
        PointcloudStampCompare<PointT>());
        */

        //uint64_t stamp = pcl_conversions::toPCL(req_stamp);

        //@TODO: Fix upper bound
        it = pointclouds_.upper_bound(req_stamp);
            //std::upper_bound(pointclouds_.begin(),
            //                  pointclouds_.end(),
            //                  req_stamp);



        //@TODO Debugging helper stuff, remove when sure it's no more needed.
        /*
          ros::Time stamp = pointclouds_.begin()->getPointcloud().header.stamp;
          std::cout << "begin ts: " << stamp.toSec() << "\n";
          stamp = (pointclouds_.end()-1)->getPointcloud().header.stamp;
          std::cout <<"end ts: " << stamp.toSec() << "\n";
          std::cout <<"req ts: " << req_stamp.toSec() << "\n";
          */

        //No cloud older than req_stamp found, aborting.
        if (it == pointclouds_.end()){
          return false;
        }
      }

      Eigen::Matrix4f sensorToFrame;
      pcl_ros::transformAsMatrix(it->second.getTransform(frame_id_idx), sensorToFrame);

      pcl::transformPointCloud(it->second.getPointcloud(), cloud, sensorToFrame);
      cloud.header.frame_id = frame_id;

      if (origin != 0){
        *origin = it->second.getTransform(frame_id_idx).getOrigin();
      }

      return true;
    }

    void getRangeImagePlanar(boost::shared_ptr<pcl::RangeImagePlanar > range_image, const sensor_msgs::CameraInfo& camera_info, const size_t aggregation_size = 500)
    {

      tf::StampedTransform world_to_camera;
      tf_listener_->lookupTransform("/world", camera_info.header.frame_id, ros::Time(0), world_to_camera);

      Eigen::Matrix4f sensorToFrame;
      pcl_ros::transformAsMatrix(world_to_camera, sensorToFrame);

      geometry_msgs::Point min;
      geometry_msgs::Point max;

      min.x = -5.0;
      min.y = -5.0;
      min.z = -5.0;
      max.x = -min.x;
      max.y = -min.y;
      max.z = -min.z;

      pcl::PointCloud<ScanPointT>::Ptr cloud (new pcl::PointCloud<ScanPointT>());

      this->getAggregateCloudBbxFiltered(cloud,"/world", min, max, 0.0, aggregation_size);

      Eigen::Affine3f test (sensorToFrame);

      std::cout << "num points: " << cloud->size() << "\n";

      range_image->createFromPointCloudWithFixedSize(*cloud,
                                                    static_cast<int>(camera_info.width),
                                                    static_cast<int>(camera_info.height),
                                                    static_cast<float>(camera_info.width*0.5),
                                                    static_cast<float>(camera_info.height*0.5f),
                                                    static_cast<float>(camera_info.K[0]),
                                                    static_cast<float>(camera_info.K[4]),
                                                    test);

      std::cout << "range image num points: " << range_image->size() << "\n";

      //static pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
      //range_image_widget.showRangeImage (*range_image);
    }

    bool hasDataWithTimestamp(const ros::Time& time)
    {
        boost::mutex::scoped_lock lock(cloud_circular_buffer_mutex_);

        if (pointclouds_.find(time) != pointclouds_.end())
          return true;

        return false;
    }

    bool hasFrame(const std::string& frame_id) const
    {
      if (frame_to_id_map_.find(frame_id) == frame_to_id_map_.end()){
        return false;
      }

      return true;
    }

    int getFrameId(const std::string& frame_id) const
    {
      std::map<std::string, size_t>::const_iterator it = frame_to_id_map_.find(frame_id);
      if (it == frame_to_id_map_.end()){
        return -1;
      }

      return it->second;
    }

    void writeCloudToFile(const std::string& name)
    {
      boost::shared_ptr<pcl::PointCloud<PointT> > cloud (new pcl::PointCloud<PointT>());
      this->getAggregateCloud(cloud, "/world");

      pcl::io::savePCDFile(name, *cloud);
    }

    size_t size() const { return pointclouds_.size(); };

  protected:

    boost::shared_ptr<tf::TransformListener> tf_listener_;

    boost::mutex cloud_circular_buffer_mutex_;
    //boost::circular_buffer<PointCloudContainer<PointT> > pointclouds_;
    std::map<ros::Time, PointCloudContainer<PointT> > pointclouds_;
    size_t max_storage;

    //PointCloudContainer<PointT> tmp_comparison_container_;

    std::map<std::string, size_t> frame_to_id_map_;
    std::map<size_t, std::string> id_to_frame_map_;

    std::string p_aggregator_frames_list_;
    std::vector<std::string> frames_list_;

    pcl::CropBox<PointT> crop_box_filter;

    ros::Time last_insertion_;
  };

}

#endif
